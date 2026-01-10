#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_srvs/srv/empty.hpp>

#include <deque>
#include <string>
#include <unordered_map>
#include <vector>
#include <optional>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>

namespace
{
    struct Sample
    {
        rclcpp::Time t;
        double q;
    };

    struct Rgb
    {
        float r;
        float g;
        float b;
    };

    Rgb hsvToRgb(float h, float s, float v)
    {
        const float c = v * s;
        const float h_nonneg = (h < 0.0f) ? 0.0f : h;
        const float h_prime = std::fmod(h_nonneg, 1.0f) * 6.0f;
        const float x = c * (1.0f - std::fabs(std::fmod(h_prime, 2.0f) - 1.0f));

        float r1 = 0.0f;
        float g1 = 0.0f;
        float b1 = 0.0f;

        if (0.0f <= h_prime && h_prime < 1.0f)
        {
            r1 = c;
            g1 = x;
            b1 = 0.0f;
        }
        else if (1.0f <= h_prime && h_prime < 2.0f)
        {
            r1 = x;
            g1 = c;
            b1 = 0.0f;
        }
        else if (2.0f <= h_prime && h_prime < 3.0f)
        {
            r1 = 0.0f;
            g1 = c;
            b1 = x;
        }
        else if (3.0f <= h_prime && h_prime < 4.0f)
        {
            r1 = 0.0f;
            g1 = x;
            b1 = c;
        }
        else if (4.0f <= h_prime && h_prime < 5.0f)
        {
            r1 = x;
            g1 = 0.0f;
            b1 = c;
        }
        else
        {
            r1 = c;
            g1 = 0.0f;
            b1 = x;
        }

        const float m = v - c;
        return {r1 + m, g1 + m, b1 + m};
    }

} // namespace

class JointTrajectoryVizNode : public rclcpp::Node
{
public:
    JointTrajectoryVizNode() : Node("joint_trajectory_viz_node")
    {
        this->declare_parameter<std::string>("plot_frame", "base_link");
        this->declare_parameter<int>("max_points", 2000);
        this->declare_parameter<double>("time_window_sec", 0.0);
        this->declare_parameter<double>("publish_rate_hz", 10.0);

        this->declare_parameter<double>("time_scale", 0.2);
        this->declare_parameter<double>("value_scale", 1.0);
        this->declare_parameter<double>("value_center", 0.0);
        this->declare_parameter<double>("joint_z_offset", 0.05);
        this->declare_parameter<double>("line_width", 0.004);

        this->declare_parameter<int>("sample_stride", 1);
        this->declare_parameter<double>("min_sample_dt_sec", 0.0);

        this->declare_parameter<bool>("show_commanded", false);
        this->declare_parameter<std::string>("commanded_topic", "/joint_trajectory_controller/joint_trajectory");

        this->declare_parameter<bool>("show_labels", false);
        this->declare_parameter<bool>("clear_service", true);

        plot_frame_ = this->get_parameter("plot_frame").as_string();
        max_points_ = static_cast<int>(this->get_parameter("max_points").as_int());
        if (max_points_ < 1)
        {
            max_points_ = 1;
        }

        time_window_sec_ = this->get_parameter("time_window_sec").as_double();
        if (time_window_sec_ < 0.0)
        {
            time_window_sec_ = 0.0;
        }

        publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_double();
        if (!(std::isfinite(publish_rate_hz_) && publish_rate_hz_ > 0.0))
        {
            publish_rate_hz_ = 10.0;
        }

        time_scale_ = this->get_parameter("time_scale").as_double();
        value_scale_ = this->get_parameter("value_scale").as_double();
        value_center_ = this->get_parameter("value_center").as_double();
        joint_z_offset_ = this->get_parameter("joint_z_offset").as_double();
        line_width_ = this->get_parameter("line_width").as_double();

        sample_stride_ = static_cast<int>(this->get_parameter("sample_stride").as_int());
        if (sample_stride_ < 1)
        {
            sample_stride_ = 1;
        }

        min_sample_dt_sec_ = this->get_parameter("min_sample_dt_sec").as_double();
        if (min_sample_dt_sec_ < 0.0)
        {
            min_sample_dt_sec_ = 0.0;
        }

        show_commanded_ = this->get_parameter("show_commanded").as_bool();
        commanded_topic_ = this->get_parameter("commanded_topic").as_string();

        show_labels_ = this->get_parameter("show_labels").as_bool();
        clear_service_enabled_ = this->get_parameter("clear_service").as_bool();

        viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/joint_traj_viz", rclcpp::QoS(10));

        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states",
            rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::JointState::SharedPtr msg)
            {
                onJointState(*msg);
            });

        if (show_commanded_)
        {
            traj_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
                commanded_topic_,
                rclcpp::QoS(10),
                [this](trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
                {
                    onCommandedTrajectory(*msg);
                });
            RCLCPP_INFO(get_logger(), "Subscribing commanded trajectory: %s", commanded_topic_.c_str());
        }

        if (clear_service_enabled_)
        {
            clear_srv_ = this->create_service<std_srvs::srv::Empty>(
                "/clear_joint_traj_viz",
                [this](
                    const std::shared_ptr<std_srvs::srv::Empty::Request>,
                    std::shared_ptr<std_srvs::srv::Empty::Response>)
                {
                    clearAll();
                });
        }

        const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / publish_rate_hz_));
        publish_timer_ = this->create_wall_timer(period, [this]()
                                                 { publishMarkers(); });

        RCLCPP_INFO(get_logger(), "joint_trajectory_viz_node ready. Publishing /joint_traj_viz (MarkerArray)");
    }

private:
    void registerJointIfNeeded(const std::string &name)
    {
        if (joint_to_z_index_.find(name) != joint_to_z_index_.end())
        {
            return;
        }
        const std::size_t idx = joint_order_.size();
        joint_order_.push_back(name);
        joint_to_z_index_[name] = idx;

        actual_[name] = {};
        commanded_[name] = {};
        last_pushed_time_[name] = std::nullopt;
        pushed_count_[name] = 0;

        RCLCPP_INFO(get_logger(), "Tracking joint '%s' (index=%zu)", name.c_str(), idx);
    }

    bool shouldPushSample(const std::string &name, const rclcpp::Time &t)
    {
        auto &cnt = pushed_count_[name];
        cnt++;

        const bool stride_ok = (sample_stride_ <= 1) ? true : ((cnt % static_cast<uint64_t>(sample_stride_)) == 0);
        if (!stride_ok)
        {
            return false;
        }

        if (min_sample_dt_sec_ <= 0.0)
        {
            return true;
        }

        auto &last = last_pushed_time_[name];
        if (!last.has_value())
        {
            last = t;
            return true;
        }

        const double dt = (t - last.value()).seconds();
        if (dt >= min_sample_dt_sec_)
        {
            last = t;
            return true;
        }

        return false;
    }

    void trimDeque(std::deque<Sample> &buf, const rclcpp::Time &now)
    {
        if (time_window_sec_ > 0.0)
        {
            while (!buf.empty() && (now - buf.front().t).seconds() > time_window_sec_)
            {
                buf.pop_front();
            }
        }

        while (static_cast<int>(buf.size()) > max_points_)
        {
            buf.pop_front();
        }
    }

    void onJointState(const sensor_msgs::msg::JointState &msg)
    {
        const auto now = this->now();

        if (!t0_.has_value())
        {
            t0_ = now;
        }

        const auto n = std::min(msg.name.size(), msg.position.size());
        for (std::size_t i = 0; i < n; ++i)
        {
            const auto &jn = msg.name[i];
            const double q = msg.position[i];

            registerJointIfNeeded(jn);

            if (!std::isfinite(q))
            {
                continue;
            }

            if (!shouldPushSample(jn, now))
            {
                continue;
            }

            auto &buf = actual_[jn];
            buf.push_back(Sample{now, q});
            trimDeque(buf, now);
        }
    }

    void onCommandedTrajectory(const trajectory_msgs::msg::JointTrajectory &msg)
    {
        if (!show_commanded_)
        {
            return;
        }

        if (msg.joint_names.empty() || msg.points.empty())
        {
            return;
        }

        const auto now = this->now();
        const bool stamp_valid = (msg.header.stamp.nanosec != 0u) || (msg.header.stamp.sec != 0);
        const rclcpp::Time base = stamp_valid ? rclcpp::Time(msg.header.stamp) : now;

        // Replace commanded curves with latest command.
        for (const auto &jn : msg.joint_names)
        {
            registerJointIfNeeded(jn);
            commanded_[jn].clear();
        }

        for (const auto &pt : msg.points)
        {
            const auto t = base + rclcpp::Duration(pt.time_from_start);

            const auto m = std::min(msg.joint_names.size(), pt.positions.size());
            for (std::size_t j = 0; j < m; ++j)
            {
                const auto &jn = msg.joint_names[j];
                const double q = pt.positions[j];
                if (!std::isfinite(q))
                {
                    continue;
                }
                commanded_[jn].push_back(Sample{t, q});
            }
        }

        for (const auto &jn : msg.joint_names)
        {
            trimDeque(commanded_[jn], now);
        }

        if (!t0_.has_value())
        {
            t0_ = now;
        }
    }

    std_msgs::msg::ColorRGBA colorForIndex(std::size_t idx, float alpha) const
    {
        const float h = (joint_order_.empty()) ? 0.0f : (static_cast<float>(idx) / static_cast<float>(joint_order_.size()));
        const auto rgb = hsvToRgb(h, 0.85f, 1.0f);
        std_msgs::msg::ColorRGBA c;
        c.r = rgb.r;
        c.g = rgb.g;
        c.b = rgb.b;
        c.a = alpha;
        return c;
    }

    visualization_msgs::msg::Marker makeLineMarker(
        const std::string &ns,
        int id,
        const std_msgs::msg::ColorRGBA &color) const
    {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = plot_frame_;
        m.header.stamp = this->now();
        m.ns = ns;
        m.id = id;
        m.type = visualization_msgs::msg::Marker::LINE_STRIP;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x = static_cast<float>(line_width_);
        m.color = color;
        return m;
    }

    visualization_msgs::msg::Marker makeTextMarker(
        const std::string &ns,
        int id,
        const std::string &text,
        const geometry_msgs::msg::Point &pos,
        const std_msgs::msg::ColorRGBA &color) const
    {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = plot_frame_;
        m.header.stamp = this->now();
        m.ns = ns;
        m.id = id;
        m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position = pos;
        m.pose.orientation.w = 1.0;
        m.scale.z = 0.06;
        m.color = color;
        m.text = text;
        return m;
    }

    geometry_msgs::msg::Point sampleToPoint(const Sample &s, std::size_t z_index) const
    {
        geometry_msgs::msg::Point p;
        if (!t0_.has_value())
        {
            return p;
        }
        const double x = (s.t - t0_.value()).seconds() * time_scale_;
        const double y = (s.q - value_center_) * value_scale_;
        const double z = static_cast<double>(z_index) * joint_z_offset_;
        p.x = x;
        p.y = y;
        p.z = z;
        return p;
    }

    void publishMarkers()
    {
        visualization_msgs::msg::MarkerArray arr;

        if (!t0_.has_value())
        {
            viz_pub_->publish(arr);
            return;
        }

        for (std::size_t i = 0; i < joint_order_.size(); ++i)
        {
            const auto &jn = joint_order_[i];

            const auto z_it = joint_to_z_index_.find(jn);
            if (z_it == joint_to_z_index_.end())
            {
                continue;
            }
            const std::size_t z_index = z_it->second;

            const auto c = colorForIndex(i, 1.0f);
            auto m = makeLineMarker("joint_traj", static_cast<int>(i), c);

            const auto &buf = actual_[jn];
            m.points.reserve(buf.size());
            for (const auto &s : buf)
            {
                m.points.push_back(sampleToPoint(s, z_index));
            }
            arr.markers.push_back(std::move(m));

            if (show_commanded_)
            {
                const auto c_cmd = colorForIndex(i, 0.5f);
                auto cmd = makeLineMarker("joint_traj_cmd", static_cast<int>(i), c_cmd);

                const auto &cb = commanded_[jn];
                cmd.points.reserve(cb.size());
                for (const auto &s : cb)
                {
                    cmd.points.push_back(sampleToPoint(s, z_index));
                }
                arr.markers.push_back(std::move(cmd));
            }

            if (show_labels_ && !buf.empty())
            {
                const auto &last = buf.back();
                auto pos = sampleToPoint(last, z_index);
                pos.z += 0.02;
                char txt[256];
                std::snprintf(txt, sizeof(txt), "%s: %.3f rad", jn.c_str(), last.q);
                arr.markers.push_back(makeTextMarker("joint_traj_text", static_cast<int>(i), txt, pos, c));
            }
        }

        viz_pub_->publish(arr);
    }

    void clearAll()
    {
        actual_.clear();
        commanded_.clear();
        joint_order_.clear();
        joint_to_z_index_.clear();
        last_pushed_time_.clear();
        pushed_count_.clear();
        t0_.reset();

        visualization_msgs::msg::MarkerArray arr;
        visualization_msgs::msg::Marker del;
        del.header.frame_id = plot_frame_;
        del.header.stamp = this->now();
        del.action = visualization_msgs::msg::Marker::DELETEALL;
        arr.markers.push_back(del);
        viz_pub_->publish(arr);

        RCLCPP_INFO(get_logger(), "Cleared joint trajectory visualization.");
    }

private:
    std::string plot_frame_;
    int max_points_ = 2000;
    double time_window_sec_ = 0.0;

    double publish_rate_hz_ = 10.0;

    double time_scale_ = 0.2;
    double value_scale_ = 1.0;
    double value_center_ = 0.0;
    double joint_z_offset_ = 0.05;
    double line_width_ = 0.004;

    int sample_stride_ = 1;
    double min_sample_dt_sec_ = 0.0;

    bool show_commanded_ = false;
    std::string commanded_topic_;

    bool show_labels_ = false;
    bool clear_service_enabled_ = true;

    std::optional<rclcpp::Time> t0_;

    std::vector<std::string> joint_order_;
    std::unordered_map<std::string, std::size_t> joint_to_z_index_;

    std::unordered_map<std::string, std::deque<Sample>> actual_;
    std::unordered_map<std::string, std::deque<Sample>> commanded_;

    std::unordered_map<std::string, std::optional<rclcpp::Time>> last_pushed_time_;
    std::unordered_map<std::string, uint64_t> pushed_count_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_sub_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clear_srv_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointTrajectoryVizNode>());
    rclcpp::shutdown();
    return 0;
}

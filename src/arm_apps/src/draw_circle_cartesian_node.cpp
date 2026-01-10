// draw_circle_cartesian_node
//
// Example:
//   ros2 run arm_apps draw_circle_cartesian_node --ros-args -p frame_id:=base_link -p radius:=0.12 -p plane:=xy -p num_points:=80 -p loops:=2 -p vel_scale:=0.2 -p max_acc:=0.3 -p point_duration:=0.3

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "arm_apps/action/go_to_pose.hpp"

#include <cmath>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <optional>
#include <atomic>
#include <stdexcept>
#include <string>
#include <vector>

namespace arm_apps
{
    namespace
    {
        constexpr double kTwoPi = 2.0 * M_PI;

        void require(bool cond, const std::string &msg)
        {
            if (!cond)
            {
                throw std::runtime_error(msg);
            }
        }

        geometry_msgs::msg::PoseStamped makePose(
            const std::string &frame_id,
            const rclcpp::Time &stamp,
            double x,
            double y,
            double z,
            const std::vector<double> &constant_rpy)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = frame_id;
            pose.header.stamp = stamp;

            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = z;

            tf2::Quaternion q;
            q.setRPY(constant_rpy[0], constant_rpy[1], constant_rpy[2]);
            q.normalize();
            pose.pose.orientation = tf2::toMsg(q);
            return pose;
        }

        std::optional<std::ofstream> openCsvIfEnabled(
            const rclcpp::Logger &logger,
            bool enabled,
            const std::string &path)
        {
            if (!enabled)
            {
                return std::nullopt;
            }

            const auto csv_path = std::filesystem::path(path);
            std::error_code ec;
            std::filesystem::create_directories(csv_path.parent_path(), ec);
            if (ec)
            {
                RCLCPP_WARN(logger, "Failed to create CSV directory '%s': %s", csv_path.parent_path().string().c_str(), ec.message().c_str());
            }

            std::ofstream out(path, std::ios::out | std::ios::trunc);
            if (!out.is_open())
            {
                throw std::runtime_error("Could not open csv_path: " + path);
            }

            out << "index,x,y,z,success,planned_duration,iterations,pos_err,rot_err\n";
            out.flush();
            return out;
        }

    } // namespace

    class DrawCircleCartesianNode : public rclcpp::Node
    {
    public:
        using GoToPose = arm_apps::action::GoToPose;
        using GoalHandleGoToPose = rclcpp_action::ClientGoalHandle<GoToPose>;

        DrawCircleCartesianNode() : Node("draw_circle_cartesian_node")
        {
            // Circle params
            this->declare_parameter<std::string>("frame_id", "base_link");
            this->declare_parameter<std::vector<double>>("center", {0.35, 0.0, 0.30});
            this->declare_parameter<double>("radius", 0.10);
            this->declare_parameter<std::string>("plane", "xy");
            this->declare_parameter<int>("num_points", 60);
            this->declare_parameter<int>("loops", 1);
            this->declare_parameter<double>("start_angle", 0.0);
            this->declare_parameter<int>("direction", 1);
            this->declare_parameter<std::vector<double>>("constant_rpy", {0.0, 1.57, 0.0});
            this->declare_parameter<double>("pause_s", 0.0);

            // Motion params (forwarded to action)
            this->declare_parameter<double>("vel_scale", 0.2);
            this->declare_parameter<double>("max_acc", 0.3);
            this->declare_parameter<int>("num_waypoints", 50);
            this->declare_parameter<double>("desired_duration", 0.0);
            this->declare_parameter<double>("point_duration", 0.0);

            // Behavior
            this->declare_parameter<bool>("continue_on_fail", false);

            // Optional CSV
            this->declare_parameter<bool>("log_csv", false);
            this->declare_parameter<std::string>("csv_path", "/tmp/arm_apps_logs/circle_run.csv");

            // Startup gating (avoids first-goal failure before go_to_pose_node has /joint_states)
            this->declare_parameter<std::string>("joint_states_topic", "/joint_states");
            this->declare_parameter<bool>("wait_for_joint_states", true);
            // 0 = wait forever
            this->declare_parameter<double>("joint_states_timeout_s", 0.0);

            action_client_ = rclcpp_action::create_client<GoToPose>(this, "/go_to_pose");

            joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                this->get_parameter("joint_states_topic").as_string(),
                10,
                [this](sensor_msgs::msg::JointState::SharedPtr)
                {
                    joint_state_count_.fetch_add(1, std::memory_order_relaxed);
                });

            loadAndValidateParams();
            csv_out_ = openCsvIfEnabled(this->get_logger(), log_csv_, csv_path_);
        }

        void run()
        {
            RCLCPP_INFO(get_logger(), "Waiting for action server /go_to_pose...");
            while (rclcpp::ok() && !action_client_->wait_for_action_server(std::chrono::seconds(1)))
            {
                rclcpp::spin_some(shared_from_this());
            }
            if (!rclcpp::ok())
            {
                return;
            }

            if (this->get_parameter("wait_for_joint_states").as_bool())
            {
                const double timeout_s = this->get_parameter("joint_states_timeout_s").as_double();
                const auto start = std::chrono::steady_clock::now();

                RCLCPP_INFO(get_logger(), "Waiting for /joint_states... (topic=%s)",
                            this->get_parameter("joint_states_topic").as_string().c_str());

                while (rclcpp::ok())
                {
                    rclcpp::spin_some(shared_from_this());

                    const int count = joint_state_count_.load(std::memory_order_relaxed);
                    if (count >= 2)
                    {
                        break;
                    }

                    if (timeout_s > 0.0)
                    {
                        const auto elapsed = std::chrono::steady_clock::now() - start;
                        if (elapsed >= std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                           std::chrono::duration<double>(timeout_s)))
                        {
                            RCLCPP_WARN(get_logger(), "Timed out waiting for /joint_states after %.2fs", timeout_s);
                            break;
                        }
                    }

                    rclcpp::sleep_for(std::chrono::milliseconds(50));
                }

                if (!rclcpp::ok())
                {
                    return;
                }
            }

            const int total_points = loops_ * num_points_;
            RCLCPP_INFO(
                get_logger(),
                "Executing circle: plane=%s center=[%.3f %.3f %.3f] radius=%.3f points=%d loops=%d (total=%d)",
                plane_.c_str(),
                center_[0],
                center_[1],
                center_[2],
                radius_,
                num_points_,
                loops_,
                total_points);

            for (int k = 0; rclcpp::ok() && k < total_points; ++k)
            {
                const double frac = static_cast<double>(k) / static_cast<double>(num_points_);
                const double theta = start_angle_ + static_cast<double>(direction_) * kTwoPi * frac;

                double x = center_[0];
                double y = center_[1];
                double z = center_[2];

                if (plane_ == "xy")
                {
                    x = center_[0] + radius_ * std::cos(theta);
                    y = center_[1] + radius_ * std::sin(theta);
                    z = center_[2];
                }
                else if (plane_ == "xz")
                {
                    x = center_[0] + radius_ * std::cos(theta);
                    y = center_[1];
                    z = center_[2] + radius_ * std::sin(theta);
                }
                else if (plane_ == "yz")
                {
                    x = center_[0];
                    y = center_[1] + radius_ * std::cos(theta);
                    z = center_[2] + radius_ * std::sin(theta);
                }

                auto goal = GoToPose::Goal();
                goal.target = makePose(frame_id_, this->now(), x, y, z, constant_rpy_);
                goal.vel_scale = vel_scale_;
                goal.max_acc = max_acc_;
                goal.num_waypoints = num_waypoints_;
                goal.desired_duration = resolveDesiredDuration();

                const bool ok = sendOneGoal(k, goal);
                if (!ok && !continue_on_fail_)
                {
                    RCLCPP_WARN(get_logger(), "Stopping due to failed goal at index %d", k);
                    break;
                }

                if (pause_s_ > 0.0)
                {
                    const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::duration<double>(pause_s_));
                    rclcpp::sleep_for(ns);
                }
            }
        }

    private:
        void loadAndValidateParams()
        {
            frame_id_ = this->get_parameter("frame_id").as_string();
            center_ = this->get_parameter("center").as_double_array();
            radius_ = this->get_parameter("radius").as_double();
            plane_ = this->get_parameter("plane").as_string();
            num_points_ = this->get_parameter("num_points").as_int();
            loops_ = this->get_parameter("loops").as_int();
            start_angle_ = this->get_parameter("start_angle").as_double();
            direction_ = this->get_parameter("direction").as_int();
            constant_rpy_ = this->get_parameter("constant_rpy").as_double_array();
            pause_s_ = this->get_parameter("pause_s").as_double();

            vel_scale_ = this->get_parameter("vel_scale").as_double();
            max_acc_ = this->get_parameter("max_acc").as_double();
            num_waypoints_ = this->get_parameter("num_waypoints").as_int();
            desired_duration_ = this->get_parameter("desired_duration").as_double();
            point_duration_ = this->get_parameter("point_duration").as_double();

            continue_on_fail_ = this->get_parameter("continue_on_fail").as_bool();

            log_csv_ = this->get_parameter("log_csv").as_bool();
            csv_path_ = this->get_parameter("csv_path").as_string();

            require(center_.size() == 3, "Parameter 'center' must be a 3-element list");
            require(constant_rpy_.size() == 3, "Parameter 'constant_rpy' must be a 3-element list");
            require(radius_ > 0.0, "Parameter 'radius' must be > 0");
            require(num_points_ >= 3, "Parameter 'num_points' must be >= 3");
            require(loops_ >= 1, "Parameter 'loops' must be >= 1");
            require(direction_ == 1 || direction_ == -1, "Parameter 'direction' must be 1 or -1");
            require(plane_ == "xy" || plane_ == "xz" || plane_ == "yz", "Parameter 'plane' must be one of: xy|xz|yz");
            require(pause_s_ >= 0.0, "Parameter 'pause_s' must be >= 0");
            require(vel_scale_ > 0.0, "Parameter 'vel_scale' must be > 0");
            require(vel_scale_ <= 1.0, "Parameter 'vel_scale' must be <= 1");
            require(max_acc_ >= 0.0, "Parameter 'max_acc' must be >= 0");
            require(num_waypoints_ >= 2, "Parameter 'num_waypoints' must be >= 2");
            require(desired_duration_ >= 0.0, "Parameter 'desired_duration' must be >= 0");
            require(point_duration_ >= 0.0, "Parameter 'point_duration' must be >= 0");
            require(!frame_id_.empty(), "Parameter 'frame_id' must not be empty");
            if (log_csv_)
            {
                require(!csv_path_.empty(), "Parameter 'csv_path' must not be empty when log_csv is true");
            }
        }

        double resolveDesiredDuration() const
        {
            if (point_duration_ > 0.0)
            {
                return point_duration_;
            }
            return desired_duration_;
        }

        bool sendOneGoal(int index, const GoToPose::Goal &goal)
        {
            rclcpp_action::Client<GoToPose>::SendGoalOptions options;

            options.feedback_callback =
                [this, index](GoalHandleGoToPose::SharedPtr, const std::shared_ptr<const GoToPose::Feedback> feedback)
            {
                if (!feedback)
                {
                    return;
                }

                RCLCPP_INFO_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "[%d] stage=%s iter=%d pos_err=%.4f rot_err=%.4f",
                    index,
                    feedback->stage.c_str(),
                    feedback->iterations,
                    feedback->pos_err,
                    feedback->rot_err);
            };

            auto future_handle = action_client_->async_send_goal(goal, options);
            const auto rc_handle = rclcpp::spin_until_future_complete(shared_from_this(), future_handle);
            if (rc_handle != rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_WARN(get_logger(), "[%d] Failed to send goal (future rc=%d)", index, static_cast<int>(rc_handle));
                writeCsv(index, goal.target, false, 0.0, 0, 0.0, 0.0);
                return false;
            }

            auto goal_handle = future_handle.get();
            if (!goal_handle)
            {
                RCLCPP_WARN(get_logger(), "[%d] Goal rejected by server", index);
                writeCsv(index, goal.target, false, 0.0, 0, 0.0, 0.0);
                return false;
            }

            auto future_result = action_client_->async_get_result(goal_handle);
            const auto rc_result = rclcpp::spin_until_future_complete(shared_from_this(), future_result);
            if (rc_result != rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_WARN(get_logger(), "[%d] Failed waiting for result (future rc=%d)", index, static_cast<int>(rc_result));
                writeCsv(index, goal.target, false, 0.0, 0, 0.0, 0.0);
                return false;
            }

            const auto wrapped = future_result.get();
            const auto result = wrapped.result;
            const bool action_ok = (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED);

            if (!result)
            {
                RCLCPP_WARN(get_logger(), "[%d] Empty result from server (code=%d)", index, static_cast<int>(wrapped.code));
                writeCsv(index, goal.target, false, 0.0, 0, 0.0, 0.0);
                return false;
            }

            const bool success = action_ok && result->success;
            if (success)
            {
                RCLCPP_INFO(
                    get_logger(),
                    "[%d] OK planned_duration=%.3f iter=%d pos_err=%.4f rot_err=%.4f",
                    index,
                    result->planned_duration,
                    result->iterations,
                    result->pos_err,
                    result->rot_err);
            }
            else
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[%d] FAIL (code=%d) success=%s msg='%s' planned_duration=%.3f iter=%d pos_err=%.4f rot_err=%.4f",
                    index,
                    static_cast<int>(wrapped.code),
                    result->success ? "true" : "false",
                    result->message.c_str(),
                    result->planned_duration,
                    result->iterations,
                    result->pos_err,
                    result->rot_err);
            }

            writeCsv(
                index,
                goal.target,
                success,
                result->planned_duration,
                result->iterations,
                result->pos_err,
                result->rot_err);
            return success;
        }

        void writeCsv(
            int index,
            const geometry_msgs::msg::PoseStamped &pose,
            bool success,
            double planned_duration,
            int iterations,
            double pos_err,
            double rot_err)
        {
            if (!csv_out_.has_value())
            {
                return;
            }

            auto &out = csv_out_.value();
            out << index << ','
                << pose.pose.position.x << ','
                << pose.pose.position.y << ','
                << pose.pose.position.z << ','
                << (success ? 1 : 0) << ','
                << planned_duration << ','
                << iterations << ','
                << pos_err << ','
                << rot_err << '\n';
            out.flush();
        }

        rclcpp_action::Client<GoToPose>::SharedPtr action_client_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
        std::atomic<int> joint_state_count_{0};

        // Circle params
        std::string frame_id_;
        std::vector<double> center_;
        double radius_ = 0.10;
        std::string plane_;
        int num_points_ = 60;
        int loops_ = 1;
        double start_angle_ = 0.0;
        int direction_ = 1;
        std::vector<double> constant_rpy_;
        double pause_s_ = 0.0;

        // Motion params
        double vel_scale_ = 0.2;
        double max_acc_ = 0.3;
        int num_waypoints_ = 50;
        double desired_duration_ = 0.0;
        double point_duration_ = 0.0;

        // Behavior
        bool continue_on_fail_ = false;

        // CSV
        bool log_csv_ = false;
        std::string csv_path_;
        std::optional<std::ofstream> csv_out_;
    };

} // namespace arm_apps

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<arm_apps::DrawCircleCartesianNode>();
        node->run();
    }
    catch (const std::exception &e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("draw_circle_cartesian_node"), "Fatal: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}

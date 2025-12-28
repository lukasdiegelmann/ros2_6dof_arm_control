// go_to_pose_node.cpp (Auszug)
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "arm_apps/ik_dls.hpp"

#include "arm_apps/urdf_chain_fk.hpp"
#include "arm_apps/joint_mapping.hpp"

class GoToPoseNode : public rclcpp::Node
{
public:
    GoToPoseNode() : Node("go_to_pose_node")
    {
        RCLCPP_WARN(get_logger(), "===== GoToPoseNode Constructor STARTING =====");

        // Ensure our internal timestamps use the same clock type as this node.
        // Otherwise (SYSTEM vs ROS time) time subtraction can throw and crash the node.
        const auto clock_type = this->get_clock()->get_clock_type();
        last_fk_log_ = rclcpp::Time(0, 0, clock_type);
        last_js_log_ = rclcpp::Time(0, 0, clock_type);

        const auto clockTypeToString = [](rcl_clock_type_t t)
        {
            switch (t)
            {
            case RCL_ROS_TIME:
                return "ROS_TIME";
            case RCL_SYSTEM_TIME:
                return "SYSTEM_TIME";
            case RCL_STEADY_TIME:
                return "STEADY_TIME";
            default:
                return "UNKNOWN";
            }
        };
        RCLCPP_INFO(get_logger(), "Clock type: %s", clockTypeToString(clock_type));

        this->declare_parameter<std::string>("base_link", "base_link");
        this->declare_parameter<std::string>("tip_link", "ee_link");
        this->declare_parameter<std::string>("trajectory_topic", "/joint_trajectory_controller/joint_trajectory");
        this->declare_parameter<bool>("debug_hardcoded_trajectory", false);

        const auto base_link = this->get_parameter("base_link").as_string();
        const auto tip_link = this->get_parameter("tip_link").as_string();
        trajectory_topic_ = this->get_parameter("trajectory_topic").as_string();
        debug_hardcoded_ = this->get_parameter("debug_hardcoded_trajectory").as_bool();

        // robot_description holen
        std::string urdf_xml;
        if (!this->has_parameter("robot_description"))
        {
            this->declare_parameter<std::string>("robot_description", "");
        }
        urdf_xml = this->get_parameter("robot_description").as_string();
        if (urdf_xml.empty())
        {
            throw std::runtime_error(
                "robot_description is empty. Make sure robot_state_publisher is running "
                "and robot_description is available to this node (params / remapping).");
        }

        fk_.initFromUrdf(urdf_xml, base_link, tip_link);
        mapping_.init(fk_.jointNames());

        RCLCPP_INFO(get_logger(), "FK ready. DOF=%zu, base=%s, tip=%s",
                    fk_.dof(), base_link.c_str(), tip_link.c_str());
        RCLCPP_INFO(get_logger(), "Trajectory topic: %s", trajectory_topic_.c_str());
        if (debug_hardcoded_)
        {
            RCLCPP_WARN(get_logger(), "DEBUG MODE: Hardcoded trajectory enabled!");
        }

        // Subscriptions
        RCLCPP_WARN(get_logger(), "Creating subscriptions...");
        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&GoToPoseNode::onJointState, this, std::placeholders::_1));
        RCLCPP_WARN(get_logger(), "✓ /joint_states subscription created");

        target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/target_pose", 10,
            std::bind(&GoToPoseNode::onTargetPose, this, std::placeholders::_1));
        RCLCPP_WARN(get_logger(), "✓ /target_pose subscription created");

        // Publisher
        traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            trajectory_topic_, 10);
        RCLCPP_WARN(get_logger(), "✓ Trajectory publisher created");

        RCLCPP_WARN(get_logger(), "===== GoToPoseNode Constructor COMPLETE =====");
    }

private:
    void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        last_js_ = *msg;
        have_js_ = true;

        // Throttled debug log for JointState
        const auto now = this->now();
        try
        {
            if ((now - last_js_log_).seconds() > 2.0)
            {
                RCLCPP_INFO(get_logger(), "JointState received: names=%zu positions=%zu",
                            last_js_.name.size(), last_js_.position.size());
                last_js_log_ = now;
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Time arithmetic failed in onJointState: %s", e.what());
            // Reset to current time to avoid repeated exceptions.
            last_js_log_ = now;
        }

        // (Debug) FK check: 1x pro Sekunde loggen
        try
        {
            if ((now - last_fk_log_).seconds() > 1.0)
            {
                try
                {
                    auto q = mapping_.toChainOrder(last_js_);
                    auto ee = fk_.fk(q);
                    RCLCPP_INFO(get_logger(), "EE pos: [%.3f %.3f %.3f]",
                                ee.p.x(), ee.p.y(), ee.p.z());
                }
                catch (const std::exception &e)
                {
                    RCLCPP_WARN(get_logger(), "FK debug failed: %s", e.what());
                }
                last_fk_log_ = now;
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Time arithmetic failed in FK throttle: %s", e.what());
            last_fk_log_ = now;
        }
    }

    void onTargetPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "TargetPose received (frame=%s)", msg->header.frame_id.c_str());

        RCLCPP_INFO(get_logger(), "Graph: publishers(/target_pose)=%zu subscribers(%s)=%zu",
                    this->count_publishers("/target_pose"),
                    trajectory_topic_.c_str(),
                    this->count_subscribers(trajectory_topic_));

        if (!have_js_)
        {
            RCLCPP_WARN(get_logger(), "No /joint_states yet, cannot solve IK.");
            return;
        }

        // DEBUG MODE: Hardcoded trajectory
        if (debug_hardcoded_)
        {
            publishHardcodedTrajectory();
            return;
        }

        std::vector<double> q_init;
        try
        {
            q_init = mapping_.toChainOrder(last_js_);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Joint mapping failed: %s", e.what());
            return;
        }

        arm_apps::IkParams p;
        p.max_iters = 200;
        p.pos_tol = 1e-3;
        p.rot_tol = 1e-2;
        p.lambda = 0.05;
        p.alpha = 0.5;
        p.eps = 1e-5;

        arm_apps::IkResult r;
        try
        {
            r = arm_apps::solveIkDls(fk_, msg->pose, q_init, p);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "IK threw exception: %s", e.what());
            return;
        }

        if (!r.success)
        {
            RCLCPP_WARN(get_logger(),
                        "IK failed after %d iters. pos_err=%.6f m rot_err=%.6f rad",
                        r.iterations, r.pos_err, r.rot_err);
            return;
        }

        RCLCPP_INFO(get_logger(),
                    "IK success in %d iters. pos_err=%.6f m rot_err=%.6f rad",
                    r.iterations, r.pos_err, r.rot_err);

        trajectory_msgs::msg::JointTrajectory traj;
        traj.header.stamp = this->now();
        traj.joint_names = fk_.jointNames();

        // Start point helps some controllers interpolate cleanly
        trajectory_msgs::msg::JointTrajectoryPoint start_pt;
        start_pt.positions = q_init;
        start_pt.time_from_start = rclcpp::Duration::from_seconds(0.0);
        traj.points.push_back(start_pt);

        trajectory_msgs::msg::JointTrajectoryPoint pt;
        pt.positions = r.q;
        pt.time_from_start = rclcpp::Duration::from_seconds(2.0);
        traj.points.push_back(pt);

        RCLCPP_INFO(get_logger(), "Trajectory publisher subscription_count=%zu", traj_pub_->get_subscription_count());
        traj_pub_->publish(traj);
        RCLCPP_INFO(get_logger(), "Published IK trajectory.");
    }

    void publishHardcodedTrajectory()
    {
        RCLCPP_WARN(get_logger(), "Publishing HARDCODED trajectory (debug mode)");

        trajectory_msgs::msg::JointTrajectory traj;
        traj.header.stamp = this->now();
        traj.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {0.0, -1.2, 1.2, 0.0, 1.57, 0.0};
        point.time_from_start = rclcpp::Duration::from_seconds(2.0);
        traj.points.push_back(point);

        RCLCPP_INFO(get_logger(), "Publishing hardcoded trajectory to '%s' with %zu joints",
                    trajectory_topic_.c_str(), traj.joint_names.size());
        traj_pub_->publish(traj);
        RCLCPP_INFO(get_logger(), "Hardcoded trajectory published.");
    }

    std::string joinNames(const std::vector<std::string> &names)
    {
        std::string result;
        for (size_t i = 0; i < names.size(); ++i)
        {
            if (i > 0)
                result += ", ";
            result += names[i];
        }
        return result;
    }

    arm_apps::UrdfChainFK fk_;
    arm_apps::JointMapping mapping_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;

    sensor_msgs::msg::JointState last_js_;
    bool have_js_{false};
    rclcpp::Time last_fk_log_{};
    rclcpp::Time last_js_log_{};

    std::string trajectory_topic_;
    bool debug_hardcoded_{false};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoToPoseNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

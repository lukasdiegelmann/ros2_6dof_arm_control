#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "arm_apps/ik_dls.hpp"

#include "arm_apps/urdf_chain_fk.hpp"
#include "arm_apps/joint_limits.hpp"
#include "arm_apps/joint_mapping.hpp"

class GoToPoseNode : public rclcpp::Node
{
public:
    GoToPoseNode() : Node("go_to_pose_node"), joint_limits_(fk_)
    {
        RCLCPP_WARN(get_logger(), "===== GoToPoseNode Constructor STARTING =====");

        /*
         * Initialize the last forward kinematics and joint state log times
         * with the same clock type as this node to avoid time-related errors.
         */
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

        /*
         * Declare parameters with default values.
         *
         * Important to notice is here that the go_to_pose_node needs to know from
         * where to where the FK should be calculated (base_link to tip_link). Where
         * the tip_link is usually the end-effector link.
         *
         * The trajectory_topic will be the topic where the calculated joint trajectory
         * will be published to.
         */
        this->declare_parameter<std::string>("base_link", "base_link");
        this->declare_parameter<std::string>("tip_link", "ee_link");
        this->declare_parameter<std::string>("trajectory_topic", "/joint_trajectory_controller/joint_trajectory");
        this->declare_parameter<bool>("debug_hardcoded_trajectory", false);

        /*
         * Get the parameters from the parameter server.
         */
        const auto base_link = this->get_parameter("base_link").as_string();
        const auto tip_link = this->get_parameter("tip_link").as_string();
        trajectory_topic_ = this->get_parameter("trajectory_topic").as_string();

        /*
         * load robot description as parameter
         */
        std::string urdf_xml;

        /*
         * Check if the robot_description parameter exists. If not declare it with
         * an empty string as default value.
         */
        if (!this->has_parameter("robot_description"))
        {
            this->declare_parameter<std::string>("robot_description", "");
        }

        /* Get the urdf_xml string value */
        urdf_xml = this->get_parameter("robot_description").as_string();
        if (urdf_xml.empty())
        {
            throw std::runtime_error(
                "robot_description is empty. Make sure robot_state_publisher is running "
                "and robot_description is available to this node (params / remapping).");
        }

        /* feed the urdf into the forward kinematics calculator */
        fk_.initFromUrdf(urdf_xml, base_link, tip_link);
        joint_limits_ = arm_apps::JointLimits(fk_);
        /* generate JointMapping from joint names */
        mapping_.init(fk_.jointNames());

        RCLCPP_INFO(get_logger(), "FK ready. DOF=%zu, base=%s, tip=%s",
                    fk_.dof(), base_link.c_str(), tip_link.c_str());
        RCLCPP_INFO(get_logger(), "Trajectory topic: %s", trajectory_topic_.c_str());

        /*
         * Define subscriber for JointState messages. This subscriber will listen
         * to the /joint_states topic and call the onJointState callback whenever
         * a new JointState message is received.
         */
        RCLCPP_WARN(get_logger(), "Creating subscriptions...");
        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](sensor_msgs::msg::JointState::SharedPtr msg)
            {
                onJointState(msg);
            });
        RCLCPP_WARN(get_logger(), "✓ /joint_states subscription created");

        /*
         * Define subscriber for target PoseStamped messages. This subscriber will
         * listen to the /target_pose topic and call the onTargetPose callback
         * whenever a new PoseStamped message is received.
         */
        target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/target_pose", 10,
            [this](geometry_msgs::msg::PoseStamped::SharedPtr msg)
            {
                onTargetPose(msg);
            });
        RCLCPP_WARN(get_logger(), "✓ /target_pose subscription created");

        /*
         * Define publisher for the joint trajectory messages. This publisher will
         * be used to publish the calculated joint trajectories to the configured
         * trajectory topic.
         */
        traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            trajectory_topic_, 10);
        RCLCPP_WARN(get_logger(), "✓ Trajectory publisher created");

        RCLCPP_WARN(get_logger(), "===== GoToPoseNode Constructor COMPLETE =====");
    }

private:
    /*
     * Callback for JointState messages, this listener will be called whenever
     * a new JointState message is received. Meaning whenever the joint state
     * changes this function will be called.
     *
     * The main purpose of this function is to store the last received JointState
     * such that it can be used later in when a target pose is received on /target_pose.
     *
     * Additionally this function also logs the received JointState at most once
     * every 2 seconds to avoid flooding the console/logs. It will also log the FK every
     * second for debugging purposes.
     */
    void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        /* Dereference the shared pointer to get the actual JointState message */
        last_js_ = *msg;
        have_js_ = true;

        /*
         * Throttle logging: max once every 2 seconds. Once logged it will save the
         * last logging time and compare it on the next call. This prevents flooding
         * the console/logs with JointState messages.
         */
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
            /* Reset to current time to avoid repeated exceptions. */
            last_js_log_ = now;
        }

        // (Debug) FK check: log every second
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

    /*
     * Callback for target PoseStamped messages. This function will be called
     * whenever a new target pose is received on /target_pose. The main purpose
     * of this function is to solve the IK for the received target pose and
     * publish the resulting joint trajectory to the configured trajectory topic.
     *
     * Since the /target_pose gives information about the desired end-effector pose
     * in the cartesian space, the IK solver is needed to convert this pose into
     * joint angles that can achieve this pose.
     */
    void onTargetPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "TargetPose received (frame=%s)", msg->header.frame_id.c_str());

        RCLCPP_INFO(get_logger(), "Graph: publishers(/target_pose)=%zu subscribers(%s)=%zu",
                    this->count_publishers("/target_pose"),
                    trajectory_topic_.c_str(),
                    this->count_subscribers(trajectory_topic_));

        /*
         * Check if the subscriber has received any JointState messages yet. If not
         * abort. The IK solver needs to know a target pose to solve for.
         */
        if (!have_js_)
        {
            RCLCPP_WARN(get_logger(), "No /joint_states yet, cannot solve IK.");
            return;
        }

        /*
         * Initial joint vector that will be filled with the information from the last
         * received JointState message.
         */
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

        /*
         * Set up IK parameters. This will define the boundary conditions for
         * the inverse kinematics solver.
         */
        arm_apps::IkParams p;
        p.max_iters = 200;
        p.pos_tol = 1e-3;
        p.rot_tol = 1e-2;
        p.lambda = 0.05;
        p.alpha = 0.5;
        p.eps = 1e-5;

        /*
         * Call the IK solver with the current FK, target pose, initial joint
         * configuration and IK parameters.
         */
        arm_apps::IkResult ik_result;
        try
        {
            /*
             * Do the IK solving using Damped Least Squares method. This returns a
             * IkResult struct containing information about the IK solution.
             */
            ik_result = arm_apps::solveIkDls(fk_, msg->pose, q_init, p);

            /*
             * Clamp the resulting joint angles to be within the joint limits. This
             * mutates the ik_result.q vector in place.
             */
            joint_limits_.clamp(ik_result.q);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "IK threw exception: %s", e.what());
            return;
        }

        /*
         * Check if the IK solver was successful. If not, log a warning with the
         * number of iterations and the final position and rotation errors.
         */
        if (!ik_result.success)
        {
            RCLCPP_WARN(get_logger(),
                        "IK failed after %d iters. pos_err=%.6f m rot_err=%.6f rad",
                        ik_result.iterations, ik_result.pos_err, ik_result.rot_err);
            return;
        }

        RCLCPP_INFO(get_logger(),
                    "IK success in %d iters. pos_err=%.6f m rot_err=%.6f rad",
                    ik_result.iterations, ik_result.pos_err, ik_result.rot_err);

        /*
         * Define a JointTrajectory message to publish the IK result as a trajectory.
         * This message will contain the joint names, a current time stamp, a start
         * and a target point.
         */
        trajectory_msgs::msg::JointTrajectory traj;
        traj.header.stamp = this->now();
        traj.joint_names = fk_.jointNames();

        /*
         * Define start point for the trajectory using the stores joint state.
         * Explicitly defining a start point helps controllers to interpolate
         * more smoothly from the current position to the target position.
         */
        trajectory_msgs::msg::JointTrajectoryPoint start_pt;
        start_pt.positions = q_init;
        start_pt.time_from_start = rclcpp::Duration::from_seconds(0.0);
        traj.points.push_back(start_pt);

        /*
         * Definine target point for the trajectory using the IK result.
         * time_from_start defines how long it should take to reach this point,
         * from the time the trajectory is started.
         */
        trajectory_msgs::msg::JointTrajectoryPoint target_pt;
        target_pt.positions = ik_result.q;
        target_pt.time_from_start = rclcpp::Duration::from_seconds(2.0);
        traj.points.push_back(target_pt);

        /*
         * Publish the trajectory to the configured topic.
         */
        RCLCPP_INFO(get_logger(), "Trajectory publisher subscription_count=%zu", traj_pub_->get_subscription_count());
        traj_pub_->publish(traj);
        RCLCPP_INFO(get_logger(), "Published IK trajectory.");
    }

    arm_apps::UrdfChainFK fk_;
    arm_apps::JointLimits joint_limits_;
    arm_apps::JointMapping mapping_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;

    sensor_msgs::msg::JointState last_js_;
    bool have_js_{false};
    rclcpp::Time last_fk_log_{};
    rclcpp::Time last_js_log_{};

    std::string trajectory_topic_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoToPoseNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

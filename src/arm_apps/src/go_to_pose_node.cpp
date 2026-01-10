#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <rcl_action/action_server.h>

#include <optional>
#include <mutex>
#include <thread>
#include <cmath>
#include <chrono>

#include "arm_apps/ik_dls.hpp"

#include "arm_apps/urdf_chain_fk.hpp"
#include "arm_apps/joint_limits.hpp"
#include "arm_apps/joint_mapping.hpp"
#include "arm_apps/joint_trajectory.hpp"

#include "arm_apps/action/go_to_pose.hpp"

class GoToPoseNode : public rclcpp::Node
{
public:
    using GoToPose = arm_apps::action::GoToPose;
    using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPose>;

    GoToPoseNode() : Node("go_to_pose_node"), joint_limits_(fk_)
    {
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

        // Trajectory planning parameters
        this->declare_parameter<double>("vel_scale", 0.5);
        this->declare_parameter<double>("max_acc", 0.0); // <=0: velocity-only
        this->declare_parameter<int>("num_waypoints", 50);
        this->declare_parameter<double>("desired_duration", 0.0); // <=0: no scaling

        // Action semantics: wait for trajectory execution time before reporting success.
        this->declare_parameter<bool>("wait_for_execution", true);
        this->declare_parameter<int>("execution_check_period_ms", 20);
        this->declare_parameter<double>("execution_stall_warn_s", 0.75);

        // Performance diagnostics (wall-clock). Warn if conversion/publish blocks noticeably.
        this->declare_parameter<int>("perf_warn_ms", 200);

        // Respect simulation time if set by launch.
        // Note: In many ROS 2 distros this parameter may already be declared by rclcpp.
        if (!this->has_parameter("use_sim_time"))
        {
            this->declare_parameter<bool>("use_sim_time", false);
        }

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
        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](sensor_msgs::msg::JointState::SharedPtr msg)
            {
                onJointState(msg);
            });

        /*
         * Define publisher for the joint trajectory messages. This publisher will
         * be used to publish the calculated joint trajectories to the configured
         * trajectory topic.
         */
        traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            trajectory_topic_, 10);

        /*
         * Define the GoToPose action server. This action server will listen for
         * incoming goals on the /go_to_pose topic and handle them accordingly.
         */
        action_server_ = rclcpp_action::create_server<GoToPose>(
            this,
            "/go_to_pose",
            [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const GoToPose::Goal> goal)
            {
                return handle_goal(goal);
            },
            [this](std::shared_ptr<GoalHandleGoToPose> goal_handle)
            {
                return handle_cancel(goal_handle);
            },
            [this](std::shared_ptr<GoalHandleGoToPose> goal_handle)
            {
                handle_accepted(goal_handle);
            });

        RCLCPP_INFO(get_logger(), "GoToPose Action Server ready on /go_to_pose");
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
        std::lock_guard<std::mutex> lock(state_mutex_);
        last_js_ = *msg;
        have_js_ = true;

        last_js_recv_steady_ = std::chrono::steady_clock::now();
        have_js_recv_time_ = true;

        try
        {
            last_joint_state_q_ = mapping_.toChainOrder(last_js_);
        }
        catch (const std::exception &)
        {
            // Mapping may fail if names are incomplete during startup.
        }
    }

    rclcpp_action::GoalResponse handle_goal(const std::shared_ptr<const GoToPose::Goal> goal)
    {
        if (!goal)
        {
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (!(std::isfinite(goal->vel_scale) && goal->vel_scale > 0.0 && goal->vel_scale <= 1.0))
        {
            RCLCPP_WARN(get_logger(), "Rejecting goal: vel_scale must be in (0, 1]");
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (!(std::isfinite(goal->max_acc) && goal->max_acc >= 0.0))
        {
            RCLCPP_WARN(get_logger(), "Rejecting goal: max_acc must be >= 0");
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (goal->num_waypoints < 2)
        {
            RCLCPP_WARN(get_logger(), "Rejecting goal: num_waypoints must be >= 2");
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (!(std::isfinite(goal->desired_duration) && goal->desired_duration >= 0.0))
        {
            RCLCPP_WARN(get_logger(), "Rejecting goal: desired_duration must be >= 0");
            return rclcpp_action::GoalResponse::REJECT;
        }

        {
            std::lock_guard<std::mutex> lock(goal_mutex_);
            if (goal_reserved_ && !goal_active_)
            {
                const auto now = std::chrono::steady_clock::now();
                const auto age = now - goal_reserved_since_;
                if (age > std::chrono::seconds(2))
                {
                    RCLCPP_WARN(
                        get_logger(),
                        "Clearing stale goal reservation (likely due to a previous response timeout)");
                    goal_reserved_ = false;
                }
            }

            if (goal_active_ || goal_reserved_)
            {
                RCLCPP_WARN(get_logger(), "Rejecting goal: another goal is currently running");
                return rclcpp_action::GoalResponse::REJECT;
            }
            goal_reserved_ = true;
            goal_reserved_since_ = std::chrono::steady_clock::now();
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleGoToPose> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(get_logger(), "Cancel request received");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleGoToPose> goal_handle)
    {
        {
            std::lock_guard<std::mutex> lock(goal_mutex_);
            goal_active_ = true;
            goal_reserved_ = false;
        }

        std::thread([this, goal_handle]()
                    { execute_goal(goal_handle); })
            .detach();
    }

    void publish_feedback(
        const std::shared_ptr<GoalHandleGoToPose> &goal_handle,
        const std::string &stage,
        int iterations,
        double pos_err,
        double rot_err)
    {
        if (!rclcpp::ok())
        {
            return;
        }

        try
        {
            auto fb = std::make_shared<GoToPose::Feedback>();
            fb->stage = stage;
            fb->iterations = iterations;
            fb->pos_err = pos_err;
            fb->rot_err = rot_err;
            goal_handle->publish_feedback(fb);
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(get_logger(), "Failed to publish feedback during shutdown/race: %s", e.what());
        }
    }

    void execute_goal(const std::shared_ptr<GoalHandleGoToPose> goal_handle)
    {
        auto result = std::make_shared<GoToPose::Result>();

        const auto finish = [this]()
        {
            std::lock_guard<std::mutex> lock(goal_mutex_);
            goal_active_ = false;
            goal_reserved_ = false;
        };

        try
        {
            const auto goal = goal_handle->get_goal();
            publish_feedback(goal_handle, "IK", 0, 0.0, 0.0);

            if (goal_handle->is_canceling())
            {
                publish_feedback(goal_handle, "FAILED", 0, 0.0, 0.0);
                result->success = false;
                result->message = "Cancelled";
                finish();
                try
                {
                    goal_handle->canceled(result);
                }
                catch (const std::exception &e)
                {
                    RCLCPP_WARN(get_logger(), "Failed to publish canceled result (shutdown/race): %s", e.what());
                }
                return;
            }

            std::vector<double> q_start;
            std::optional<std::vector<double>> last_solution_copy;

            /*
             * In this scope the state_mutex_ is locked to safely access the last
             * known JointState and convert it to the chain order joint vector.
             */
            {
                /*
                 * Lock the state mutex to safely access the last known JointState.
                 * This will call state_mutex_.lock() and unlock() automatically.
                 * When entering and leaving this scope respectively.
                 */
                std::lock_guard<std::mutex> lock(state_mutex_);

                /*
                 * If there is no JointState to work with, abort the goal. And return
                 * failure.
                 */
                if (!have_js_)
                {
                    result->success = false;
                    result->message = "No /joint_states received yet";
                    publish_feedback(goal_handle, "FAILED", 0, 0.0, 0.0);
                    finish();
                    try
                    {
                        goal_handle->abort(result);
                    }
                    catch (const std::exception &e)
                    {
                        RCLCPP_WARN(get_logger(), "Failed to publish abort result (shutdown/race): %s", e.what());
                    }
                    return;
                }

                /*
                 * Use the last known joint state as starting point for IK. If there
                 * is a kinematic joint state available use this one directly, if not
                 * use the raw joint state message and map it to chain order.
                 */
                if (last_joint_state_q_.has_value())
                {
                    q_start = *last_joint_state_q_;
                }
                else
                {
                    q_start = mapping_.toChainOrder(last_js_);
                }

                /*
                 * Store the last_solution locally to use it as IK initial guess outside
                 * of the locked scope.
                 */
                last_solution_copy = last_solution_;
            }

            /*
             * Use the last IK solution as initial guess if available for better continuity.
             * Should there be no last solution, use the current joint state as initial guess.
             */
            const std::vector<double> &q_init = last_solution_copy.has_value() ? *last_solution_copy : q_start;

            /*
             * Define the Inverse Kinematics parameters to be used for solving IK.
             */
            arm_apps::IkParams p;
            p.max_iters = 200;
            p.pos_tol = 1e-3;
            p.rot_tol = 1e-2;
            p.lambda = lambda_;
            p.alpha = 0.5;
            p.eps = 1e-5;

            /*
             * Try to solve IK for the given target pose. If IK fails, abort the goal
             * and return failure. Also clamp the resulting IK solution to joint limits.
             */
            arm_apps::IkResult ik;
            try
            {
                ik = arm_apps::solveIkDls(fk_, goal->target.pose, q_init, p);
                joint_limits_.clamp(ik.q);
            }
            catch (const std::exception &e)
            {
                result->success = false;
                result->message = std::string("IK exception: ") + e.what();
                publish_feedback(goal_handle, "FAILED", 0, 0.0, 0.0);
                finish();
                try
                {
                    goal_handle->abort(result);
                }
                catch (const std::exception &e2)
                {
                    RCLCPP_WARN(get_logger(), "Failed to publish abort result (shutdown/race): %s", e2.what());
                }
                return;
            }

            /*
             * Publish IK feedback, giving all listeners an update about the current
             * IK status.
             */
            publish_feedback(goal_handle, "IK", ik.iterations, ik.pos_err, ik.rot_err);
            result->iterations = ik.iterations;
            result->pos_err = ik.pos_err;
            result->rot_err = ik.rot_err;

            /*
             * If IK was not successful, abort the goal and return failure.
             */
            if (!ik.success)
            {
                result->success = false;
                result->message = "IK failed";
                publish_feedback(goal_handle, "FAILED", ik.iterations, ik.pos_err, ik.rot_err);
                finish();
                try
                {
                    goal_handle->abort(result);
                }
                catch (const std::exception &e)
                {
                    RCLCPP_WARN(get_logger(), "Failed to publish abort result (shutdown/race): %s", e.what());
                }
                return;
            }

            /*
             * If IK took more than 70% of the maximum iterations, increase the damping
             * factor for the next IK call to improve convergence. This is an heuristic
             * approach to deal with hard IK problems.
             */
            if (ik.iterations > static_cast<int>(0.7 * static_cast<double>(p.max_iters)))
            {
                /*
                 * Increase the damping factor, but cap it to a maximum value to avoid
                 * excessive damping.
                 */
                lambda_ = std::min(lambda_ * 2.0, lambda_max_);
                RCLCPP_WARN(get_logger(), "Hard IK (%d/%d). Increasing damping to %.4f", ik.iterations, p.max_iters, lambda_);
            }

            /*
             * Check if a cancel request has been received during IK or before
             * starting trajectory planning.
             */
            if (goal_handle->is_canceling())
            {
                publish_feedback(goal_handle, "FAILED", ik.iterations, ik.pos_err, ik.rot_err);
                result->success = false;
                result->message = "Cancelled";
                finish();
                try
                {
                    goal_handle->canceled(result);
                }
                catch (const std::exception &e)
                {
                    RCLCPP_WARN(get_logger(), "Failed to publish canceled result (shutdown/race): %s", e.what());
                }
                return;
            }

            /*
             * Publish feedback indicating that trajectory planning is starting.
             */
            publish_feedback(goal_handle, "PLANNING", ik.iterations, ik.pos_err, ik.rot_err);

            /*
             * These are the parameters necessary for the trajectory planning.
             */
            const double vel_scale = goal->vel_scale;
            const double max_acc = goal->max_acc;
            const size_t num_waypoints = static_cast<size_t>(goal->num_waypoints);
            const double desired_duration = goal->desired_duration;

            arm_apps::PlannedTrajectory planned;

            /*
             * Try to plan a joint trajectory from the current joint state to the
             * calculated IK solution. If planning fails, abort the goal and return
             * failure.
             */
            try
            {
                planned = arm_apps::planJointTrajectory(q_start, ik.q, joint_limits_, num_waypoints, vel_scale, max_acc, desired_duration);
            }
            catch (const std::exception &e)
            {
                result->success = false;
                result->message = std::string("Trajectory planning failed: ") + e.what();
                publish_feedback(goal_handle, "FAILED", ik.iterations, ik.pos_err, ik.rot_err);
                finish();
                try
                {
                    goal_handle->abort(result);
                }
                catch (const std::exception &e2)
                {
                    RCLCPP_WARN(get_logger(), "Failed to publish abort result (shutdown/race): %s", e2.what());
                }
                return;
            }

            /*
             * Append the planned trajectory duration to the result message.
             */
            const double planned_duration = planned.t.back() - planned.t.front();
            result->planned_duration = planned_duration;

            /*
             * Check if a cancel request has been received during planning or before
             * publishing the trajectory.
             */
            if (goal_handle->is_canceling())
            {
                publish_feedback(goal_handle, "FAILED", ik.iterations, ik.pos_err, ik.rot_err);
                result->success = false;
                result->message = "Cancelled";
                finish();
                try
                {
                    goal_handle->canceled(result);
                }
                catch (const std::exception &e)
                {
                    RCLCPP_WARN(get_logger(), "Failed to publish canceled result (shutdown/race): %s", e.what());
                }
                return;
            }

            /*
             * Publish feedback indicating that the trajectory is being published.
             */
            publish_feedback(goal_handle, "PUBLISHING", ik.iterations, ik.pos_err, ik.rot_err);

            /*
             * Convert the planned trajectory to a ROS JointTrajectory message and
             * publish it to the configured trajectory topic.
             */
            const int perf_warn_ms_raw = static_cast<int>(this->get_parameter("perf_warn_ms").as_int());
            const int perf_warn_ms = (perf_warn_ms_raw < 0) ? 0 : perf_warn_ms_raw;

            // Use a zero stamp so the controller starts the trajectory immediately on receipt.
            // This is more robust in simulation if /clock stutters or jumps.
            const auto t_convert0 = std::chrono::steady_clock::now();
            auto traj_msg = arm_apps::toRosTrajectoryMsg(planned, fk_.jointNames(), rclcpp::Time(0, 0, RCL_ROS_TIME));
            const auto t_convert1 = std::chrono::steady_clock::now();

            const auto t_pub0 = std::chrono::steady_clock::now();
            traj_pub_->publish(traj_msg);
            const auto t_pub1 = std::chrono::steady_clock::now();

            if (perf_warn_ms > 0)
            {
                const auto convert_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_convert1 - t_convert0).count();
                const auto publish_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_pub1 - t_pub0).count();
                if (convert_ms >= perf_warn_ms || publish_ms >= perf_warn_ms)
                {
                    RCLCPP_WARN(get_logger(),
                                "Slow PUBLISHING step: toRosTrajectoryMsg=%ldms publish=%ldms (threshold=%dms). This can explain perceived lag between PUBLISHING and EXECUTING.",
                                static_cast<long>(convert_ms),
                                static_cast<long>(publish_ms),
                                perf_warn_ms);
                }
            }

            // Optionally wait until the trajectory should have finished executing.
            // This makes the action result align with actual motion time (instead of returning
            // immediately after publishing).
            if (this->get_parameter("wait_for_execution").as_bool() && planned_duration > 0.0)
            {
                publish_feedback(goal_handle, "EXECUTING", ik.iterations, ik.pos_err, ik.rot_err);

                const int period_ms_raw = static_cast<int>(this->get_parameter("execution_check_period_ms").as_int());
                const int period_ms = (period_ms_raw < 1) ? 1 : period_ms_raw;
                const auto start_time = this->now();
                const auto end_time = start_time + rclcpp::Duration::from_seconds(planned_duration);

                const double stall_warn_s = std::max(0.0, this->get_parameter("execution_stall_warn_s").as_double());
                rclcpp::Time last_ros_time = start_time;

                const auto start_steady = std::chrono::steady_clock::now();
                const auto end_steady = start_steady + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                                           std::chrono::duration<double>(planned_duration));
                auto last_warn_steady = start_steady;
                auto last_ros_time_progress_steady = start_steady;
                auto last_joint_progress_steady = start_steady;

                bool clock_was_stalled = false;
                bool js_was_stalled = false;
                bool motion_was_stalled = false;
                bool using_wall_deadline = false;

                std::optional<std::vector<double>> q_last;

                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    if (last_joint_state_q_.has_value())
                    {
                        q_last = *last_joint_state_q_;
                    }
                }

                while (rclcpp::ok())
                {
                    const auto steady_now = std::chrono::steady_clock::now();

                    // Termination condition:
                    // - Normally, we wait on ROS time (so action duration matches sim time).
                    // - If /clock stalls (proven by wall-clock measurement), switch to a wall-clock deadline
                    //   to avoid waiting forever on a stopped sim clock.
                    if (!using_wall_deadline)
                    {
                        if (this->now() >= end_time)
                        {
                            break;
                        }
                    }
                    else
                    {
                        if (steady_now >= end_steady)
                        {
                            break;
                        }
                    }

                    const auto t_now = this->now();

                    if (goal_handle->is_canceling())
                    {
                        publish_feedback(goal_handle, "FAILED", ik.iterations, ik.pos_err, ik.rot_err);
                        result->success = false;
                        result->message = "Cancelled";
                        finish();
                        try
                        {
                            goal_handle->canceled(result);
                        }
                        catch (const std::exception &e)
                        {
                            RCLCPP_WARN(get_logger(), "Failed to publish canceled result (shutdown/race): %s", e.what());
                        }
                        return;
                    }

                    // Detect stalls in wall-clock time so /clock pauses are observable.
                    // We monitor three things independently:
                    //  1) ROS time progress (i.e., /clock advancing)
                    //  2) /joint_states callback reception (executor/transport)
                    //  3) Joint position progress (actual motion)
                    if (stall_warn_s > 0.0)
                    {
                        if (t_now > last_ros_time)
                        {
                            // If ROS time was previously stalled (wall), explicitly log resume.
                            if (clock_was_stalled)
                            {
                                const auto stall = std::chrono::duration_cast<std::chrono::duration<double>>(
                                                       steady_now - last_ros_time_progress_steady)
                                                       .count();
                                if (std::isfinite(stall) && stall >= stall_warn_s)
                                {
                                    RCLCPP_WARN(get_logger(),
                                                "ROS time (/clock) resumed after %.3fs wall (during EXECUTING)",
                                                stall);
                                }
                            }
                            last_ros_time = t_now;
                            last_ros_time_progress_steady = steady_now;
                            clock_was_stalled = false;
                        }

                        std::optional<std::vector<double>> q_now;
                        rclcpp::Time js_stamp(0, 0, RCL_ROS_TIME);
                        std::chrono::steady_clock::time_point js_recv_steady{};
                        bool have_js_recv = false;
                        {
                            std::lock_guard<std::mutex> lock(state_mutex_);
                            if (last_joint_state_q_.has_value())
                            {
                                q_now = *last_joint_state_q_;
                            }
                            js_stamp = rclcpp::Time(last_js_.header.stamp);

                            have_js_recv = have_js_recv_time_;
                            js_recv_steady = last_js_recv_steady_;
                        }

                        if (q_now.has_value() && q_last.has_value() && q_now->size() == q_last->size())
                        {
                            double max_abs_dq = 0.0;
                            for (size_t j = 0; j < q_now->size(); ++j)
                            {
                                max_abs_dq = std::max(max_abs_dq, std::abs((*q_now)[j] - (*q_last)[j]));
                            }
                            if (max_abs_dq > 1e-4)
                            {
                                // Explicitly log motion resume after a stall.
                                if (motion_was_stalled)
                                {
                                    const auto stall = std::chrono::duration_cast<std::chrono::duration<double>>(
                                                           steady_now - last_joint_progress_steady)
                                                           .count();
                                    if (std::isfinite(stall) && stall >= stall_warn_s)
                                    {
                                        RCLCPP_WARN(get_logger(),
                                                    "Joint motion resumed after %.3fs wall (during EXECUTING)",
                                                    stall);
                                    }
                                }
                                q_last = q_now;
                                last_joint_progress_steady = steady_now;
                                motion_was_stalled = false;
                            }
                        }

                        const auto toSeconds = [](std::chrono::steady_clock::duration d)
                        {
                            return std::chrono::duration_cast<std::chrono::duration<double>>(d).count();
                        };

                        const double since_warn_s = toSeconds(steady_now - last_warn_steady);
                        if (std::isfinite(since_warn_s) && since_warn_s >= stall_warn_s)
                        {
                            const double clock_stall_wall_s = toSeconds(steady_now - last_ros_time_progress_steady);
                            const double joint_stall_wall_s = toSeconds(steady_now - last_joint_progress_steady);
                            const double js_rx_age_wall_s = have_js_recv ? toSeconds(steady_now - js_recv_steady) : -1.0;
                            const double js_stamp_age_ros_s = (t_now - js_stamp).seconds();

                            if (std::isfinite(clock_stall_wall_s) && clock_stall_wall_s >= stall_warn_s)
                            {
                                clock_was_stalled = true;

                                if (!using_wall_deadline && this->get_parameter("use_sim_time").as_bool())
                                {
                                    using_wall_deadline = true;
                                    RCLCPP_WARN(get_logger(),
                                                "Detected /clock stall for %.3fs wall; switching EXECUTING wait termination to wall-clock deadline to avoid waiting on frozen sim time.",
                                                clock_stall_wall_s);
                                }
                            }
                            if (std::isfinite(joint_stall_wall_s) && joint_stall_wall_s >= stall_warn_s)
                            {
                                motion_was_stalled = true;
                            }
                            if (std::isfinite(js_rx_age_wall_s) && js_rx_age_wall_s >= stall_warn_s)
                            {
                                js_was_stalled = true;
                            }
                            else if (js_was_stalled && have_js_recv && std::isfinite(js_rx_age_wall_s) && js_rx_age_wall_s < (stall_warn_s * 0.5))
                            {
                                RCLCPP_WARN(get_logger(),
                                            "/joint_states reception resumed (rx_age_wall=%.3fs during EXECUTING)",
                                            js_rx_age_wall_s);
                                js_was_stalled = false;
                            }

                            // Warn when any signal indicates a stall.
                            if ((std::isfinite(clock_stall_wall_s) && clock_stall_wall_s >= stall_warn_s) ||
                                (std::isfinite(js_rx_age_wall_s) && js_rx_age_wall_s >= stall_warn_s) ||
                                (std::isfinite(joint_stall_wall_s) && joint_stall_wall_s >= stall_warn_s))
                            {
                                RCLCPP_WARN(get_logger(),
                                            "Execution stall (wall>=%.2fs): /clock_stall_wall=%.3fs joint_move_stall_wall=%.3fs joint_states_rx_age_wall=%.3fs (ros_time_now=%.3f, joint_state_stamp_age_ros=%.3fs)",
                                            stall_warn_s,
                                            std::isfinite(clock_stall_wall_s) ? clock_stall_wall_s : -1.0,
                                            std::isfinite(joint_stall_wall_s) ? joint_stall_wall_s : -1.0,
                                            std::isfinite(js_rx_age_wall_s) ? js_rx_age_wall_s : -1.0,
                                            t_now.seconds(),
                                            std::isfinite(js_stamp_age_ros_s) ? js_stamp_age_ros_s : -1.0);
                                last_warn_steady = steady_now;
                            }
                        }
                    }

                    rclcpp::sleep_for(std::chrono::milliseconds(period_ms));
                }
            }

            /*
             * Store last IK solution for continuity in the next IK call. This needs
             * to happen in a locked scope to avoid data races.
             */
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                last_solution_ = ik.q;
            }

            /*
             * Publish final success feedback and set the result accordingly.
             */
            result->success = true;
            result->message = "OK";
            publish_feedback(goal_handle, "DONE", ik.iterations, ik.pos_err, ik.rot_err);

            /*
             * Finish the goal successfully.
             */
            finish();
            try
            {
                goal_handle->succeed(result);
            }
            catch (const std::exception &e)
            {
                RCLCPP_WARN(get_logger(), "Failed to publish succeed result (shutdown/race): %s", e.what());
            }
        }
        catch (const std::exception &e)
        {
            result->success = false;
            result->message = std::string("Unhandled exception: ") + e.what();
            finish();
            try
            {
                goal_handle->abort(result);
            }
            catch (const std::exception &e2)
            {
                RCLCPP_WARN(get_logger(), "Failed to publish abort result (shutdown/race): %s", e2.what());
            }
        }
    }

    arm_apps::UrdfChainFK fk_;
    arm_apps::JointLimits joint_limits_;
    arm_apps::JointMapping mapping_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;

    rclcpp_action::Server<GoToPose>::SharedPtr action_server_;

    std::mutex goal_mutex_;
    bool goal_active_{false};
    bool goal_reserved_{false};
    std::chrono::steady_clock::time_point goal_reserved_since_{};

    std::mutex state_mutex_;
    sensor_msgs::msg::JointState last_js_;
    bool have_js_{false};

    std::chrono::steady_clock::time_point last_js_recv_steady_{};
    bool have_js_recv_time_{false};

    std::optional<std::vector<double>> last_solution_;
    std::optional<std::vector<double>> last_joint_state_q_;
    double lambda_{0.05};
    double lambda_max_{1.0};

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

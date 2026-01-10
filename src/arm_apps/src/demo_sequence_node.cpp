#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <controller_manager_msgs/srv/list_controllers.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include "arm_apps/action/go_to_pose.hpp"

#include <array>
#include <chrono>
#include <filesystem>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace arm_apps
{
    namespace
    {
        struct Target
        {
            std::array<double, 3> position;
            std::array<double, 3> rpy;
            std::string frame_id;
        };

        std::vector<Target> loadTargets(const std::string &yaml_path, const std::string &default_frame)
        {
            YAML::Node root = YAML::LoadFile(yaml_path);
            YAML::Node targets = root["targets"];
            if (!targets || !targets.IsSequence())
            {
                throw std::runtime_error("demo_targets.yaml: missing 'targets' sequence");
            }

            std::vector<Target> out;
            out.reserve(targets.size());

            for (const auto &t : targets)
            {
                const auto pos = t["position"];
                const auto rpy = t["rpy"];

                if (!pos || !pos.IsSequence() || pos.size() != 3)
                {
                    throw std::runtime_error("demo_targets.yaml: target.position must be a 3-element list");
                }
                if (!rpy || !rpy.IsSequence() || rpy.size() != 3)
                {
                    throw std::runtime_error("demo_targets.yaml: target.rpy must be a 3-element list");
                }

                Target tgt;
                tgt.position = {pos[0].as<double>(), pos[1].as<double>(), pos[2].as<double>()};
                tgt.rpy = {rpy[0].as<double>(), rpy[1].as<double>(), rpy[2].as<double>()};
                tgt.frame_id = t["frame_id"] ? t["frame_id"].as<std::string>() : default_frame;
                out.push_back(tgt);
            }

            return out;
        }

        geometry_msgs::msg::PoseStamped targetToPoseStamped(const Target &t, const rclcpp::Time &stamp)
        {
            geometry_msgs::msg::PoseStamped msg;
            msg.header.frame_id = t.frame_id;
            msg.header.stamp = stamp;

            msg.pose.position.x = t.position[0];
            msg.pose.position.y = t.position[1];
            msg.pose.position.z = t.position[2];

            tf2::Quaternion q;
            q.setRPY(t.rpy[0], t.rpy[1], t.rpy[2]);
            q.normalize();
            msg.pose.orientation = tf2::toMsg(q);
            return msg;
        }

    } // namespace

    class DemoSequenceNode : public rclcpp::Node
    {
    public:
        using GoToPose = arm_apps::action::GoToPose;
        using GoalHandleGoToPose = rclcpp_action::ClientGoalHandle<GoToPose>;

        DemoSequenceNode() : Node("demo_sequence_node")
        {
            this->declare_parameter<std::string>("targets_yaml", "config/demo_targets.yaml");

            // Goal defaults (applied to every target)
            this->declare_parameter<double>("vel_scale", 0.5);
            this->declare_parameter<double>("max_acc", 0.0);
            this->declare_parameter<int>("num_waypoints", 50);
            this->declare_parameter<double>("desired_duration", 0.0);
            this->declare_parameter<double>("pause_s", 0.0);

            // Sequence behavior
            this->declare_parameter<bool>("log_targets", true);
            this->declare_parameter<bool>("continue_on_failure", false);
            // If a target fails, attempt to return to a 'home' target afterwards.
            // By default, uses the last target in the list.
            this->declare_parameter<bool>("return_home_on_failure", true);
            this->declare_parameter<int>("home_index", -1);

            // Default frame if YAML doesn't provide one
            this->declare_parameter<std::string>("frame_id", "base_link");

            // Trajectory topic to wait for controller readiness (subscribers present).
            this->declare_parameter<std::string>(
                "trajectory_topic",
                "/joint_trajectory_controller/joint_trajectory");

            // Joint state gating (needed so the action server can build q_start)
            this->declare_parameter<std::string>("joint_states_topic", "/joint_states");

            // Controller-manager readiness gating (avoids startup race where the first
            // trajectory gets published before the controller is ACTIVE).
            this->declare_parameter<bool>("wait_controller_active", true);
            this->declare_parameter<std::string>("controller_manager_service", "/controller_manager/list_controllers");
            this->declare_parameter<std::string>("controller_name", "joint_trajectory_controller");
            this->declare_parameter<int>("controller_active_check_period_ms", 200);

            // Respect simulation time if set by launch.
            // Note: In many ROS 2 distros this parameter may already be declared by rclcpp.
            if (!this->has_parameter("use_sim_time"))
            {
                this->declare_parameter<bool>("use_sim_time", false);
            }

            action_client_ = rclcpp_action::create_client<GoToPose>(this, "/go_to_pose");

            joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                this->get_parameter("joint_states_topic").as_string(),
                10,
                [this](sensor_msgs::msg::JointState::SharedPtr msg)
                {
                    (void)msg;
                    std::lock_guard<std::mutex> lock(js_mutex_);
                    have_joint_state_ = true;
                });
        }

        bool haveJointState() const
        {
            std::lock_guard<std::mutex> lock(js_mutex_);
            return have_joint_state_;
        }

        std::vector<Target> loadConfiguredTargets()
        {
            std::string yaml_param = this->get_parameter("targets_yaml").as_string();
            if (yaml_param.empty())
            {
                throw std::runtime_error("targets_yaml parameter is empty");
            }

            std::string yaml_path = yaml_param;
            if (yaml_path.front() != '/')
            {
                const std::string share = ament_index_cpp::get_package_share_directory("arm_apps");
                yaml_path = (std::filesystem::path(share) / yaml_path).string();
            }

            RCLCPP_INFO(get_logger(), "Loading targets from: %s", yaml_path.c_str());
            return loadTargets(yaml_path, this->get_parameter("frame_id").as_string());
        }

        bool waitForServer(const std::chrono::milliseconds &period)
        {
            return action_client_->wait_for_action_server(period);
        }

        std::string trajectoryTopic() const
        {
            return this->get_parameter("trajectory_topic").as_string();
        }

        bool waitForControllerActive()
        {
            if (!this->get_parameter("wait_controller_active").as_bool())
            {
                return true;
            }

            const auto service_name = this->get_parameter("controller_manager_service").as_string();
            const auto controller_name = this->get_parameter("controller_name").as_string();

            auto client = this->create_client<controller_manager_msgs::srv::ListControllers>(service_name);

            RCLCPP_INFO(get_logger(), "Waiting for controller_manager service %s...", service_name.c_str());
            while (rclcpp::ok() && !client->wait_for_service(std::chrono::seconds(1)))
            {
                rclcpp::spin_some(shared_from_this());
            }

            if (!rclcpp::ok())
            {
                return false;
            }

            const int period_ms_raw = this->get_parameter("controller_active_check_period_ms").as_int();
            const int period_ms = (period_ms_raw < 10) ? 10 : period_ms_raw;

            RCLCPP_INFO(get_logger(), "Waiting for controller '%s' to become ACTIVE...", controller_name.c_str());
            while (rclcpp::ok())
            {
                auto req = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
                auto fut = client->async_send_request(req);

                const auto rc = rclcpp::spin_until_future_complete(shared_from_this(), fut, std::chrono::seconds(2));
                if (rc == rclcpp::FutureReturnCode::SUCCESS)
                {
                    const auto resp = fut.get();
                    bool found = false;
                    bool active = false;

                    for (const auto &c : resp->controller)
                    {
                        if (c.name == controller_name)
                        {
                            found = true;
                            active = (c.state == "active");
                            if (active)
                            {
                                break;
                            }
                        }
                    }

                    if (!found)
                    {
                        RCLCPP_WARN(get_logger(), "Controller '%s' not listed yet; retrying...", controller_name.c_str());
                    }
                    else if (active)
                    {
                        RCLCPP_INFO(get_logger(), "Controller '%s' is ACTIVE.", controller_name.c_str());
                        return true;
                    }
                }
                else
                {
                    RCLCPP_WARN(get_logger(), "Timeout/error while calling %s; retrying...", service_name.c_str());
                }

                rclcpp::sleep_for(std::chrono::milliseconds(period_ms));
            }

            return false;
        }

        double sendTarget(const Target &tgt, size_t index)
        {
            GoToPose::Goal goal;
            goal.target = targetToPoseStamped(tgt, this->now());

            goal.vel_scale = this->get_parameter("vel_scale").as_double();
            goal.max_acc = this->get_parameter("max_acc").as_double();
            goal.num_waypoints = this->get_parameter("num_waypoints").as_int();
            goal.desired_duration = this->get_parameter("desired_duration").as_double();

            RCLCPP_INFO(get_logger(),
                        "[%zu] sending target frame=%s pos=[%.3f %.3f %.3f] rpy=[%.3f %.3f %.3f] vel_scale=%.3f max_acc=%.3f num_waypoints=%d desired_duration=%.3f",
                        index,
                        tgt.frame_id.c_str(),
                        tgt.position[0], tgt.position[1], tgt.position[2],
                        tgt.rpy[0], tgt.rpy[1], tgt.rpy[2],
                        goal.vel_scale,
                        goal.max_acc,
                        goal.num_waypoints,
                        goal.desired_duration);

            rclcpp_action::Client<GoToPose>::SendGoalOptions opts;
            opts.feedback_callback =
                [this, index](GoalHandleGoToPose::SharedPtr,
                              const std::shared_ptr<const GoToPose::Feedback> feedback)
            {
                RCLCPP_INFO(get_logger(), "[%zu] feedback stage=%s iters=%d pos_err=%.6f rot_err=%.6f",
                            index,
                            feedback->stage.c_str(),
                            feedback->iterations,
                            feedback->pos_err,
                            feedback->rot_err);
            };

            auto goal_handle_future = action_client_->async_send_goal(goal, opts);

            auto rc = rclcpp::spin_until_future_complete(shared_from_this(), goal_handle_future);
            if (rc != rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_ERROR(get_logger(), "[%zu] Failed waiting for goal_handle", index);
                return 0.0;
            }

            auto goal_handle = goal_handle_future.get();
            if (!goal_handle)
            {
                RCLCPP_ERROR(get_logger(), "[%zu] Goal rejected by server", index);
                return 0.0;
            }

            auto result_future = action_client_->async_get_result(goal_handle);
            rc = rclcpp::spin_until_future_complete(shared_from_this(), result_future);
            if (rc != rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_ERROR(get_logger(), "[%zu] Failed waiting for result", index);
                return 0.0;
            }

            const auto wrapped = result_future.get();
            if (!wrapped.result)
            {
                RCLCPP_ERROR(get_logger(), "[%zu] Result missing", index);
                return 0.0;
            }

            const auto &res = *wrapped.result;
            RCLCPP_INFO(get_logger(),
                        "[%zu] result success=%s msg='%s' planned_duration=%.3f iters=%d pos_err=%.6f rot_err=%.6f",
                        index,
                        res.success ? "true" : "false",
                        res.message.c_str(),
                        res.planned_duration,
                        res.iterations,
                        res.pos_err,
                        res.rot_err);

            return res.success ? std::max(0.0, res.planned_duration) : 0.0;
        }

        double pauseSeconds() const
        {
            return this->get_parameter("pause_s").as_double();
        }

    private:
        rclcpp_action::Client<GoToPose>::SharedPtr action_client_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
        mutable std::mutex js_mutex_;
        bool have_joint_state_{false};
    };

} // namespace arm_apps

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<arm_apps::DemoSequenceNode>();
    RCLCPP_INFO(node->get_logger(), "Waiting for /go_to_pose action server...");

    while (rclcpp::ok() && !node->waitForServer(std::chrono::milliseconds(250)))
    {
        // keep node responsive and allow /clock updates
        rclcpp::spin_some(node);
    }

    if (!rclcpp::ok())
    {
        rclcpp::shutdown();
        return 0;
    }

    RCLCPP_INFO(node->get_logger(), "Waiting for first /joint_states...");
    while (rclcpp::ok() && !node->haveJointState())
    {
        rclcpp::spin_some(node);
        rclcpp::sleep_for(std::chrono::milliseconds(50));
    }

    if (!rclcpp::ok())
    {
        rclcpp::shutdown();
        return 0;
    }

    {
        const auto traj_topic = node->trajectoryTopic();
        RCLCPP_INFO(node->get_logger(), "Waiting for subscribers on %s...", traj_topic.c_str());
        while (rclcpp::ok())
        {
            try
            {
                if (node->count_subscribers(traj_topic) > 0)
                {
                    break;
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_WARN(node->get_logger(), "Failed to count subscribers (shutdown/race): %s", e.what());
                break;
            }

            rclcpp::spin_some(node);
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }

        size_t subs = 0;
        try
        {
            subs = node->count_subscribers(traj_topic);
        }
        catch (const std::exception &)
        {
            subs = 0;
        }
        RCLCPP_INFO(node->get_logger(), "Trajectory controller ready (subscribers=%zu)", subs);
    }

    // Ensure controller is actually ACTIVE before sending the first trajectory.
    if (rclcpp::ok() && !node->waitForControllerActive())
    {
        RCLCPP_ERROR(node->get_logger(), "Controller did not become ACTIVE; exiting.");
        rclcpp::shutdown();
        return 3;
    }

    std::vector<arm_apps::Target> targets;
    try
    {
        targets = node->loadConfiguredTargets();
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to load targets: %s", e.what());
        rclcpp::shutdown();
        return 2;
    }

    const bool log_targets = node->get_parameter("log_targets").as_bool();
    const bool continue_on_failure = node->get_parameter("continue_on_failure").as_bool();
    const bool return_home_on_failure = node->get_parameter("return_home_on_failure").as_bool();
    const int home_index_param = node->get_parameter("home_index").as_int();

    if (log_targets)
    {
        RCLCPP_INFO(node->get_logger(), "Loaded %zu targets:", targets.size());
        for (size_t i = 0; i < targets.size(); ++i)
        {
            const auto &t = targets[i];
            RCLCPP_INFO(node->get_logger(), "  [%zu] frame=%s pos=[%.3f %.3f %.3f] rpy=[%.3f %.3f %.3f]",
                        i,
                        t.frame_id.c_str(),
                        t.position[0], t.position[1], t.position[2],
                        t.rpy[0], t.rpy[1], t.rpy[2]);
        }
    }

    const double pause_s = std::max(0.0, node->pauseSeconds());
    bool any_failure = false;
    size_t failed_index = 0;

    for (size_t i = 0; rclcpp::ok() && i < targets.size(); ++i)
    {
        const double planned_duration_s = node->sendTarget(targets[i], i);
        if (planned_duration_s <= 0.0)
        {
            any_failure = true;
            failed_index = i;
            RCLCPP_ERROR(node->get_logger(), "Target %zu failed", i);
            if (!continue_on_failure)
            {
                RCLCPP_ERROR(node->get_logger(), "Stopping demo sequence due to failure at index %zu", i);
                break;
            }
        }

        if (pause_s > 0.0)
        {
            rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(pause_s)));
        }
    }

    // If something failed, try to return to 'home' afterwards.
    if (rclcpp::ok() && any_failure && return_home_on_failure && !targets.empty())
    {
        size_t home_index = targets.size() - 1;
        if (home_index_param >= 0)
        {
            const auto idx = static_cast<size_t>(home_index_param);
            if (idx < targets.size())
            {
                home_index = idx;
            }
            else
            {
                RCLCPP_WARN(node->get_logger(), "home_index=%d out of range (targets=%zu); using last target as home", home_index_param, targets.size());
            }
        }

        RCLCPP_WARN(node->get_logger(), "Attempting return-to-home after failure at index %zu (home_index=%zu)", failed_index, home_index);
        (void)node->sendTarget(targets[home_index], home_index);
    }

    RCLCPP_INFO(node->get_logger(), "Demo sequence complete.");
    rclcpp::shutdown();
    return 0;
}

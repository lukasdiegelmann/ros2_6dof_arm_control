// go_to_pose_node.cpp (Auszug)
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "arm_apps/urdf_chain_fk.hpp"
#include "arm_apps/joint_mapping.hpp"

class GoToPoseNode : public rclcpp::Node
{
public:
    GoToPoseNode() : Node("go_to_pose_node")
    {
        this->declare_parameter<std::string>("base_link", "base_link");
        this->declare_parameter<std::string>("tip_link", "tool0");

        const auto base_link = this->get_parameter("base_link").as_string();
        const auto tip_link = this->get_parameter("tip_link").as_string();

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

        // Subscriptions
        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&GoToPoseNode::onJointState, this, std::placeholders::_1));

        target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/target_pose", 10,
            std::bind(&GoToPoseNode::onTargetPose, this, std::placeholders::_1));
    }

private:
    void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        last_js_ = *msg;
        have_js_ = true;

        // (Debug) FK check: 1x pro Sekunde loggen
        const auto now = this->now();
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

    void onTargetPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!have_js_)
        {
            RCLCPP_WARN(get_logger(), "No /joint_states yet, cannot solve IK.");
            return;
        }
        // Nächster Schritt: IK + publish trajectory (kommt als Step 3 unten)
        RCLCPP_INFO(get_logger(), "Received target pose (frame=%s)",
                    msg->header.frame_id.c_str());
    }

    arm_apps::UrdfChainFK fk_;
    arm_apps::JointMapping mapping_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;

    sensor_msgs::msg::JointState last_js_;
    bool have_js_{false};
    rclcpp::Time last_fk_log_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoToPoseNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

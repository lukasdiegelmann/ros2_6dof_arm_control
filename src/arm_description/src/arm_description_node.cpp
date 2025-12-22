#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class ArmDescriptionNode : public rclcpp::Node
{
public:
  ArmDescriptionNode() : Node("arm_description_node")
  {
    
  }
};
#pragma once
#include <string>
#include <vector>
#include <unordered_map>

#include <sensor_msgs/msg/joint_state.hpp>

namespace arm_apps
{

    class JointMapping
    {
    public:
        void init(const std::vector<std::string> &chain_joint_names);
        std::vector<double> toChainOrder(const sensor_msgs::msg::JointState &js) const;

    private:
        std::vector<std::string> chain_joint_names_;
    };

} // namespace arm_apps

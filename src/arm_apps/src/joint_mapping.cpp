#include "arm_apps/joint_mapping.hpp"
#include <stdexcept>

namespace arm_apps {

void JointMapping::init(const std::vector<std::string>& chain_joint_names) {
  chain_joint_names_ = chain_joint_names;
}

std::vector<double> JointMapping::toChainOrder(const sensor_msgs::msg::JointState& js) const {
  if (chain_joint_names_.empty())
    throw std::runtime_error("JointMapping not initialized");

  std::unordered_map<std::string, double> pos;
  pos.reserve(js.name.size());
  for (size_t i = 0; i < js.name.size(); ++i) {
    if (i < js.position.size())
      pos[js.name[i]] = js.position[i];
  }

  std::vector<double> q(chain_joint_names_.size(), 0.0);
  for (size_t i = 0; i < chain_joint_names_.size(); ++i) {
    auto it = pos.find(chain_joint_names_[i]);
    if (it == pos.end()) {
      throw std::runtime_error("JointState missing joint: " + chain_joint_names_[i]);
    }
    q[i] = it->second;
  }
  return q;
}

}  // namespace arm_apps

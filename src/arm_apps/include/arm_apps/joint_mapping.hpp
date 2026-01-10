#pragma once
#include <string>
#include <vector>
#include <unordered_map>

#include <sensor_msgs/msg/joint_state.hpp>

namespace arm_apps
{

    /**
     * @brief Maps ROS JointState messages into the robot's kinematic chain joint order.
     *
     * Many parts of the motion pipeline (FK/IK/trajectory planning) operate on joint
     * vectors in a fixed, deterministic order: the order of joints in the kinematic
     * chain (as provided by the URDF/FK model). However, incoming
     * `sensor_msgs::msg::JointState` messages can contain joints in any order.
     *
     * `JointMapping` stores the chain joint names and provides a conversion from a
     * `JointState` into a `std::vector<double>` in chain order.
     */

    class JointMapping
    {
    public:
        /**
         * @brief Initialize the mapping from the kinematic chain joint names.
         *
         * Call this once after the FK/URDF chain is initialized.
         *
         * @param chain_joint_names Joint names in the exact order expected by the
         *                          kinematic chain (e.g. `UrdfChainFK::jointNames()`).
         */
        void init(const std::vector<std::string> &chain_joint_names);

        /**
         * @brief Convert a JointState into a joint vector in chain order.
         *
         * The returned vector has the same length and ordering as the chain joint
         * list provided in `init()`.
         *
         * @param js JointState message containing `name[]` and `position[]`.
         * @return Joint positions in chain joint order.
         *
         * @throws std::runtime_error (or other std::exception) if the JointState does
         *         not contain all required joints, or if the message is inconsistent
         *         (e.g. different lengths for name/position).
         */
        std::vector<double> toChainOrder(const sensor_msgs::msg::JointState &js) const;

    private:
        /// Joint names in the kinematic chain order.
        std::vector<std::string> chain_joint_names_;
    };

}

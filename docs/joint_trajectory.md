# Joint Trajectory API

This document describes the small joint-trajectory helper API declared in
[src/arm_apps/include/arm_apps/joint_trajectory.hpp](src/arm_apps/include/arm_apps/joint_trajectory.hpp).

**Overview**
- **`PlannedTrajectory`**: A compact representation of a planned joint-space
  trajectory. It contains a list of joint-space waypoints and a matching list
  of absolute times (seconds) for each waypoint.
- **`planJointTrajectory(...)`**: Plans a trajectory between two joint vectors
  while respecting joint limits.
- **`toRosTrajectoryMsg(...)`**: Converts a `PlannedTrajectory` into a ROS
  `trajectory_msgs::msg::JointTrajectory` message.

**`PlannedTrajectory`**
- `q` : vector of waypoints. Each waypoint is a `std::vector<double>` of joint
  positions (length = number of joints).
- `t` : vector of absolute times (seconds). `t.size() == q.size()` and each
  `t[i]` gives the time at which `q[i]` should be reached.

Notes:
- Times in `t` are absolute (seconds). When converting to a ROS message with
  `toRosTrajectoryMsg`, the provided `stamp` is used as the trajectory header
  stamp and each point's `time_from_start` is computed relative to that
  `stamp`.

**`planJointTrajectory(...)`**
Signature summary:

```
PlannedTrajectory planJointTrajectory(
    const std::vector<double> &q_start,
    const std::vector<double> &q_goal,
    const JointLimits &limits,
    size_t num_waypoints = 50,
    double vel_scale = 0.5,
    double max_acc = 0.0
);
```

- `q_start`, `q_goal`: start and goal joint vectors (same dimensionality).
- `limits`: `JointLimits` object used to enforce per-joint limits (velocities,
  positions, etc.). See the `JointLimits` declaration for details.
- `num_waypoints`: number of discrete waypoints to generate (default 50).
- `vel_scale`: global scale applied to velocities (0..1). Lower values produce
  slower motion.
- `max_acc`: maximum acceleration (<= 0 means acceleration limits are ignored).

Behavior (informative):
- The function produces `num_waypoints` joint-space waypoints between
  `q_start` and `q_goal` and computes times so that the trajectory respects
  the provided limits and velocity scaling. The concrete interpolation and
  enforcement strategy is implemented in the source corresponding to the
  header.

**`toRosTrajectoryMsg(...)`**
Signature summary:

```
trajectory_msgs::msg::JointTrajectory toRosTrajectoryMsg(
    const PlannedTrajectory &traj,
    const std::vector<std::string> &joint_names,
    const rclcpp::Time &stamp
);
```

- `traj`: the `PlannedTrajectory` to convert.
- `joint_names`: list of joint names, order must match the joint values in
  each waypoint.
- `stamp`: `rclcpp::Time` applied to the returned message header. Each point's
  `time_from_start` is computed relative to `stamp` using the values in
  `traj.t`.

The produced `trajectory_msgs::msg::JointTrajectory` populates:
- `header.stamp` with `stamp`.
- `joint_names` with the provided names.
- `points` such that each point's `positions` are the corresponding entry in
  `traj.q`, and `time_from_start` corresponds to `traj.t` relative to
  `stamp`.

**Example usage**

```cpp
#include "arm_apps/joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"

// build a simple trajectory and convert to a ROS message
void example(rclcpp::Clock::SharedPtr clock) {
    std::vector<double> start = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> goal  = {0.5, -0.3, 0.4, -0.2, 0.1, 0.0};
    JointLimits limits; // populate as appropriate for your robot

    auto traj = arm_apps::planJointTrajectory(start, goal, limits, 50, 0.5);
    rclcpp::Time stamp = clock->now();
    auto msg = arm_apps::toRosTrajectoryMsg(traj, {"joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"}, stamp);

    // publish `msg` on your trajectory topic
}
```

**See also**
- Header: [src/arm_apps/include/arm_apps/joint_trajectory.hpp](src/arm_apps/include/arm_apps/joint_trajectory.hpp)

**Next steps / Suggestions**
- Add a short code example in the corresponding node that publishes the
  produced message so readers can quickly run it in the repo's demo.
- If you maintain a docs site, include this markdown under your generated
  reference pages.

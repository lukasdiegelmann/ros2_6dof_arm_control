#pragma once

#include "arm_apps/joint_limits.hpp"

#include <rclcpp/time.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <cstddef>
#include <string>
#include <vector>

namespace arm_apps {

/**
 * @brief Planned joint trajectory with waypoints and corresponding times.
 *
 * The trajectory representation is intentionally simple: a sequence of
 * joint-space waypoints `q` and a matching vector of absolute times `t`
 * (seconds). For each index i, `q[i]` contains a vector of joint
 * positions and `t[i]` is the absolute time at which that waypoint should
 * be reached. Consumers can convert this into ROS messages or other
 * representations as needed.
 */
struct PlannedTrajectory {
  /// Waypoints: each inner vector is a full joint position vector.
  std::vector<std::vector<double>> q;

  /// Absolute times (seconds) for each waypoint. Must have same length
  /// as `q`. `t[i]` is the time for waypoint `q[i]`.
  std::vector<double> t;
};

/**
 * @brief Plan a joint-space trajectory from `q_start` to `q_goal`.
 *
 * Generates a `PlannedTrajectory` containing `num_waypoints` intermediate
 * waypoints and absolute times that respect the provided `limits`.
 *
 * @param q_start Start joint vector. Size must equal robot DOF.
 * @param q_goal Goal joint vector. Same size as `q_start`.
 * @param limits Joint limits (position, velocity, etc.) used to bound the
 *               planned motion.
 * @param num_waypoints Number of discrete waypoints to generate (default
 *                      50). Includes start and goal points.
 * @param vel_scale Global velocity scale in range (0..1]. Values < 1
 *                  produce proportionally slower motion (default 0.5).
 * @param max_acc Default maximum acceleration used for time
 *                parameterization if per-joint acceleration limits are not
 *                available. Values <= 0 indicate acceleration limits are
 *                ignored (backwards compatible, default 0.0).
 * @param desired_duration Optional desired total trajectory duration in
 *                         seconds. If > 0, timing will be scaled to match
 *                         the requested duration when possible without
 *                         violating limits. If <= 0, duration scaling is
 *                         disabled (default 0.0).
 * @return A `PlannedTrajectory` with `q.size() == t.size()`.
 */
PlannedTrajectory planJointTrajectory(const std::vector<double>& q_start,
                                      const std::vector<double>& q_goal, const JointLimits& limits,
                                      size_t num_waypoints = 50, double vel_scale = 0.5,
                                      double max_acc = 0.0, double desired_duration = 0.0);

/**
 * @brief Convert a `PlannedTrajectory` into a ROS `JointTrajectory` msg.
 *
 * The returned message's `header.stamp` is set to `stamp`. Each point's
 * `time_from_start` is computed as `traj.t[i] - traj.t[0]` relative to the
 * provided `stamp` (or otherwise relative to `stamp` such that `stamp` +
 * `time_from_start` equals the absolute time given in `traj.t[i]`). The
 * order of joints in `joint_names` must match the order of values in each
 * waypoint vector in `traj.q`.
 *
 * @param traj Planned trajectory to convert.
 * @param joint_names Names of the joints, in the same order as values in
 *                    each waypoint of `traj.q`.
 * @param stamp Timestamp to use for the message header.
 * @return A populated `trajectory_msgs::msg::JointTrajectory` message.
 */
trajectory_msgs::msg::JointTrajectory toRosTrajectoryMsg(
    const PlannedTrajectory& traj, const std::vector<std::string>& joint_names,
    const rclcpp::Time& stamp);

}  // namespace arm_apps
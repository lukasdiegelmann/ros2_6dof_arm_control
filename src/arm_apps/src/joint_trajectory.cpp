#include "arm_apps/joint_trajectory.hpp"

#include <builtin_interfaces/msg/duration.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace arm_apps {
namespace {
// Minimum time step between trajectory points.
// With ros2_control controller_manager update_rate=100Hz (10ms), smaller
// time deltas can lead to points being effectively skipped/interpolated in a
// way that looks like brief pauses and jumps in simulation.
constexpr double kMinDt = 1e-2;
constexpr double kFallbackMaxVel = 1.0;  // rad/s or m/s depending on joint type

inline void validateSameSizeOrThrow(const std::vector<double>& a, const std::vector<double>& b,
                                    const char* what) {
  if (a.size() != b.size()) {
    throw std::runtime_error(std::string(what) + ": size mismatch");
  }
}

builtin_interfaces::msg::Duration durationFromSeconds(double seconds) {
  if (!std::isfinite(seconds) || seconds < 0.0) {
    throw std::runtime_error("toRosTrajectoryMsg: non-finite or negative time");
  }

  const double sec_floor = std::floor(seconds);
  const double frac = seconds - sec_floor;

  auto sec = static_cast<int32_t>(sec_floor);
  auto nsec = static_cast<uint32_t>(std::llround(frac * 1e9));

  if (nsec >= 1000000000u) {
    ++sec;
    nsec -= 1000000000u;
  }

  builtin_interfaces::msg::Duration out;
  out.sec = sec;
  out.nanosec = nsec;
  return out;
}

double computeSegmentDt(const std::vector<double>& q0,  // waypoint i-1
                        const std::vector<double>& q1,  // waypoint i
                        const JointLimits& limits,      // per-joint limits in FK order
                        double vel_scale,               // velocity scaling factor (0..1]
                        double max_acc)  // default accel if per-joint not set; <=0 disables accel
{
  validateSameSizeOrThrow(q0, q1, "computeSegmentDt");
  if (q0.empty()) {
    throw std::runtime_error("computeSegmentDt: empty waypoint");
  }
  if (limits.dof() != q0.size()) {
    throw std::runtime_error("computeSegmentDt: waypoint size does not match limits.dof()");
  }
  if (!std::isfinite(vel_scale) || vel_scale <= 0.0) {
    throw std::runtime_error("computeSegmentDt: vel_scale must be finite and > 0");
  }

  double dt = 0.0;
  bool any_valid_joint = false;
  double max_abs_dq = 0.0;

  for (size_t j = 0; j < q0.size(); ++j) {
    const double dq = std::abs(q1[j] - q0[j]);
    max_abs_dq = std::max(max_abs_dq, dq);

    const double vmax = limits.maxVelocity(j);
    const double v_eff = vmax * vel_scale;

    // If velocity is invalid, ignore this joint for timing.
    if (!std::isfinite(v_eff) || v_eff <= 0.0) {
      continue;
    }

    double t_j = 0.0;

    // Backwards compatible mode: ignore accel limits.
    if (!std::isfinite(max_acc) || max_acc <= 0.0) {
      t_j = dq / v_eff;
    } else {
      const double amax_j = limits.maxAcceleration(j);

      // If per-joint accel is not set (typically +inf), use the provided default.
      const double a_eff = (std::isfinite(amax_j) && amax_j > 0.0) ? amax_j : max_acc;

      // If accel is invalid, fall back to velocity-only.
      if (!std::isfinite(a_eff) || a_eff <= 0.0) {
        t_j = dq / v_eff;
      } else if (dq <= 0.0) {
        t_j = 0.0;
      } else {
        // Trapezoid vs triangle decision:
        // d_acc is the distance covered while accelerating and decelerating
        // when reaching v_eff (i.e., both ramps combined).
        const double d_acc = (v_eff * v_eff) / a_eff;

        if (dq >= d_acc) {
          // Trapezoidal profile: accelerate to v_eff, cruise, decelerate.
          const double t_acc = v_eff / a_eff;
          const double t_cruise = (dq - d_acc) / v_eff;
          t_j = 2.0 * t_acc + t_cruise;
        } else {
          // Triangular profile: v_eff is not reached.
          const double t_acc = std::sqrt(dq / a_eff);
          t_j = 2.0 * t_acc;
        }
      }
    }

    if (!std::isfinite(t_j) || t_j < 0.0) {
      // Conservative fallback for this joint: velocity-only.
      t_j = dq / v_eff;
    }

    any_valid_joint = true;
    dt = std::max(dt, t_j);
  }

  // If no joint had a usable velocity limit, use a fallback max velocity.
  if (!any_valid_joint) {
    dt = max_abs_dq / (kFallbackMaxVel * vel_scale);
  }

  // Ensure dt is monotonic and non-zero.
  if (!std::isfinite(dt) || dt < kMinDt) {
    dt = kMinDt;
  }

  return dt;
}
}  // namespace

PlannedTrajectory planJointTrajectory(const std::vector<double>& q_start,
                                      const std::vector<double>& q_goal, const JointLimits& limits,
                                      size_t num_waypoints, double vel_scale, double max_acc,
                                      double desired_duration) {
  /*
   * Validate if the q inputs have the same size. If not, throw an exception.
   * This should never happen if the caller uses the API correctly. Also check
   * for empty inputs and valid dof.
   */
  validateSameSizeOrThrow(q_start, q_goal, "planJointTrajectory");

  if (q_start.empty()) {
    throw std::runtime_error("planJointTrajectory: q_start is empty");
  }

  if (q_goal.empty()) {
    throw std::runtime_error("planJointTrajectory: q_goal is empty");
  }

  if (limits.dof() != q_start.size()) {
    throw std::runtime_error("planJointTrajectory: q size does not match limits.dof()");
  }

  /*
   * Ensure there are at least two waypoints (start and goal).
   */
  if (num_waypoints < 2) {
    num_waypoints = 2;
  }

  /*
   * Validate vel_scale input, should be finite and > 0. If not, throw
   * an exception.
   */
  if (!std::isfinite(vel_scale) || vel_scale <= 0.0) {
    throw std::runtime_error("planJointTrajectory: vel_scale must be finite and > 0");
  }

  /*
   * Prepare output trajectory structure with pre-sized vectors. Fill with
   * zeros for now. The actual values will be filled in later.
   */
  PlannedTrajectory traj;
  traj.q.resize(num_waypoints, std::vector<double>(q_start.size(), 0.0));
  traj.t.resize(num_waypoints, 0.0);

  /*
   * Linear interpolation between q_start and q_goal for each waypoint.
   * The interpolation factor alpha goes from 0.0 (start) to 1.0 (goal).
   * After computing each waypoint, clamp it to the joint limits.
   */
  for (size_t i = 0; i < num_waypoints; ++i) {
    /* Calculate the interpolation factor */
    const double alpha = (num_waypoints == 1)
                             /* Edge Case: Only one waypoint */
                             ? 1.0
                             /* Normal case: interpolate between start and goal */
                             : static_cast<double>(i) / static_cast<double>(num_waypoints - 1);

    /*
     * Fill the joint values for this waypoint by linear interpolation.
     * Get a pointer to the current waypoint vector for easier access.
     */
    auto& q = traj.q[i];
    for (size_t j = 0; j < q.size(); ++j) {
      /* (LERP) */
      q[j] = (1.0 - alpha) * q_start[j] + alpha * q_goal[j];
    }

    limits.clamp(q);
  }

  /*
   * At this point all waypoints have been filled in the traj.q vector. Now we
   * need to compute the absolute times for each waypoint based on the joint limits,
   * vel_scale, and (optionally) acceleration limits.
   *
   * Per segment, time is chosen as the maximum across joints.
   *
   * If max_acc <= 0: velocity-only time parameterization (backwards compatible).
   * Otherwise: trapezoidal/triangular velocity profile per joint.
   */
  traj.t[0] = 0.0; /* start time */
  for (size_t i = 1; i < num_waypoints; ++i) {
    const double dt = computeSegmentDt(traj.q[i - 1], traj.q[i], limits, vel_scale, max_acc);
    traj.t[i] = traj.t[i - 1] + dt;
  }

  // Optional duration scaling: slowing down (scale >= 1) is always safe.
  // Speeding up (scale < 1) is only applied if it still respects per-segment
  // constraints computed from the same limits.
  if (std::isfinite(desired_duration) && desired_duration > 0.0) {
    const double current_duration = traj.t.back() - traj.t.front();
    if (std::isfinite(current_duration) && current_duration > 0.0) {
      const double scale = desired_duration / current_duration;

      if (std::isfinite(scale) && scale > 0.0) {
        if (scale >= 1.0) {
          for (size_t i = 1; i < traj.t.size(); ++i) {
            traj.t[i] *= scale;
          }
        } else {
          bool can_speed_up = true;
          for (size_t i = 1; i < traj.q.size(); ++i) {
            const double dt0 = traj.t[i] - traj.t[i - 1];
            const double dt_scaled = dt0 * scale;
            const double dt_req =
                computeSegmentDt(traj.q[i - 1], traj.q[i], limits, vel_scale, max_acc);

            if (!std::isfinite(dt_scaled) || dt_scaled + 1e-12 < dt_req) {
              can_speed_up = false;
              break;
            }
          }

          if (can_speed_up) {
            for (size_t i = 1; i < traj.t.size(); ++i) {
              traj.t[i] *= scale;
            }
          }
        }
      }
    }
  }

  return traj;
}

trajectory_msgs::msg::JointTrajectory toRosTrajectoryMsg(
    const PlannedTrajectory& traj, const std::vector<std::string>& joint_names,
    const rclcpp::Time& stamp) {
  if (traj.q.size() != traj.t.size()) {
    throw std::runtime_error("toRosTrajectoryMsg: traj.q and traj.t size mismatch");
  }

  if (traj.q.empty()) {
    throw std::runtime_error("toRosTrajectoryMsg: empty trajectory");
  }

  if (joint_names.empty()) {
    throw std::runtime_error("toRosTrajectoryMsg: joint_names is empty");
  }

  for (size_t i = 0; i < traj.q.size(); ++i) {
    if (traj.q[i].size() != joint_names.size()) {
      throw std::runtime_error("toRosTrajectoryMsg: waypoint size does not match joint_names size");
    }
  }

  trajectory_msgs::msg::JointTrajectory msg;
  msg.header.stamp = stamp;
  msg.joint_names = joint_names;
  msg.points.resize(traj.q.size());

  const double t0 = traj.t.front();

  // Compute velocities using finite differences to help the trajectory controller's
  // spline interpolation. This reduces "stop/go" artifacts when only positions are
  // commanded.
  const size_t n = traj.q.size();
  const size_t dof = joint_names.size();

  for (size_t i = 0; i < n; ++i) {
    auto& pt = msg.points[i];
    pt.positions = traj.q[i];
    pt.velocities.assign(dof, 0.0);
    pt.time_from_start = durationFromSeconds(traj.t[i] - t0);

    if (n < 2) {
      continue;
    }

    const size_t i_prev = (i == 0) ? 0 : (i - 1);
    const size_t i_next = (i + 1 >= n) ? (n - 1) : (i + 1);

    const double t_prev = traj.t[i_prev];
    const double t_next = traj.t[i_next];
    const double dt = t_next - t_prev;

    if (!std::isfinite(dt) || dt <= 1e-9) {
      continue;
    }

    // Keep boundary conditions well-defined for spline interpolation.
    // Many controllers require the final velocity to be exactly zero.
    if (i == 0 || i + 1 == n) {
      continue;
    }

    for (size_t j = 0; j < dof; ++j) {
      const double dq = traj.q[i_next][j] - traj.q[i_prev][j];
      const double v = dq / dt;
      pt.velocities[j] = std::isfinite(v) ? v : 0.0;
    }
  }

  return msg;
}

}  // namespace arm_apps
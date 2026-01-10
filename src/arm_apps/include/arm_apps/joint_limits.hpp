#pragma once

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "urdf_chain_fk.hpp"

namespace arm_apps {
  /**
   * @brief Position/velocity limits for a single joint.
   *
   * Position limits use +/-infinity if not applicable or unknown.
   * Velocity uses +/-infinity if unknown.
   */
  struct JointLimit {
    double min_position;
    double max_position;
    double max_velocity;
    double max_acceleration;
  };

  /**
   * @brief Joint limit utilities derived from the URDF referenced by an UrdfChainFK.
   *
   * Limits are stored in the exact order of @c fk.jointNames(). All input vectors
   * passed to @ref clamp() / @ref withinLimits() must follow the same order and have
   * the same size.
   */
  class JointLimits {
   public:
    /**
     * @brief Construct and cache joint limits from the given FK helper.
     *
     * For each joint name in @c fk.jointNames(), this queries the URDF joint via
     * @c fk.urdfModel().getJoint(name) and applies:
     * - REVOLUTE/PRISMATIC: min/max from URDF lower/upper (if present)
     * - CONTINUOUS: no position limits (min=-inf, max=+inf)
     * - otherwise: fallback to +/-inf
     *
     * Velocity is read from URDF if present, else set to +inf.
     */
    explicit JointLimits(const UrdfChainFK& fk);

    /**
     * @brief Clamp joint positions in-place to the cached position limits.
     *
     * Non-finite values (NaN/Inf) are left unchanged.
     *
     * @param[in,out] q Joint positions in FK chain order.
     * @throws std::runtime_error if @p q size differs from the number of cached limits.
     */
    void clamp(std::vector<double>& q) const;

    /**
     * @brief Check whether all joint positions are within cached position limits.
     *
     * Any non-finite value (NaN/Inf) fails the check.
     *
     * @param q Joint positions in FK chain order.
     * @return True if all joints are finite and within [min_position, max_position].
     * @throws std::runtime_error if @p q size differs from the number of cached limits.
     */
    bool withinLimits(const std::vector<double>& q) const;

    /**
     * @brief Number of joints (DoF) this JointLimits instance covers.
     */
    size_t dof() const;

    /**
     * @brief Max velocity limit for joint @p idx.
     *
     * Returns +inf if unknown (as loaded from URDF), or throws if idx is out of range.
     */
    double maxVelocity(size_t idx) const;

    /**
     * @brief Max acceleration limit for joint @p idx.
     *
     * Returns +inf if unknown (as loaded from URDF), or throws if idx is out of range.
     */
    double maxAcceleration(size_t idx) const;

   private:
    std::vector<JointLimit> limits_;
  };

};  // namespace arm_apps

#pragma once

#include "urdf_chain_fk.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace arm_apps
{
    /**
     * @brief Position/velocity limits for a single joint.
     *
     * Position limits use +/-infinity if not applicable or unknown.
     * Velocity uses +/-infinity if unknown.
     */
    struct JointLimit
    {
        double min_position;
        double max_position;
        double max_velocity;
    };

    /**
     * @brief Joint limit utilities derived from the URDF referenced by an UrdfChainFK.
     *
     * Limits are stored in the exact order of @c fk.jointNames(). All input vectors
     * passed to @ref clamp() / @ref withinLimits() must follow the same order and have
     * the same size.
     */
    class JointLimits
    {
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
        explicit JointLimits(const UrdfChainFK &fk);

        /**
         * @brief Clamp joint positions in-place to the cached position limits.
         *
         * Non-finite values (NaN/Inf) are left unchanged.
         *
         * @param[in,out] q Joint positions in FK chain order.
         * @throws std::runtime_error if @p q size differs from the number of cached limits.
         */
        void clamp(std::vector<double> &q) const;

        /**
         * @brief Check whether all joint positions are within cached position limits.
         *
         * Any non-finite value (NaN/Inf) fails the check.
         *
         * @param q Joint positions in FK chain order.
         * @return True if all joints are finite and within [min_position, max_position].
         * @throws std::runtime_error if @p q size differs from the number of cached limits.
         */
        bool withinLimits(const std::vector<double> &q) const;

    private:
        std::vector<JointLimit> limits_;
    };

};

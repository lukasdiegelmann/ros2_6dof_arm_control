#include "arm_apps/joint_limits.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <urdf/model.h>

namespace arm_apps
{
    JointLimits::JointLimits(const UrdfChainFK &fk)
    {
        /*
         * Extract joint names and URDF model from the provided UrdfChainFK instance.
         */
        const auto &joint_names = fk.jointNames();
        const urdf::Model &model = fk.urdfModel();

        limits_.clear();
        limits_.reserve(joint_names.size());

        const auto inf = std::numeric_limits<double>::infinity();

        /*
         * Iterate through all joint names in the FK chain and extract their limits
         * from the URDF model. If a joint has no limits defined in the URDF, fallback
         * to +/- infinity for position and velocity.
         */
        for (const auto &name : joint_names)
        {
            /*
             * Create JointLimit struct for this joint, this will be filled
             * below with data from the URDF if available.
             */
            JointLimit lim;

            /*
             * Default limits: +/- infinity for position and velocity
             */
            lim.min_position = -inf;
            lim.max_position = inf;
            lim.max_velocity = inf;

            urdf::JointConstSharedPtr joint = model.getJoint(name);
            if (joint)
            {
                if (joint->limits)
                {
                    lim.max_velocity = static_cast<double>(joint->limits->velocity);
                }

                switch (joint->type)
                {
                case urdf::Joint::REVOLUTE:
                case urdf::Joint::PRISMATIC:
                    if (joint->limits)
                    {
                        lim.min_position = static_cast<double>(joint->limits->lower);
                        lim.max_position = static_cast<double>(joint->limits->upper);
                    }
                    break;
                case urdf::Joint::CONTINUOUS:
                    // No position limits (stay at +/-inf). Keep max_velocity if available.
                    break;
                default:
                    // Fixed/Floating/Planar/Unknown: leave fallback +/-inf.
                    break;
                }
            }

            limits_.push_back(lim);
        }
    }

    void JointLimits::clamp(std::vector<double> &q) const
    {
        if (q.size() != limits_.size())
        {
            throw std::runtime_error("JointLimits::clamp: q size does not match limits size");
        }

        for (std::size_t i = 0; i < q.size(); ++i)
        {
            if (!std::isfinite(q[i]))
            {
                continue;
            }

            const double min_p = limits_[i].min_position;
            const double max_p = limits_[i].max_position;

            q[i] = std::clamp(q[i], min_p, max_p);
        }
    }

    bool JointLimits::withinLimits(const std::vector<double> &q) const
    {
        if (q.size() != limits_.size())
        {
            throw std::runtime_error("JointLimits::withinLimits: q size does not match limits size");
        }

        for (std::size_t i = 0; i < q.size(); ++i)
        {
            if (!std::isfinite(q[i]))
            {
                return false;
            }

            const double min_p = limits_[i].min_position;
            const double max_p = limits_[i].max_position;
            if (q[i] < min_p || q[i] > max_p)
            {
                return false;
            }
        }

        return true;
    }
}
#pragma once

#include <vector>
#include <geometry_msgs/msg/pose.hpp>

#include "arm_apps/urdf_chain_fk.hpp"

namespace arm_apps
{

    struct IkParams
    {
        int max_iters = 200;
        double pos_tol = 1e-3; // meters
        double rot_tol = 1e-2; // radians
        double lambda = 0.05;  // damping
        double alpha = 0.5;    // step size
        double eps = 1e-5;     // numeric jacobian epsilon
    };

    struct IkResult
    {
        bool success = false;
        int iterations = 0;
        double pos_err = 0.0;
        double rot_err = 0.0;
        std::vector<double> q; // solution in chain order
    };

    IkResult solveIkDls(
        const UrdfChainFK &fk,
        const geometry_msgs::msg::Pose &target_pose,
        const std::vector<double> &q_init,
        const IkParams &params);

} // namespace arm_apps

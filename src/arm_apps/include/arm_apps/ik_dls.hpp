#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <vector>

#include "arm_apps/urdf_chain_fk.hpp"

namespace arm_apps {

  /**
   * @brief Configuration parameters for the Damped Least-Squares IK solver.
   *
   * These parameters tune the convergence behaviour of `solveIkDls`.
   * - `max_iters`: maximum number of solver iterations.
   * - `pos_tol` / `rot_tol`: termination tolerances (meters / radians).
   * - `lambda`: damping factor used in the DLS update (higher lambda -> more
   *   conservative updates, helps near singularities).
   * - `alpha`: step size multiplier applied to the computed joint update.
   * - `eps`: finite-difference epsilon for numeric Jacobian computation.
   */
  struct IkParams {
    int max_iters = 200;
    double pos_tol = 1e-3;  // meters
    double rot_tol = 1e-2;  // radians
    double lambda = 0.05;   // damping
    double alpha = 0.5;     // step size
    double eps = 1e-5;      // numeric jacobian epsilon
  };

  /**
   * @brief Result of the IK solver.
   *
   * - `success` indicates whether the solver reached the termination tolerances
   *   within `max_iters` iterations.
   * - `iterations` contains the number of iterations performed (0..max_iters).
   * - `pos_err` / `rot_err` are the final Cartesian position and orientation
   *   errors (meters / radians).
   * - `q` contains the returned joint solution in chain order. Even on
   *   failure the solver returns the best-effort joint vector in `q`.
   */
  struct IkResult {
    bool success = false;
    int iterations = 0;
    double pos_err = 0.0;
    double rot_err = 0.0;
    std::vector<double> q;  // solution in chain order
  };

  /**
   * @brief Optional per-iteration logging output for IK convergence evaluation.
   *
   * When provided to @ref solveIkDls, the solver appends the position and
   * orientation error norms for each iteration.
   */
  struct IkTrace {
    std::vector<double> pos_err_m;
    std::vector<double> rot_err_rad;
  };

  /**
   * @brief Solve inverse kinematics using the Damped Least-Squares (DLS) method.
   *
   * This is a numeric IK solver that attempts to find a joint vector `q` such
   * that the forward kinematics of `fk` reaches `target_pose` within the
   * tolerances specified in `params`.
   *
   * @param fk Forward-kinematics helper describing the kinematic chain. The
   *           chain order of `fk.jointNames()` defines the expected ordering
   *           of `q_init` and the returned `IkResult.q`.
   * @param target_pose Desired end-effector pose (position + orientation).
   * @param q_init Initial guess for the joint vector (must match chain DOF).
   * @param params Solver parameters (see `IkParams`).
   * @return IkResult with final `q`, convergence flags and error metrics.
   *
   * The function performs at most `params.max_iters` iterations and returns
   * `success=true` when both `pos_err < params.pos_tol` and
   * `rot_err < params.rot_tol`. On failure it returns `success=false` but
   * still provides the best-effort joint vector in `q`.
   *
   * The solver uses a numeric Jacobian (finite differences, epsilon=`params.eps`)
   * and a DLS update rule with damping `params.lambda`.
   */
  IkResult solveIkDls(const UrdfChainFK& fk, const geometry_msgs::msg::Pose& target_pose,
                      const std::vector<double>& q_init, const IkParams& params,
                      IkTrace* trace = nullptr);

}  // namespace arm_apps

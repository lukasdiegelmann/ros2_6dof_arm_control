#include "arm_apps/ik_dls.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <kdl/frames.hpp>
#include <stdexcept>

namespace arm_apps {
  namespace {

    static Eigen::Vector3d kdlVecToEigen(const KDL::Vector& v) {
      return Eigen::Vector3d(v.x(), v.y(), v.z());
    }

    static Eigen::Matrix3d kdlRotToEigen(const KDL::Rotation& R) {
      double x, y, z, w;
      R.GetQuaternion(x, y, z, w);
      Eigen::Quaterniond q(w, x, y, z);
      q.normalize();
      return q.toRotationMatrix();
    }

    static Eigen::Quaterniond msgQuatToEigen(const geometry_msgs::msg::Quaternion& q) {
      Eigen::Quaterniond qe(q.w, q.x, q.y, q.z);
      // normalize to be safe
      qe.normalize();
      return qe;
    }

    // Rotation matrix -> rotation vector (axis * angle)
    static Eigen::Vector3d rotMatToRotVec(const Eigen::Matrix3d& R) {
      Eigen::AngleAxisd aa(R);
      double angle = aa.angle();
      Eigen::Vector3d axis = aa.axis();

      // handle tiny angles gracefully
      if (std::isfinite(angle) && angle < 1e-12) {
        return Eigen::Vector3d::Zero();
      }
      if (!std::isfinite(angle)) {
        // fallback: no rotation
        return Eigen::Vector3d::Zero();
      }

      return axis * angle;
    }

    static void fkPose(const UrdfChainFK& fk, const std::vector<double>& q, Eigen::Vector3d& p_out,
                       Eigen::Matrix3d& R_out) {
      const KDL::Frame T = fk.fk(q);
      p_out = kdlVecToEigen(T.p);
      R_out = kdlRotToEigen(T.M);
    }

    // 6x6 numeric jacobian (pos + rotvec) w.r.t joints
    static Eigen::Matrix<double, 6, Eigen::Dynamic> numericJacobian(const UrdfChainFK& fk,
                                                                    const std::vector<double>& q,
                                                                    double eps) {
      const int n = static_cast<int>(q.size());
      Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, n);
      J.setZero();

      Eigen::Vector3d p1;
      Eigen::Matrix3d R1;
      fkPose(fk, q, p1, R1);

      for (int i = 0; i < n; ++i) {
        std::vector<double> q2 = q;
        q2[i] += eps;

        Eigen::Vector3d p2;
        Eigen::Matrix3d R2;
        fkPose(fk, q2, p2, R2);

        Eigen::Vector3d dp = (p2 - p1) / eps;

        // orientation difference: R_delta = R2 * R1^T
        Eigen::Matrix3d R_delta = R2 * R1.transpose();
        Eigen::Vector3d drot = rotMatToRotVec(R_delta) / eps;

        J.block<3, 1>(0, i) = dp;
        J.block<3, 1>(3, i) = drot;
      }

      return J;
    }

  }  // namespace

  IkResult solveIkDls(const UrdfChainFK& fk, const geometry_msgs::msg::Pose& target_pose,
                      const std::vector<double>& q_init, const IkParams& params, IkTrace* trace) {
    const int n = static_cast<int>(q_init.size());
    if (n <= 0)
      throw std::runtime_error("IK: q_init empty");
    if (static_cast<int>(fk.dof()) != n)
      throw std::runtime_error("IK: dof mismatch");

    // target in Eigen
    const Eigen::Vector3d p_t(target_pose.position.x, target_pose.position.y,
                              target_pose.position.z);

    const Eigen::Quaterniond q_t = msgQuatToEigen(target_pose.orientation);
    const Eigen::Matrix3d R_t = q_t.toRotationMatrix();

    std::vector<double> q = q_init;

    IkResult res;
    res.q = q;

    if (trace) {
      trace->pos_err_m.clear();
      trace->rot_err_rad.clear();
      trace->pos_err_m.reserve(static_cast<std::size_t>(params.max_iters));
      trace->rot_err_rad.reserve(static_cast<std::size_t>(params.max_iters));
    }

    for (int it = 0; it < params.max_iters; ++it) {
      Eigen::Vector3d p_c;
      Eigen::Matrix3d R_c;
      fkPose(fk, q, p_c, R_c);

      // position error
      const Eigen::Vector3d e_p = p_t - p_c;

      // orientation error: R_err = R_t * R_c^T
      const Eigen::Matrix3d R_err = R_t * R_c.transpose();
      const Eigen::Vector3d e_R = rotMatToRotVec(R_err);

      const double pos_err = e_p.norm();
      const double rot_err = e_R.norm();

      if (trace) {
        trace->pos_err_m.push_back(pos_err);
        trace->rot_err_rad.push_back(rot_err);
      }

      res.iterations = it;
      res.pos_err = pos_err;
      res.rot_err = rot_err;

      if (pos_err < params.pos_tol && rot_err < params.rot_tol) {
        res.success = true;
        res.q = q;
        return res;
      }

      // build error vector
      Eigen::Matrix<double, 6, 1> e;
      e.block<3, 1>(0, 0) = e_p;
      e.block<3, 1>(3, 0) = e_R;

      // numeric jacobian
      Eigen::Matrix<double, 6, Eigen::Dynamic> J = numericJacobian(fk, q, params.eps);

      // DLS: dq = J^T (J J^T + lambda^2 I)^-1 e
      Eigen::Matrix<double, 6, 6> JJt = J * J.transpose();
      Eigen::Matrix<double, 6, 6> A =
          JJt + (params.lambda * params.lambda) * Eigen::Matrix<double, 6, 6>::Identity();

      Eigen::Matrix<double, 6, 1> x = A.ldlt().solve(e);
      Eigen::VectorXd dq = J.transpose() * x;  // n x 1

      // update
      for (int i = 0; i < n; ++i) {
        q[i] += params.alpha * dq[i];
      }
    }

    res.success = false;
    res.q = q;
    return res;
  }

}  // namespace arm_apps

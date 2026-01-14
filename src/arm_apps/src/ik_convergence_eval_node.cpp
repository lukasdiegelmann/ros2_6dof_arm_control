#include <Eigen/Geometry>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <geometry_msgs/msg/pose.hpp>
#include <iomanip>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "arm_apps/ik_dls.hpp"
#include "arm_apps/urdf_chain_fk.hpp"

namespace {

  static Eigen::Quaterniond quatFromRpy(double roll, double pitch, double yaw) {
    const Eigen::AngleAxisd Rx(roll, Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd Ry(pitch, Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd Rz(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = Rz * Ry * Rx;
    q.normalize();
    return q;
  }

  static std::vector<double> getDoubleArray(rclcpp::Node& node, const std::string& name,
                                            std::size_t expected_size) {
    std::vector<double> v = node.get_parameter(name).as_double_array();
    if (v.size() != expected_size) {
      throw std::runtime_error("Parameter '" + name + "' must have size " +
                               std::to_string(expected_size) + ", got " + std::to_string(v.size()));
    }
    return v;
  }

  static void ensureParentDirExists(const std::string& path) {
    const std::filesystem::path p(path);
    if (p.has_parent_path()) {
      std::filesystem::create_directories(p.parent_path());
    }
  }

  static void writeMetaHeader(std::ofstream& out, const arm_apps::IkParams& params,
                              const arm_apps::IkResult& result) {
    out << "# ik.max_iters=" << params.max_iters << "\n";
    out << "# ik.pos_tol=" << params.pos_tol << "\n";
    out << "# ik.rot_tol=" << params.rot_tol << "\n";
    out << "# ik.lambda=" << params.lambda << "\n";
    out << "# ik.alpha=" << params.alpha << "\n";
    out << "# ik.eps=" << params.eps << "\n";
    out << "# success=" << (result.success ? "true" : "false")
        << ", iterations=" << result.iterations << ", final_pos_err_m=" << result.pos_err
        << ", final_rot_err_rad=" << result.rot_err << "\n";
  }

  static std::string timestampNow() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm;
    localtime_r(&t, &tm);

    std::ostringstream ss;
    ss << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return ss.str();
  }

  static std::filesystem::path outputDirectoryFromParam(const std::string& output_path) {
    const std::filesystem::path p(output_path);

    // Backwards compatible interpretation:
    // - if a .csv path is provided, write outputs into its parent directory
    // - otherwise treat it as a directory
    if (p.extension() == ".csv") {
      return p.has_parent_path() ? p.parent_path() : std::filesystem::path(".");
    }
    return p.empty() ? std::filesystem::path(".") : p;
  }

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("ik_convergence_eval");

  node->declare_parameter<std::string>("robot_description", "");
  node->declare_parameter<std::string>("base_link", "base_link");
  node->declare_parameter<std::string>("tip_link", "ee_link");

  node->declare_parameter<std::vector<double>>("q_init", {0.0, -1.57, 1.57, 0.0, 1.57, 0.0});
  node->declare_parameter<std::vector<double>>("target_position", {0.4, 0.0, 0.4});
  node->declare_parameter<std::vector<double>>("target_rpy", {0.0, 1.57, 0.0});

  // If this parameter is a directory, timestamped outputs are written into it.
  // If it is a *.csv path (legacy), timestamped outputs are written into its parent directory.
  node->declare_parameter<std::string>("output_csv", "docs/evaluation");
  node->declare_parameter<bool>("overwrite", true);

  node->declare_parameter<int>("ik.max_iters", 200);
  node->declare_parameter<double>("ik.pos_tol", 1e-3);
  node->declare_parameter<double>("ik.rot_tol", 1e-2);
  node->declare_parameter<double>("ik.lambda", 0.05);
  node->declare_parameter<double>("ik.alpha", 0.5);
  node->declare_parameter<double>("ik.eps", 1e-5);

  try {
    const std::string robot_description = node->get_parameter("robot_description").as_string();
    if (robot_description.empty()) {
      throw std::runtime_error(
          "Parameter 'robot_description' is empty. Run via the provided launch file so xacro is "
          "expanded and passed in.");
    }

    const std::string base_link = node->get_parameter("base_link").as_string();
    const std::string tip_link = node->get_parameter("tip_link").as_string();

    arm_apps::UrdfChainFK fk;
    fk.initFromUrdf(robot_description, base_link, tip_link);

    const std::vector<double> q_init = getDoubleArray(*node, "q_init", fk.dof());
    const std::vector<double> target_position = getDoubleArray(*node, "target_position", 3);
    const std::vector<double> target_rpy = getDoubleArray(*node, "target_rpy", 3);

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = target_position[0];
    target_pose.position.y = target_position[1];
    target_pose.position.z = target_position[2];

    const Eigen::Quaterniond q = quatFromRpy(target_rpy[0], target_rpy[1], target_rpy[2]);
    target_pose.orientation.x = q.x();
    target_pose.orientation.y = q.y();
    target_pose.orientation.z = q.z();
    target_pose.orientation.w = q.w();

    arm_apps::IkParams params;
    params.max_iters = node->get_parameter("ik.max_iters").as_int();
    params.pos_tol = node->get_parameter("ik.pos_tol").as_double();
    params.rot_tol = node->get_parameter("ik.rot_tol").as_double();
    params.lambda = node->get_parameter("ik.lambda").as_double();
    params.alpha = node->get_parameter("ik.alpha").as_double();
    params.eps = node->get_parameter("ik.eps").as_double();

    arm_apps::IkTrace trace;
    const arm_apps::IkResult result = arm_apps::solveIkDls(fk, target_pose, q_init, params, &trace);

    const std::string output_path = node->get_parameter("output_csv").as_string();
    const std::filesystem::path out_dir = outputDirectoryFromParam(output_path);
    const std::string ts = timestampNow();
    const std::filesystem::path output_csv = out_dir / (ts + "_ik_dls_position_convergence.csv");
    const std::filesystem::path output_rot_csv =
        out_dir / (ts + "_ik_dls_rotation_convergence.csv");
    const bool overwrite = node->get_parameter("overwrite").as_bool();

    if (!overwrite && std::filesystem::exists(output_csv)) {
      throw std::runtime_error("Refusing to overwrite existing file: " + output_csv.string());
    }

    if (!overwrite && std::filesystem::exists(output_rot_csv)) {
      throw std::runtime_error("Refusing to overwrite existing file: " + output_rot_csv.string());
    }

    ensureParentDirExists(output_csv.string());
    ensureParentDirExists(output_rot_csv.string());

    std::ofstream out(output_csv, std::ios::out | std::ios::trunc);
    if (!out) {
      throw std::runtime_error("Failed to open output CSV for writing: " + output_csv.string());
    }

    writeMetaHeader(out, params, result);

    out << "iteration,pos_err_m,rot_err_rad\n";
    const std::size_t n = std::min(trace.pos_err_m.size(), trace.rot_err_rad.size());
    for (std::size_t i = 0; i < n; ++i) {
      out << i << "," << trace.pos_err_m[i] << "," << trace.rot_err_rad[i] << "\n";
    }

    out.close();

    // Rotation-only CSV (iteration,rot_err_rad)
    {
      std::ofstream out_rot(output_rot_csv, std::ios::out | std::ios::trunc);
      if (!out_rot) {
        throw std::runtime_error("Failed to open rotation CSV for writing: " +
                                 output_rot_csv.string());
      }

      writeMetaHeader(out_rot, params, result);
      out_rot << "iteration,rot_err_rad\n";
      for (std::size_t i = 0; i < n; ++i) {
        out_rot << i << "," << trace.rot_err_rad[i] << "\n";
      }
    }

    RCLCPP_INFO(node->get_logger(),
                "IK convergence CSV written to '%s' (success=%s, iters=%d, final pos_err=%.6g m, "
                "final rot_err=%.6g rad)",
                output_csv.string().c_str(), result.success ? "true" : "false", result.iterations,
                result.pos_err, result.rot_err);

    RCLCPP_INFO(node->get_logger(), "IK rotation convergence CSV written to '%s'",
                output_rot_csv.string().c_str());

    rclcpp::shutdown();
    return result.success ? 0 : 2;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "IK convergence eval failed: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
}

cmake_minimum_required(VERSION 3.8)
project(arm_apps)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdf REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GoToPose.action"
  DEPENDENCIES geometry_msgs action_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# instructions for build the go_to_pose_node executable
add_executable(go_to_pose_node
  src/go_to_pose_node.cpp
  src/urdf_chain_fk.cpp
  src/joint_limits.cpp
  src/joint_mapping.cpp
  src/ik_dls.cpp
  src/joint_trajectory.cpp
)

# for building the go_to_pose_node executable, also search for header files
# in the include directory
target_include_directories(go_to_pose_node PUBLIC include)

# Eigen3::Eigen is not linked automatically, for building go_to_pose_node
target_link_libraries(go_to_pose_node Eigen3::Eigen) 
target_link_libraries(go_to_pose_node "${cpp_typesupport_target}")

# tell the build system which packages to link for go_to_pose_node
ament_target_dependencies(go_to_pose_node
  rclcpp
  rclcpp_action
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  action_msgs
  kdl_parser
  orocos_kdl
  urdf
  eigen3_cmake_module
)

# instructions for building the go_to_pose executable
add_executable(go_to_pose
  src/go_to_pose_cli.cpp
)

# tell the build system which packages to link for go_to_pose
ament_target_dependencies(go_to_pose
  rclcpp
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

# demo_sequence_node executable (YAML targets -> Action goals)
add_executable(demo_sequence_node
  src/demo_sequence_node.cpp
)

target_include_directories(demo_sequence_node PUBLIC include)
target_link_libraries(demo_sequence_node yaml-cpp)
target_link_libraries(demo_sequence_node "${cpp_typesupport_target}")

ament_target_dependencies(demo_sequence_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  rclcpp_action
  ament_index_cpp
  action_msgs
  controller_manager_msgs
)

# draw_circle_cartesian_node executable (Cartesian circle -> Action goals)
add_executable(draw_circle_cartesian_node
  src/draw_circle_cartesian_node.cpp
)

target_include_directories(draw_circle_cartesian_node PUBLIC include)
target_link_libraries(draw_circle_cartesian_node "${cpp_typesupport_target}")

ament_target_dependencies(draw_circle_cartesian_node
  rclcpp
  rclcpp_action
  sensor_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  action_msgs
)

# joint_trajectory_viz_node executable (JointState -> RViz MarkerArray)
add_executable(joint_trajectory_viz_node
  src/joint_trajectory_viz_node.cpp
)

ament_target_dependencies(joint_trajectory_viz_node
  rclcpp
  sensor_msgs
  trajectory_msgs
  visualization_msgs
  geometry_msgs
  std_srvs
)

# ik_convergence_eval executable (offline IK convergence -> CSV)
add_executable(ik_convergence_eval
  src/ik_convergence_eval_node.cpp
  src/urdf_chain_fk.cpp
  src/ik_dls.cpp
)

target_include_directories(ik_convergence_eval PUBLIC include)
target_link_libraries(ik_convergence_eval Eigen3::Eigen)

ament_target_dependencies(ik_convergence_eval
  rclcpp
  geometry_msgs
  kdl_parser
  orocos_kdl
  urdf
  eigen3_cmake_module
)

# install the go_to_pose_node and go_to_pose executables
install(TARGETS
  go_to_pose_node
  go_to_pose
  demo_sequence_node
  draw_circle_cartesian_node
  joint_trajectory_viz_node
  ik_convergence_eval
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  action
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

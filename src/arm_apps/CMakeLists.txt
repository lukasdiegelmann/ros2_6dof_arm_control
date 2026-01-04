cmake_minimum_required(VERSION 3.8)
project(arm_apps)

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
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdf REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# instructions for build the go_to_pose_node executable
add_executable(go_to_pose_node
  src/go_to_pose_node.cpp
  src/urdf_chain_fk.cpp
  src/joint_limits.cpp
  src/joint_mapping.cpp
  src/ik_dls.cpp
)

# for building the go_to_pose_node executable, also search for header files
# in the include directory
target_include_directories(go_to_pose_node PUBLIC include)

# Eigen3::Eigen is not linked automatically, for building go_to_pose_node
target_link_libraries(go_to_pose_node Eigen3::Eigen) 

# tell the build system which packages to link for go_to_pose_node
ament_target_dependencies(go_to_pose_node
  rclcpp
  sensor_msgs
  geometry_msgs
  trajectory_msgs
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

# install the go_to_pose_node and go_to_pose executables
install(TARGETS
  go_to_pose_node
  go_to_pose
  DESTINATION lib/${PROJECT_NAME}
)

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

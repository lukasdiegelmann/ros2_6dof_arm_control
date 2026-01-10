# ros2_6dof_arm_control

ROS2-based 6-DOF robotic arm control stack with inverse kinematics, trajectory planning,
joint limits, and RViz/Gazebo visualization.

![Pick and Place Demo](media/pick_and_place.gif)

> Planned first release tag: **v0.1-mvp** (repo is prepared for release; no git tag created here).

## Key Features

- Inverse Kinematics (Damped Least Squares)
- Joint limits & velocity constraints
- Time-parameterized trajectories
- Pick & Place demo + Cartesian circle demo
- RViz visualization (TCP trail, joint trajectories)
- Gazebo simulation via ros2_control

## Tech Stack

- ROS2 (Humble / Jazzy)
- C++ (rclcpp) + Python (rclpy)
- Gazebo (gz sim)
- ros2_control + ros2_controllers
- RViz2

## Installation (copy-paste ready)

### System requirements

- Linux (tested on Ubuntu 22.04/24.04)
- ROS2 **Humble** (Ubuntu 22.04) or **Jazzy** (Ubuntu 24.04)

### Build & setup

```bash
# 1) Source your ROS distro
source /opt/ros/$ROS_DISTRO/setup.bash

# 2) System tools
sudo apt update
sudo apt install -y \
	python3-rosdep \
	python3-colcon-common-extensions \
	python3-vcstool

# 3) rosdep (first time on your machine)
sudo rosdep init || true
rosdep update

# 4) Install package dependencies (from package.xml)
cd /path/to/ros2_6dof_arm_control
rosdep install --from-paths src --ignore-src -r -y

# 5) Build
colcon build --symlink-install

# 6) Source overlay
source install/setup.bash
```

## Running the Demos

All demo launch files live in `arm_bringup`.

### Pick & Place

Starts Gazebo sim, spawns the UR5, ensures controllers are active, runs `go_to_pose_node`,
and executes a demo target sequence.

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash

ros2 launch arm_bringup pick_and_place.demo.launch.py
```

### Cartesian circle (draw_circle_cartesian)

Runs a cartesian circle by generating TCP waypoints and solving IK for each point.

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash

ros2 launch arm_bringup circle_cartesian.demo.launch.py
```

Useful args (optional):

```bash
ros2 launch arm_bringup circle_cartesian.demo.launch.py \
	radius:=0.18 plane:=xy loops:=2 num_points:=80 point_duration:=0.30
```

### Joint trajectory visualization

Publishes RViz `visualization_msgs/Marker` traces for joint trajectories.

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash

ros2 launch arm_bringup joint_traj_viz.launch.py
```

### RViz (optional)

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash

ros2 launch arm_bringup rviz.launch.py
```

### One-shot pose command (CLI)

When `go_to_pose_node` is running (e.g., via Pick&Place or Circle demo), send a target pose:

```bash
ros2 run arm_apps go_to_pose --x 0.45 --y 0.15 --z 0.25 --roll 0 --pitch 0 --yaw 0
```

## Architecture Overview

### Node graph (high level)

```text
 Gazebo (gz sim)
	 │
	 ├── ros_gz_bridge  ───────────────►  /clock
	 │
	 └── gz_ros2_control
					│
					├── controller_manager
					│     ├── joint_state_broadcaster      ─► /joint_states
					│     └── joint_trajectory_controller  ◄─ /joint_trajectory_controller/joint_trajectory
					│
					└── arm_bringup/ensure_controllers_active (idempotent activation)

 arm_apps/go_to_pose_node
	 ├── IK solver (Damped Least Squares)
	 ├── joint limits + velocity constraints
	 └── time-parameterized JointTrajectory  ───────► /joint_trajectory_controller/joint_trajectory

 arm_apps/joint_trajectory_viz_node  ─────────────► RViz markers (trails/plots)
 robot_state_publisher               ─────────────► /tf, /tf_static
```

### Data flow (what happens when you command a pose)

- `go_to_pose_node` receives a target end-effector pose (CLI or demo sequence).
- The DLS IK computes a joint configuration that matches the pose (within limits).
- The planner time-parameterizes a smooth joint trajectory subject to constraints.
- The trajectory is published to `joint_trajectory_controller`.
- ros2_control executes the trajectory in Gazebo; `joint_state_broadcaster` publishes feedback.
- `joint_trajectory_viz_node` (optional) turns joint data into RViz markers for inspection.

## Results

- Typical IK behavior: converges quickly for reachable targets; fails gracefully for unreachable poses.
- Trajectory quality: time-parameterized joint motion with smooth transitions and bounded velocities.
- Execution: deterministic controller-driven motion in simulation (plus built-in stall/lag diagnosis scripts).

## Roadmap

- MoveIt2 integration (planning + scene management)
- Impedance / admittance control
- Collision-aware planning
- Real hardware support (bringup + calibration)

## CV / Portfolio bullets

- Developed a ROS2-based 6-DOF robotic arm control stack with IK, joint-limit enforcement, and trajectory planning, demonstrated via Pick & Place and Cartesian motion demos.
- Built a modular ROS2 architecture with RViz and Gazebo integration, enabling reproducible simulation, visualization, and evaluation of motion quality.


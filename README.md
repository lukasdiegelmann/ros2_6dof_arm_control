# ros2_6dof_arm_control

<p align="center">
  <img src="media/pick_and_place.gif" width="420" alt="Simulated 6-DOF arm executing a simple sequence">
</p>

<p align="center">
  <b>Actuated 6‑DOF serial mechanism control (simulation)</b><br>
  Kinematic control → joint‑space trajectories → ros2_control actuator abstraction (Gazebo)
</p>

<p align="center">
  <img src="https://img.shields.io/badge/ROS2-Jazzy-blue">
  <img src="https://img.shields.io/badge/Language-C%2B%2B%20%7C%20Python-informational">
</p>

---

## What this project does

This repository models and controls a UR5‑style 6‑DOF serial arm as an **actuated multi‑joint mechanism**.
The core engineering focus is the chain from a geometric mechanism description and joint constraints to
repeatable joint‑level command execution through a controller/hardware abstraction.

Concretely:

- Models a 6‑DOF revolute chain with geometry, joint axes, limits, and simplified inertials in Xacro/URDF.
- Uses `ros2_control` as an **actuator interface boundary** (position command + state feedback) and runs it in Gazebo via `gz_ros2_control`.
- Executes joint‑space trajectories through `joint_state_broadcaster` and `joint_trajectory_controller`.
- Implements a Damped Least Squares IK solver (numeric Jacobian) and a small trajectory generator to translate Cartesian targets into joint trajectories.
- Applies joint‑level constraints (limits and simple velocity constraints) to keep commanded motion physically plausible for a position‑controlled mechanism.
- Includes a quantitative convergence measurement (IK iteration error vs tolerance) to validate kinematic control behavior on a fixed target.

ROS 2 is treated as the integration layer (transport, lifecycle, launch, tooling); the “system” is the modeled mechanism and the joint‑level control interface.

## What this project explicitly does NOT do

This scope is intentional:

- No real hardware execution: no drivers, calibration, safety interlocks, or hardware interfaces.
- No motor electrical model: no current/voltage dynamics, PWM, back‑EMF, thermal limits.
- No dynamics identification / high‑fidelity dynamics: no parameter identification, friction identification, or validation against measured data.
- No torque/impedance control: trajectory execution is position‑interface oriented.
- No collision checking or planning stack: no MoveIt2 planning pipeline.
- No perception: no cameras/LiDAR, object detection, or grasp synthesis.
- No task autonomy framework: no behavior trees/state machines/scene management.

---

## Modeling assumptions

- The inertial parameters are **simplified** and selected for plausibility and simulation stability.
- The model is suitable for early mechanism/control interface evaluation (kinematics, limits, controller wiring), not identification‑grade dynamics.
- Reported performance is limited to the simulated mechanism and the chosen controller configuration.

---

## Packages

- `arm_description`: Xacro/URDF + meshes for the UR5‑style arm (includes sim‑oriented `ros2_control` wiring).
- `arm_control`: controller configuration (`joint_state_broadcaster`, `joint_trajectory_controller`).
- `arm_bringup`: launch files/utilities to start simulation, spawn the robot, and start RViz.
- `arm_apps`: C++ nodes for IK, trajectory generation, demo sequencing, and visualization.

---

## Quantitative evaluation (IK convergence)

This repository includes one reproducible quantitative artifact measuring **Damped Least Squares IK convergence**.
For a fixed initial joint configuration and fixed Cartesian target pose, the solver logs the per‑iteration:

- end‑effector position error norm (m)
- end‑effector rotation error norm (rad)

Why it matters: convergence behavior is a practical check that the mechanism’s kinematic chain, joint ordering, and
joint‑space update law are wired correctly and behave sensibly under a position‑controlled actuation abstraction.

Generate the raw data (CSV):

    source /opt/ros/$ROS_DISTRO/setup.bash
    colcon build --symlink-install
    source install/setup.bash

    # Writes timestamped CSVs under docs/evaluation/
    ros2 launch arm_bringup ik_convergence_eval.launch.py

Generate the plots (PNG):

    # Reads the latest timestamped CSVs from docs/evaluation/ and writes matching PNGs
    python3 scripts/plot_ik_convergence.py

Notes:

- The plots require matplotlib (Ubuntu: `sudo apt install -y python3-matplotlib`).
- Pandas is optional; the script falls back to the stdlib CSV reader if pandas is not installed.

Example output plots (committed sample run):

<p float="left">
  <img src="docs/evaluation/20260114_193440_ik_dls_position_convergence.png" width="48%" />
  <img src="docs/evaluation/20260114_193440_ik_dls_rotation_convergence.png" width="48%" />
</p>

Concrete result (sample run above): converged in 10 iterations to 0.000944 m position error (tol 0.001 m) and 0.00863 rad rotation error (tol 0.01 rad).
This demonstrates convergence for one fixed target pose; it is not a global guarantee over all poses/configurations.

---

## Build

Requirements:

- Linux (tested on Ubuntu 24.04)
- ROS 2 Jazzy (Ubuntu 24.04)

Build:

    source /opt/ros/$ROS_DISTRO/setup.bash
    rosdep install --from-paths src --ignore-src -r -y
    colcon build --symlink-install
    source install/setup.bash

---

## Demos

All demo launch files live in `arm_bringup`.

### Pick & Place

<table>
<tr>
<td width="50%" valign="top">
<pre><code>source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash

ros2 launch arm_bringup pick_and_place.demo.launch.py
</code></pre>
</td>
<td width="50%" align="center" valign="top">
<img src="media/pick_and_place.gif" width="400" alt="Pick & Place Demo">
</td>
</tr>
</table>

### Cartesian circle (draw_circle_cartesian)

<table>
<tr>
<td width="50%" valign="top">
<pre><code>source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash

ros2 launch arm_bringup circle_cartesian.demo.launch.py
</code></pre>
<strong>Optional arguments:</strong>
<pre><code>ros2 launch arm_bringup circle_cartesian.demo.launch.py \
  radius:=0.18 plane:=xy loops:=2 \
  num_points:=80 point_duration:=0.30
</code></pre>
</td>
<td width="50%" align="center" valign="top">
<img src="media/circle_cartesian.gif" width="400" alt="Cartesian Circle Demo">
</td>
</tr>
</table>

### Joint trajectory visualization

<table>
<tr>
<td width="50%" valign="top">
<pre><code>source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash

ros2 launch arm_bringup joint_traj_viz.launch.py
</code></pre>
</td>
<td width="50%" align="center" valign="top">
<img src="media/trajectory_plotting.gif" width="400" alt="Trajectory Plotting">
</td>
</tr>
</table>

---

## Architecture overview (high level)

    Gazebo (gz sim)
      │
      ├── ros_gz_bridge  ───────────────►  /clock
      │
      └── gz_ros2_control
             │
             ├── controller_manager
             │     ├── joint_state_broadcaster
             │     └── joint_trajectory_controller
             │
             └── ensure_controllers_active

    go_to_pose_node
      ├── IK (Damped Least Squares)
      ├── Joint limits & velocity constraints
      └── JointTrajectory → controller

    joint_trajectory_viz_node → RViz markers
    robot_state_publisher → /tf

---

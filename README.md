# ROS2 6-DoF Arm Control

This repository implements a ROS2-based control pipeline for a simulated 6-degree-of-freedom robotic arm, including kinematic modeling, inverse kinematics, and joint trajectory execution in Gazebo.

---

## Demo
🚧 *Demo video coming soon*  
(A short video/GIF demonstrating end-effector pose control and trajectory execution in simulation.)

---

## How to Run
🚧 *Instructions will be added once the basic simulation and control pipeline is finalized.*

## Week 3: CLI Pose Command

When `go_to_pose_node` is running, you can publish a one-shot target pose via:

```bash
ros2 run arm_apps go_to_pose --x 0.45 --y 0.15 --z 0.25 --roll 0 --pitch 0 --yaw 0
```

Use reachable poses (otherwise IK may fail).

---

## Roadmap
- [ ] Simulated 6-DoF robotic arm in Gazebo using ROS2 and ros2_control  
- [ ] Forward and inverse kinematics implementation (numerical IK)  
- [ ] Joint trajectory planning with velocity and acceleration limits  
- [ ] Cartesian end-effector pose control via ROS2 interfaces  
- [ ] Project documentation, demos, and reproducible setup  

---

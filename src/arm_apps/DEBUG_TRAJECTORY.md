# go_to_pose_node Debug Guide

## Problem: Trajectory wird nicht publiziert

### Diagnose: ist `/target_pose` subscription aktiv?

```bash
# Terminal 1: Node mit maximalen Debug-Logs
ros2 run arm_apps go_to_pose_node --ros-args -p debug_hardcoded_trajectory:=true 2>&1 | grep -E "(Constructor|subscription|CALLBACK|Published)"
```

Erwartete Logs:
```
[go_to_pose_node]: ===== GoToPoseNode Constructor STARTING =====
[go_to_pose_node]: Creating subscriptions...
[go_to_pose_node]: ✓ /joint_states subscription created
[go_to_pose_node]: ✓ /target_pose subscription created
[go_to_pose_node]: ✓ Trajectory publisher created
[go_to_pose_node]: ===== GoToPoseNode Constructor COMPLETE =====
```

Wenn diese Logs nicht erscheinen: robot_description ist nicht verfügbar!

---

### Schritt 1: Robot State Publisher + Node starten

**Wichtig:** robot_state_publisher MUSS VOR dem go_to_pose_node gestartet werden!

```bash
# Terminal 1: Robot State Publisher
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:="$(find arm_description)/share/arm_description/urdf/ur5.urdf"
```

Warte auf:
```
[robot_state_publisher]: Robot initialized
```

**Terminal 2: go_to_pose_node**
```bash
ros2 run arm_apps go_to_pose_node --ros-args \
  -p debug_hardcoded_trajectory:=true \
  -p trajectory_topic:=/joint_trajectory_controller/joint_trajectory
```

Erwartete Logs:
```
[go_to_pose_node]: ===== GoToPoseNode Constructor STARTING =====
[go_to_pose_node]: ✓ /target_pose subscription created
[go_to_pose_node]: ===== GoToPoseNode Constructor COMPLETE =====
```

---

### Schritt 2: Simulierte Joint States publizieren

Ohne Simulation musst du manuelle joint_states publishen:

```bash
# Terminal 3: Publish fake joint states
ros2 topic pub /joint_states sensor_msgs/msg/JointState '{header: {frame_id: base_link}, name: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint], position: [0, -1.57, 1.57, 0, 0, 0]}'
```

Prüfe Logs:
```
[go_to_pose_node]: JointState received: names=6 positions=6
[go_to_pose_node]: EE pos: [0.XXX 0.XXX 0.XXX]
```

---

### Schritt 3: Target Pose senden

```bash
# Terminal 4: Send target pose
ros2 topic pub --once /target_pose geometry_msgs/PoseStamped \
  '{header: {frame_id: base_link}, pose: {orientation: {w: 1.0}}}'
```

**Erwartete Logs:**
```
[go_to_pose_node]: ===== CALLBACK TRIGGERED: onTargetPose called! =====
[go_to_pose_node]: TargetPose received (frame_id=base_link, ...)
[go_to_pose_node]: have_js_=true, proceeding with trajectory generation...
[go_to_pose_node]: Publishing HARDCODED trajectory (debug mode)
[go_to_pose_node]: Publishing hardcoded trajectory to '/joint_trajectory_controller/joint_trajectory' with 6 joints
[go_to_pose_node]: Hardcoded trajectory published.
```

Parallel verifizieren:
```bash
ros2 topic echo /joint_trajectory_controller/joint_trajectory --once
```

Falls nichts ankommt: **onTargetPose wird nicht aufgerufen** → `/target_pose` subscription ist nicht aktiv.

---

## Troubleshooting

### Problem: Constructor meldet Fehler
```
[go_to_pose_node]: robot_description is empty
```

**Lösung:** robot_state_publisher startet nicht richtig
```bash
# Direkt robot_state_publisher starten BEFORE go_to_pose_node
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(find arm_description)/xacro/ur5.xacro"
```

### Problem: Subscription logs fehlen
```
[go_to_pose_node]: Creating subscriptions...
[gestartet, aber keine ✓ logs]
```

Das Programm hängt in `create_subscription` - das ist ungewöhnlich. Prüfe:
- ROS_DOMAIN_ID konsistent?
- Firewall blockiert?

```bash
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "ROS_LOCALHOST_ONLY: $ROS_LOCALHOST_ONLY"
```

### Problem: "Waiting for at least 1 matching subscription(s)..."

Das bedeutet, `/target_pose` hat keine aktive Subscribers.

**Prüfe:**
```bash
ros2 node list
ros2 node info /go_to_pose_node  # muss /target_pose zeigen!
```

Falls `/target_pose` nicht als Subscriber angezeigt wird: Node hat subscription nicht registriert.

---

## Parameter Übersicht

```bash
ros2 run arm_apps go_to_pose_node --ros-args \
  -p base_link:=base_link \
  -p tip_link:=ee_link \
  -p trajectory_topic:=/joint_trajectory_controller/joint_trajectory \
  -p debug_hardcoded_trajectory:=true
```

---

## Minimales Funktions-Test-Setup

```bash
# 1. Robot State Publisher (Terminal 1)
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:="$(find arm_description)/xacro/ur5.xacro"

# 2. go_to_pose_node (Terminal 2)
ros2 run arm_apps go_to_pose_node --ros-args -p debug_hardcoded_trajectory:=true

# 3. Joint States simulieren (Terminal 3)
while true; do
  ros2 topic pub /joint_states sensor_msgs/msg/JointState \
    '{header: {frame_id: base_link}, name: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint], position: [0, -1.57, 1.57, 0, 0, 0]}' \
    --rate 10
done

# 4. Trigger trajectory (Terminal 4)
ros2 topic pub --once /target_pose geometry_msgs/PoseStamped \
  '{header: {frame_id: base_link}, pose: {orientation: {w: 1.0}}}'

# 5. Verifizieren (Terminal 5)
ros2 topic echo /joint_trajectory_controller/joint_trajectory
```

Wenn alles funktioniert, siehst du in Terminal 5 eine JointTrajectory-Nachricht.


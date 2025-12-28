#!/usr/bin/env bash
set -e

ros2 run arm_apps go_to_pose --x 0.45 --y 0.00 --z 0.30 --roll 0 --pitch 0 --yaw 0
sleep 3

ros2 run arm_apps go_to_pose --x 0.45 --y -0.20 --z 0.30 --roll 0 --pitch 0 --yaw 0
sleep 3

ros2 run arm_apps go_to_pose --x 0.45 --y 0.20 --z 0.30 --roll 0 --pitch 0 --yaw 0
sleep 3

ros2 run arm_apps go_to_pose --x 0.40 --y 0.10 --z 0.45 --roll 0 --pitch 0 --yaw 0
sleep 3

ros2 run arm_apps go_to_pose --x 0.45 --y 0.00 --z 0.30 --roll 0 --pitch 0 --yaw 0

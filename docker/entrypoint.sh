#!/usr/bin/env bash
set -e

# Source ROS
if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

# Source workspace overlay (if built)
if [[ -f "/ws/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "/ws/install/setup.bash"
fi

exec "$@"

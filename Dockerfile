# syntax=docker/dockerfile:1

ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}

# Build + rosdep tooling
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-vcstool \
  && rm -rf /var/lib/apt/lists/*

# Initialize rosdep (safe to re-run)
RUN rosdep init || true && rosdep update

WORKDIR /ws
COPY . /ws

# Install ROS dependencies and build
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
  && apt-get update \
  && rosdep install --from-paths src --ignore-src -r -y \
  && rm -rf /var/lib/apt/lists/* \
  && colcon build --symlink-install

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

#!/usr/bin/env bash

set -x

source /opt/ros/$ROS2_DISTRO/setup.bash
unset ROS_DISTRO
source /opt/ros/$ROS1_DISTRO/setup.bash
echo "Building 'ros1_bridge'..."
colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure

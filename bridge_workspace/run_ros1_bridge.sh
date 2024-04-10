#!/usr/bin/env bash

set -eo pipefail


{
source /opt/ros/$ROS2_DISTRO/setup.bash > /dev/null 2>&1
source $BRIDGE_WORKSPACE/install/setup.bash > /dev/null 2>&1
source /opt/ros/$ROS1_DISTRO/setup.bash > /dev/null 2>&1
source $ROS1_WORKSPACE/devel/setup.bash > /dev/null 2>&1 

ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
}

#!/usr/bin/env bash

set -eu

echo "Staring ros translator heartbeat"
source /opt/ros/noetic/setup.bash
cd ${ROS1_WORKSPACE}/ros1_ws
catkin_make
source devel/setup.bash

rostopic pub /heartbeat adore_if_ros_msg/Action "action: 42" -r 10


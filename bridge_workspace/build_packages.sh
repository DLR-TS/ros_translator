#!/usr/bin/env bash

#set -euo pipefail

AMENT_TRACE_SETUP_FILES=
AMENT_PYTHON_EXECUTABLE=
AMENT_PREFIX_PATH=
PYTHONPATH=
LD_LIBRARY_PATH=


source /opt/ros/$ROS2_DISTRO/setup.bash
rm -rf build install

colcon build --symlink-install --packages-skip ros1_bridge

#!/usr/bin/env #!/usr/bin/env bash

set -eo pipefail

bash patch_ros1_bridge.sh

{
source /opt/ros/$ROS2_DISTRO/setup.bash > /dev/null 2>&1
source /opt/ros/$ROS1_DISTRO/setup.bash > /dev/null 2>&1
source $ROS1_WORKSPACE/devel/setup.bash > /dev/null 2>&1 

echo "Building 'ros1_bridge'..."
colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure
}


#!/usr/bin/envbash

set -euo pipefail



EXCLUDE_PACKAGES="
action_msgs
actionlib_msgs
ament_cmake
ament_cmake_auto
ament_cmake_copyright
ament_cmake_core
ament_cmake_cppcheck
ament_cmake_cpplint
ament_cmake_export_definitions
ament_cmake_export_dependencies
ament_cmake_export_include_directories
ament_cmake_export_interfaces
ament_cmake_export_libraries
ament_cmake_export_link_flags
ament_cmake_export_targets
ament_cmake_flake8
ament_cmake_gmock
ament_cmake_gtest
ament_cmake_include_directories
ament_cmake_libraries
ament_cmake_lint_cmake
ament_cmake_pep257
ament_cmake_pytest
ament_cmake_python
ament_cmake_ros
ament_cmake_target_dependencies
ament_cmake_test
ament_cmake_uncrustify
ament_cmake_version
ament_cmake_xmllint
ament_copyright
ament_cppcheck
ament_cpplint
ament_flake8
ament_index_cpp
ament_index_python
ament_lint
ament_lint_auto
ament_lint_cmake
ament_lint_common
ament_package
ament_pep257
ament_uncrustify
ament_xmllint
builtin_interfaces
class_loader
common_interfaces
composition_interfaces
console_bridge_vendor
demo_nodes_cpp
demo_nodes_py
diagnostic_msgs
domain_coordinator
eigen3_cmake_module
example_interfaces
fastrtps_cmake_module
gazebo_msgs
geometry2
geometry_msgs
kdl_parser
launch
launch_ros
launch_testing
launch_testing_ament_cmake
launch_testing_ros
launch_xml
launch_yaml
libstatistics_collector
libyaml_vendor
lifecycle_msgs
message_filters
nav_msgs
osrf_pycommon
pluginlib
python_cmake_module
rcl
rcl_action
rcl_interfaces
rcl_lifecycle
rcl_logging_spdlog
rcl_yaml_param_parser
rclcpp
rclcpp_action
rclcpp_components
rclcpp_lifecycle
rclpy
rcpputils
rcutils
rmw
rmw_dds_common
rmw_fastrtps_cpp
rmw_fastrtps_shared_cpp
rmw_implementation
rmw_implementation_cmake
robot_state_publisher
ros1_bridge
ros2action
ros2bag
ros2cli
ros2component
ros2doctor
ros2interface
ros2launch
ros2lifecycle
ros2multicast
ros2node
ros2param
ros2pkg
ros2run
ros2service
ros2topic
ros_base
ros_core
ros_environment
ros_workspace
rosbag2
rosbag2_compression
rosbag2_converter_default_plugins
rosbag2_cpp
rosbag2_storage
rosbag2_storage_default_plugins
rosbag2_transport
rosgraph_msgs
rosidl_adapter
rosidl_cmake
rosidl_default_generators
rosidl_default_runtime
rosidl_generator_c
rosidl_generator_cpp
rosidl_generator_py
rosidl_parser
rosidl_runtime_c
rosidl_runtime_cpp
rosidl_runtime_py
rosidl_typesupport_c
rosidl_typesupport_cpp
rosidl_typesupport_fastrtps_c
rosidl_typesupport_fastrtps_cpp
rosidl_typesupport_interface
rosidl_typesupport_introspection_c
rosidl_typesupport_introspection_cpp
rpyutils
sensor_msgs
shape_msgs
shared_queues_vendor
spdlog_vendor
sqlite3_vendor
sros2
sros2_cmake
statistics_msgs
std_msgs
std_srvs
stereo_msgs
tf2
tf2_bullet
tf2_eigen
tf2_geometry_msgs
tf2_kdl
tf2_msgs
tf2_py
tf2_ros
tf2_sensor_msgs
tf2_tools
tinyxml2_vendor
tinyxml_vendor
tracetools
trajectory_msgs
uncrustify_vendor
unique_identifier_msgs
urdf
visualization_msgs
yaml_cpp_vendor
zstd_vendor
actionlib
catkin
class_loader
cpp_common
diagnostic_msgs
gazebo_msgs
gencpp
geneus
genlisp
genmsg
gennodejs
genpy
geometry_msgs
message_filters
message_generation
message_runtime
mk
nav_msgs
pluginlib
ros_environment
rosbag
rosbag_migration_rule
rosbag_storage
rosbash
rosboost_cfg
rosbuild
rosclean
rosconsole
roscpp
roscpp_serialization
roscpp_traits
roscpp_tutorials
roscreate
rosgraph
rosgraph_msgs
roslang
roslaunch
roslib
roslisp
roslz4
rosmake
rosmaster
rosmsg
rosnode
rosout
rospack
rosparam
rospy
rospy_tutorials
rosservice
rostest
rostime
rostopic
rosunit
roswtf
sensor_msgs
shape_msgs
std_msgs
std_srvs
stereo_msgs
tf
tf2
tf2_msgs
tf2_py
tf2_ros
topic_tools
trajectory_msgs
visualization_msgs
xmlrpcpp
"


ros2_pkgs="$(ros2 pkg list)"
ros1_pkgs="$(rospack list | cut -d " " -f1)"

readarray -t exclude_array <<<"$EXCLUDE_PACKAGES"

echo "ROS 1 Packages: "
for pkg in $ros1_pkgs; do
    skip_pkg=false
    for excluded_pkg in "${exclude_array[@]}"; do
        if [[ "$pkg" == "$excluded_pkg" ]]; then
            skip_pkg=true
            break
        fi
    done

    if [ "$skip_pkg" == false ]; then
        interfaces="$(rosmsg list | grep $pkg | sed ':a;N;$!ba;s/\n/ /g')"
        echo "  Package: $pkg"
        echo "    Interfaces: $interfaces"
    fi
done
echo ""

echo "ROS 2 Packages: "
for pkg in $ros2_pkgs; do
    skip_pkg=false
    for excluded_pkg in "${exclude_array[@]}"; do
        if [[ "$pkg" == "$excluded_pkg" ]]; then
            skip_pkg=true
            break
        fi
    done

    if [ "$skip_pkg" == false ]; then
        interfaces="$(ros2 interface list | grep $pkg | sed ':a;N;$!ba;s/\n/ /g')"
        echo "  Package: ${pkg}"
        echo "    Interfaces: ${interfaces}"
    fi
done




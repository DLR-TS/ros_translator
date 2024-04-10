#!/usr/bin/env bash

set -e


export DEBIAN_FRONTEND=noninteractive

apt-get update
apt-get install -y sudo wget curl


rm -rf ~/ros_catkin_ws
mkdir /tmp/ros_noetic_build_ws

ROS_DISTRO=noetic

cd /tmp/ros_noetic_build_ws
sudo apt-get install -y python3-rosdep python3-rosinstall-generator python3-vcstools python3-vcstool build-essential
sudo rosdep init || true
rosdep update || true

rosinstall_generator desktop --rosdistro noetic --deps --tar > noetic-desktop.rosinstall
mkdir ./src
vcs import --input noetic-desktop.rosinstall ./src

sudo apt-get install -y libfltk1.3-dev

#hddtemp disable patch
sed -i -e s/"<run_depend>hddtemp<\/run_depend>"/"<\!-- <run_depend>hddtemp<\/run_depend> -->"/g ./src/diagnostics/diagnostic_common_diagnostics/package.xml

rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro noetic -y

sed -i -e s/"COMPILER_SUPPORTS_CXX11"/"COMPILER_SUPPORTS_CXX17"/g ./src/geometry/tf/CMakeLists.txt
sed -i -e s/"c++11"/"c++17"/g ./src/geometry/tf/CMakeLists.txt
sed -i -e s/"CMAKE_CXX_STANDARD 14"/"CMAKE_CXX_STANDARD 17"/g ./src/kdl_parser/kdl_parser/CMakeLists.txt
sed -i -e s/"CMAKE_CXX_STANDARD 11"/"CMAKE_CXX_STANDARD 17"/g ./src/laser_geometry/CMakeLists.txt
sed -i -e s/"c++11"/"c++17"/g ./src/resource_retriever/CMakeLists.txt
sed -i -e s/"COMPILER_SUPPORTS_CXX11"/"COMPILER_SUPPORTS_CXX17"/g ./src/robot_state_publisher/CMakeLists.txt
sed -i -e s/"c++11"/"c++17"/g ./src/robot_state_publisher/CMakeLists.txt
sed -i -e s/"c++11"/"c++17"/g ./src/rqt_image_view/CMakeLists.txt
sed -i -e s/"CMAKE_CXX_STANDARD 14"/"CMAKE_CXX_STANDARD 17"/g ./src/urdf/urdf/CMakeLists.txt

rm -rf ./src/rosconsole
cd src
git clone https://github.com/tatsuyai713/rosconsole
cd ..

./src/catkin/bin/catkin_make_isolated --install --install-space /opt/ros/noetic -DCMAKE_BUILD_TYPE=Release
rosinstall_generator desktop --rosdistro noetic --deps --tar > noetic-desktop.rosinstall
./src/catkin/bin/catkin_make_isolated --install

cd /opt/ros
ln -sf /tmp/ros_noetic_build_ws/devel noetic

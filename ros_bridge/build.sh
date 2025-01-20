#!/bin/bash

echo "Building messages for ROS 2..."
. /opt/ros/foxy/setup.bash
cd ros2_msgs_ws && colcon build

cd ..

echo "Building messages for ROS 1..."
. /opt/ros/noetic/setup.bash
cd ros1_msgs_ws && catkin_make_isolated --install

cd ..

echo "Setting ros1_bridge..."
cd bridge_ws/src
git clone -b foxy https://github.com/ros2/ros1_bridge.git

cd ..

echo "Building..."
. /opt/ros/noetic/setup.bash
. /opt/ros/foxy/setup.bash
. ../ros1_msgs_ws/install_isolated/setup.bash
. ../ros2_msgs_ws/install/local_setup.bash

colcon build --packages-select ros1_bridge --cmake-force-configure
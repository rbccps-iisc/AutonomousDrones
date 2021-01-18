#!/bin/bash

source ~/catkin_ws/devel/setup.bash
# source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/posix_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/src/Firmware/Tools/sitl_gazebo/
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/Simulation/models/
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/src/Firmware/build/px4_sitl_default/build_gazebo
roslaunch multidrone_mission multiple_vehicle_spawn_apm_demo_2.launch
pkill -9 python

#!/bin/bash
export PX4_HOME_LAT=13.027207
export PX4_HOME_LON=77.563642
export PX4_HOME_ALT=918
cd ~/src/Firmware
# source ~/python2_ws/devel/setup.bash
#make posix_sitl_default
#make posix_sitl_default sitl_gazebo
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/posix_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo/
export GAZEBO_PLUGIN_PATH=~/src/Firmware/build/px4_sitl_default/build_gazebo
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_proj/src/AutonomousDrones/px4_simulation/models/
# roslaunch px4 mavros_posix_sitl.launch vehicle:=iris_fpv_cam
# roslaunch px4 mavros_posix_sitl.launch vehicle:=typhoon_h480
# roslaunch px4 mavros_posix_sitl.launch vehicle:=iris_hitl fcu_url:=/dev/ttyACM0:921600
# roslaunch px4 mavros_posix_sitl.launch vehicle:=iris_extended
roslaunch multidrone_mission single_uav_mavros_sitl_sdf.launch
# roslaunch px4 mavros_posix_sitl.launch

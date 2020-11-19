#!/bin/bash

cd ..

DONT_RUN=1 make px4_sitl_default gazebo

#source ~/catkin_ws/devel/setup.bash    # (optional)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo

gnome-terminal -- roslaunch px4 posix_sitl.launch
sleep 5

gnome-terminal -- roslaunch mavros px4.launch fcu_url:="udp://:14540@192.168.1.36:14557"

sleep 5

gnome-terminal -- rosrun offb offb_node



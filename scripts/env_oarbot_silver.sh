#!/usr/bin/env bash

export ROS_WS=/home/oarbot_silver/catkin_ws
export ROS_KINETIC=/opt/ros/kinetic
source $ROS_KINETIC/setup.bash
source $ROS_WS/devel/setup.bash
export PATH=$ROS_ROOT/bin:$PATH
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$ROS_WS
export ROS_MASTER_URI=http://192.168.1.100:11311/
export ROS_IP=192.168.1.101
chmod +x /dev/ttyACM0
chmod +x /dev/ttyACM1
chmod +x /dev/ttyACM2
chmod +x /dev/ttyACM3

exec "$@"

# test line
# Another test line
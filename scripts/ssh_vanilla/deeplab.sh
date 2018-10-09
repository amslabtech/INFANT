#!/bin/bash

ssh -t -t vanilla <<EOF
export ROS_MASTER_URI="http://192.168.0.145:11311"

sleep 2s

/opt/ros/kinetic/bin/rosrun deeplab_ros deeplab_ros2.py &
sleep 5s



#!/bin/bash

ssh -t -t vanilla -X<<EOF
export ROS_MASTER_URI="http://192.168.0.145:11311"
sleep 2s

/opt/ros/kinetic/bin/rosrun depth2point resize_image &
sleep 5s


#!/bin/bash

ssh -t -t vanilla -X<<EOF
export ROS_MASTER_URI="http://192.168.0.145:11311"

sleep 2s

/home/amsl/scripts/republish.sh /camera/rgb/resized_image &
sleep 5s


#!/bin/bash

source /opt/ros/kinetic/setup.bash
source /home/amsl/ros_catkin_ws/devel/setup.bash
source ~/.bashrc
# export ROS_PACKAGE_PATH=/home/amsl/ros_catkin_ws:$ROS_PACKAGE_PATH

gnome-terminal -e "/opt/ros/kinetic/bin/roscore" --geometry=50x12+20+20 &
sleep 2s
#gnome-terminal -e "/home/amsl/ros_catkin_ws/scripts/ssh_vanilla/zed_start_vanilla.sh" --geometry=50x12+20+270 &
#gnome-terminal -e "/home/amsl/ros_catkin_ws/scripts/ssh_vanilla/zed_resize_vanilla.sh" --geometry=50x12+20+520 &
# gnome-terminal -e "/opt/ros/kinetic/bin/rosbag play /media/amsl/D8A80472A8045184/bagfiles/2018-09-15-14-08-49.bag" --geometry=50x12+20+270 &
#gnome-terminal -e "/opt/ros/kinetic/bin/rosbag play /home/amsl/bagfiles/2018-09-15-14-08-49.bag" --geometry=50x12+20+270 &
gnome-terminal -e "/opt/ros/kinetic/bin/rosbag play --clock /media/amsl/D8A80472A8045184/bagfiles/2018-10-13-13-23-54.bag -s 40" --geometry=50x12+20+270 &

gnome-terminal -e "/home/amsl/ros_catkin_ws/src/INFANT/scripts/ssh_vanilla/republish.sh" --geometry=50x12+20+770 &
gnome-terminal -e "/home/amsl/ros_catkin_ws/src/INFANT/scripts/ssh_vanilla/down_rate.sh" --geometry=50x12+520+20 &
gnome-terminal -e "/home/amsl/ros_catkin_ws/src/INFANT/scripts/ssh_vanilla/deeplab.sh" --geometry=50x12+520+270 &
gnome-terminal -e "/opt/ros/kinetic/bin/rqt_image_view /deeplab/image" --geometry=50x12+520+520 &
gnome-terminal -e "/opt/ros/kinetic/bin/rqt_image_view /camera/rgb/resized_image" --geometry=50x12+520+770 &

#gnome-terminal -e "/opt/ros/kinetic/bin/rosrun infant_recognition grass2grid" --geometry=50x12+1020+20

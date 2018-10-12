#!/bin/bash

source /opt/ros/kinetic/setup.bash
source /home/amsl/ros_catkin_ws/devel/setup.bash
source ~/.bashrc

gnome-terminal -e "/opt/ros/kinetic/bin/roscore" --geometry=50x12+20+20 &
sleep 3s

####rviz####
#gnome-terminal -e "/opt/ros/kinetic/bin/rosrun rviz rviz -d /home/amsl/ros_catkin_ws/src/INFANT/config/tsukuba2018.rviz" --geometry=50x12+20+250 

gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch infant_recognition occupancygrid_store.launch" --geometry=50x12+20+480 

gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch infant_planning local_path.launch" --geometry=50x12+20+710 

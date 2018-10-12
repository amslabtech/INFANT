#!/bin/bash

source /opt/ros/kinetic/setup.bash
source /home/amsl/ros_catkin_ws/devel/setup.bash


gnome-terminal -e "/opt/ros/kinetic/bin/roscore" --geometry=50x12+0+0 &
sleep 3s

####rviz########
gnome-terminal -e "/opt/ros/kinetic/bin/rosrun rviz rviz -d /home/amsl/ros_catkin_ws/src/INFANT/config/tsukuba2018.rviz" --geometry=1x1+0+0 &

###### tiny ###############
gnome-terminal -e "/opt/ros/kinetic/bin/rosrun knm_tiny_power knm_tiny_revival.py" --geometry=50x12+0+250 &

gnome-terminal -e "/opt/ros/kinetic/bin/rosrun knm_tiny_power mcnk_cheat_joy" --geometry=50x12+0+250 &

gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch joy ps3joy.launch" --geometry=50x12+0+250 &

###### localization ##########
gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch xsens_driver xsens_driver.launch" --geometry=50x12+0+500 &
sleep 2s
gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch velodyne_pointcloud 32e_points_pubtf.launch" --geometry=50x12+0+750 &
sleep 2s

###### recognition #######
gnome-terminal -e "/opt/ros/kinetic/bin/rosrun velodyne_height_map rm_ground_node_minmax.launch" --geometry=50x12+0+750 &
sleep 2s
gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch infant_recognition recognition.launch" --geometry=50x12+500+500 &
sleep 2s

###### path plan #######
gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch infant_planning local_path.launch" --geometry=50x12+500+500 &
gnome-terminal -e "/opt/ros/kinetic/bin/rosrun infant_planning motion_planner" --geometry=50x12+500+750 &
sleep 2s 

####check whether nodes are running######
gnome-terminal -e "/home/amsl/ros_catkin_ws/scripts/rwrc16_for_bag_check.sh" --geometry=50x44+1500+0
sleep 3s


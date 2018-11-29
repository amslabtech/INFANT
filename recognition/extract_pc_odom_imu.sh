#!/bin/bash

/home/amsl/ros_catkin_ws/src/INFANT/scripts/road_detec.sh
sleep 5s
gnome-terminal -e "/opt/ros/kinetic/bin/rosparam set use_sim_time true" 
gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch infant_recognition recognition_withzed.launch" 
gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch velodyne_pointcloud 32e_points.launch"
gnome-terminal -e "/opt/ros/kinetic/bin/rosrun velodyne_height_map rm_ground_node_minmax"
sleep 5s
gnome-terminal -e "/opt/ros/kinetic/bin/rosbag play --clock /home/amsl/bagfiles/2018-10-13-13-23-54.bag"
# gnome-terminal -e "/opt/ros/kinetic/bin/rosbag record /camera/rgb/resized_image/compressed /imu/data /tinypower/odom /velodyne_packets /zed_grasspoints /zed_roadpoints /deeplab/image /camera/depth/down_depth_image -O /home/amsl/Desktop/extract_pc_odom_imu.bag"
# gnome-terminal -e "/opt/ros/kinetic/bin/rosbag record /camera/rgb/resized_image/compressed /imu/data /tinypower/odom /velodyne_packets /zed_grasspoints /zed_roadpoints -O /home/amsl/Desktop/extract_pc_odom_imu.bag"
gnome-terminal -e "/opt/ros/kinetic/bin/rviz -d /home/amsl/.rviz/localmap_tsuku18.rviz"
gnome-terminal -e "/opt/ros/kinetic/bin/rviz -d /home/amsl/.rviz/localmap_lidar_tsuku18.rviz"
gnome-terminal -e "/opt/ros/kinetic/bin/rviz -d /home/amsl/.rviz/localmap_zed_tsuku18.rviz"

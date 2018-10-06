#include <ros/ros.h>

// #include "Eigen/Core"
// #include "Eigen/Geometry"

#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <iostream>
#include <vector>

#include <boost/thread.hpp>

using namespace std;
const string header_frame="/map";
const string sensor_frame="/velodyne";
//const string sensor_frame="/obstacle_local_map";
// const string sensor_frame="/velodyne2";

boost::mutex map_mutex, frame_mutex_;
nav_msgs::OccupancyGrid map_in;
bool callback_flag=false;

void mapLocal2Global(nav_msgs::OccupancyGrid& grid_map, const tf::StampedTransform& transform)
{
	float angle = tf::getYaw(transform.getRotation());
	float tmp_x = fabs(grid_map.info.origin.position.x);
	float tmp_y = fabs(grid_map.info.origin.position.y);
	
//	cout<<"tfx:"<<transform.getOrigin().x()<<"	tfy:"<<transform.getOrigin().y()<<endl;
//	cout<<"map x:"<<grid_map.info.origin.position.x<<"	map y:"<<grid_map.info.origin.position.y<<endl;

	grid_map.info.origin.position.x = transform.getOrigin().x()-cos(angle)*tmp_x + sin(angle)*tmp_y;
	grid_map.info.origin.position.y = transform.getOrigin().y()-sin(angle)*tmp_x - cos(angle)*tmp_y;
	grid_map.info.origin.position.z = -0.5;//////
	grid_map.info.origin.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,angle);
	
}

void map_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
	boost::mutex::scoped_lock(map_mutex_);
	map_in=*msg;
    callback_flag=true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "map_coord_tf");
	ros::NodeHandle n;
	
	ros::Subscriber sub = n.subscribe("/local_map", 1, map_callback);
	ros::Publisher map_pub= n.advertise<nav_msgs::OccupancyGrid>("/local_map/expand/gc", 1);

    
    bool tf_flag=true;
    nav_msgs::OccupancyGrid grid_map;    
    tf::TransformListener listener;
  	grid_map.data.resize(133*133);
    
    ros::Rate loop_rate(20);
    while (ros::ok()) {

        tf::StampedTransform transform;
		try{
			//listener.waitForTransform(header_frame, sensor_frame, ros::Time(0), ros::Duration(1.0));
			listener.lookupTransform(header_frame, sensor_frame, ros::Time(0), transform);
            tf_flag=true;
		}
		catch (tf::TransformException ex){
            tf_flag=false;
			ROS_ERROR("%s",ex.what());
		}
		
		if(callback_flag && tf_flag){
			grid_map=map_in;
			mapLocal2Global(grid_map, transform);	
			grid_map.header.frame_id="/map";
			map_pub.publish(grid_map);
            //callback_flag=false;
		}
		
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


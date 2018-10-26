#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>

#include <iostream>

nav_msgs::OccupancyGrid local_map;
bool map_received = false;
bool intersection = false;
bool target_received = false;
bool odom_received = false;
float target_orientation = 0.0;
float robot_orientation = 0.0;
float MARGIN_ANGLE = 0.3;
double longest_path_length_angle = 0.0;
bool switch_intencity_is_obstacle = true;


float get_yaw(geometry_msgs::Quaternion q)
{
	double r,p,y;
	tf::Quaternion quat(q.x,q.y,q.z,q.w);
	tf::Matrix3x3(quat).getRPY(r,p,y);
	return y;
}

float normalize(float z)
{
  return atan2(sin(z),cos(z));
}
float angle_diff(float a, float b)
{
  float d1, d2;
  a = normalize(a);
  b = normalize(b);
  d1 = a-b;
  d2 = 2*M_PI - fabs(d1);
  if(d1 > 0)
    d2 *= -1.0;
  if(fabs(d1) < fabs(d2))
    return(d1);
  else
    return(d2);
}

void TargetCallback(const std_msgs::Float64ConstPtr& msg)
{
  target_orientation = msg->data;
  target_received = true;
}

void lclCallback(const nav_msgs::OdometryConstPtr& msg)
{
  nav_msgs::Odometry odom = *msg;
  robot_orientation = odom.pose.pose.orientation.z;//get_yaw(odom.pose.pose.orientation);
  odom_received = true;
}

void IntersectionCallback(const std_msgs::BoolConstPtr& msg)
{
	intersection = msg->data;
	/*if(msg->data){
		intersection = true;
	}*/
}

void MapCallback(const nav_msgs::OccupancyGridConstPtr& msg)
{
  local_map = *msg;
  map_received = true;
}

int meterpoint_to_index(nav_msgs::OccupancyGrid grid, double x, double y)
{
	int x_ = x/grid.info.resolution + grid.info.width/2.0;
	int y_ = y/grid.info.resolution + grid.info.height/2.0;
	int index = y_*grid.info.width + x_;
	return index;
}

bool radial_search(double& path_length, double theta, double search_range)
{
	double x, y;
	while(ros::ok()){
		path_length += local_map.info.resolution;
		if(path_length>search_range){
			path_length = search_range;
			return true;
		}
		x = path_length*cos(theta);
		y = path_length*sin(theta);
		if(switch_intencity_is_obstacle){
			if(local_map.data[meterpoint_to_index(local_map, x, y)]>0 || local_map.data[meterpoint_to_index(local_map, x, y)]==-1)	return false;
		}
		else{
			if(local_map.data[meterpoint_to_index(local_map, x, y)]>50 || local_map.data[meterpoint_to_index(local_map, x, y)]==-1)	return false;
		}
	}
}
void detection_main(geometry_msgs::PoseStamped& goal)
{
    tf::TransformListener listener;
	tf::StampedTransform transform;
	try{
		listener.waitForTransform(local_map.header.frame_id, "/velodyne_odom", local_map.header.stamp, ros::Duration(1.0));
		listener.lookupTransform(local_map.header.frame_id, "/velodyne_odom", local_map.header.stamp, transform);
	}
	catch(tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}

	double roll, pitch, yaw;
	tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
	
	double target = yaw + angle_diff(target_orientation,robot_orientation);

	const double angle_step = 5.0;	//[deg]
	const double angle_range = 45.0;	//[deg]
	double search_range = local_map.info.width*local_map.info.resolution*0.25;
	if(search_range>local_map.info.height*local_map.info.resolution){
		search_range = local_map.info.height*local_map.info.resolution;
	}
	search_range -= 1.0;
	bool reached_end = false;

	double longest_path_length = 0.0;
	for(double step=0.0;step<=angle_range;step+=angle_step){
		double path_length = 0.0;
		double theta = target + step/180.0*M_PI;
		theta = normalize(theta);	
		reached_end = radial_search(path_length, theta, search_range);
		if(longest_path_length<path_length){
			longest_path_length = path_length;
			longest_path_length_angle = theta;
		}
		if(reached_end)	break;

		if(step!=0.0 && step!=180.0){
			path_length = 0.0;
			theta = target - step/180.0*M_PI;
			theta = normalize(theta);	
			reached_end = radial_search(path_length, theta, search_range);
			if(longest_path_length<path_length){
				longest_path_length = path_length;
				longest_path_length_angle = theta;
			}
		}
		if(reached_end)	break;
		if(step == -angle_range){
			std::cout << "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa" << std::endl;
		}
	}

	//if(intersection && goal_diff_angle < MARGIN_ANGLE){
	//  std::cout<< "end_trun" << std::endl;
	//  intersection = false;
	//}
	goal.header = local_map.header;
	goal.pose.position.x = longest_path_length*cos(longest_path_length_angle); 
	goal.pose.position.y = longest_path_length*sin(longest_path_length_angle); 
	goal.pose.position.z = 0.0;
	goal.pose.orientation = tf::createQuaternionMsgFromYaw(longest_path_length_angle);
	std::cout << "---------------------------------" << std::endl;
	std::cout << "intersection:"<< intersection << std::endl;
	std::cout << "search_range:" << search_range << std::endl;
	std::cout << "yaw:" << yaw << std::endl;
	std::cout << "robot_orientation:" << robot_orientation << std::endl;
	std::cout << "target_orientation:" << target_orientation << std::endl;
	std::cout << "target:" << target << std::endl;
	std::cout << "longest_path_angle:" << longest_path_length_angle << std::endl;
	std::cout << "longest_path_length:" << longest_path_length << std::endl;
}

void LocalGoalCreator()
{
  ros::NodeHandle nh;
  ros::Subscriber sub_map = nh.subscribe("/local_map", 1, MapCallback);
  ros::Subscriber sub_flag = nh.subscribe("/intersection_flag", 1, IntersectionCallback);
  ros::Subscriber sub_target = nh.subscribe("/target_yaw", 1, TargetCallback);
  ros::Subscriber sub_lcl = nh.subscribe("/lcl", 1, lclCallback);
  
  ros::Publisher pub_goal = nh.advertise<geometry_msgs::PoseStamped>("/local_goal",1);

  geometry_msgs::PoseStamped local_goal;

  ros::Rate loop_rate(10);
  while(ros::ok()){
	if(map_received && target_received && odom_received){
	  detection_main(local_goal);
	  //std::cout << local_goal << std::endl;
	  pub_goal.publish(local_goal);
	}
	else{
	  std::cout << "map:" << map_received << " target:" << target_received << " odom:"<<odom_received << std::endl;
	}
	ros::spinOnce();
	loop_rate.sleep();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "local_goal_creator");
  
  LocalGoalCreator();
  
  ROS_INFO("Killing now!!!!!");
  return 0;
}

#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <trajectory_generation/TrajectoryGeneration.h>
#include <fstream>
#include <string>
#include <list>
#include <map>
#include "trajectory_generation/Visualize_lib.h"
#include "trajectory_generation/trajectory_generation.h"
#include "trajectory_generation/boundary_state.h"
#include "trajectory_generation/LocalPlanning_lib.h"

using namespace std;
geometry_msgs::PoseStamped goal;
geometry_msgs::PoseStamped robo;
ros::Publisher goal_pub;
ros::Subscriber map_sub;
nav_msgs::OccupancyGrid grd;

void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg)
{
	grd=*msg;
	grd.info.origin.position.z=0;
}

int main(int argc,char **argv)
{
	ros::init(argc, argv, "goal_creator");
	ros::NodeHandle n;
	goal_pub = n.advertise<geometry_msgs::PoseStamped>("/local_goal",1);
	map_sub = n.subscribe("local_map/expand/gc", 5, mapCallback);
	robo.pose.position.x=0;
	robo.pose.position.y=0;
	robo.pose.position.z=0;
	robo.pose.orientation.z=0;
	
	return 0;
}

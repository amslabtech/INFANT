/*
 *	longpath_radial_search.cpp
 */

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
// #include <nav_msgs/Odometry.h>

class LongPathDetection{
	private:
		ros::NodeHandle nh;
		ros::Subscriber sub_grid;
		ros::Publisher pub_point;
		nav_msgs::OccupancyGrid grid;
		geometry_msgs::PoseStamped farthest_point;
		//geometry_msgs::PointStamped farthest_point;
		tf::TransformListener tflistener;

	public:
		LongPathDetection();
		void callback_grid(const nav_msgs::OccupancyGridConstPtr& msg);
		void detection_main(void);
		bool radial_search(double& path_length, double theta, double search_range);
};

LongPathDetection::LongPathDetection()
{
	//sub_grid = nh.subscribe("/occupancygrid/store", 1, &LongPathDetection::callback_grid, this);
	//pub_point = nh.advertise<geometry_msgs::PointStamped>("/farthest_point", 1);
	sub_grid = nh.subscribe("/local_map", 1, &LongPathDetection::callback_grid, this);
	pub_point = nh.advertise<geometry_msgs::PoseStamped>("/local_goal", 1);
	//pub_point = nh.advertise<geometry_msgs::PointStamped>("/local_goal", 1);
}

void LongPathDetection::callback_grid(const nav_msgs::OccupancyGridConstPtr& msg)
{
	grid = *msg;

	detection_main();

	pub_point.publish(farthest_point);
}

int meterpoint_to_index(nav_msgs::OccupancyGrid grid, double x, double y)
{
	int x_ = x/grid.info.resolution + grid.info.width/2.0;
	int y_ = y/grid.info.resolution + grid.info.height/2.0;
	int index = y_*grid.info.width + x_;
	return index;
}

void LongPathDetection::detection_main(void)
{
	tf::StampedTransform transform;
	try{
		tflistener.waitForTransform(grid.header.frame_id, "/velodyne", grid.header.stamp, ros::Duration(1.0));
		tflistener.lookupTransform(grid.header.frame_id, "/velodyne", grid.header.stamp, transform);
	}
	catch(tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}

	double roll, pitch, yaw;
	tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);

	const double angle_step = 10.0;	//[deg]
	const double angle_range = 90.0;	//[deg]
	double search_range = grid.info.width*grid.info.resolution*0.5;
	if(search_range>grid.info.height*grid.info.resolution)	search_range = grid.info.height*grid.info.resolution;
	search_range -= 1.0;
	bool reached_end = false;

	double longest_path_length = 0.0;
	double longest_path_length_angle = 0.0;
	for(double step=0.0;step<=angle_range;step+=angle_step){
		double path_length = 0.0;
		double theta = yaw + step/180.0*M_PI;
		if(theta>M_PI)	theta -= 2.0*M_PI;
		if(theta<M_PI)	theta += 2.0*M_PI;
		
		reached_end = radial_search(path_length, theta, search_range);
		if(longest_path_length<path_length){
			longest_path_length = path_length;
			longest_path_length_angle = theta;
		}
		if(reached_end)	break;

		if(step!=0.0 && step!=180.0){
			path_length = 0.0;
			theta = yaw - step/180.0*M_PI;
			reached_end = radial_search(path_length, theta, search_range);
			if(longest_path_length<path_length){
				longest_path_length = path_length;
				longest_path_length_angle = theta;
			}
		}
		if(reached_end)	break;
	}

	farthest_point.header = grid.header;
	farthest_point.pose.position.x = longest_path_length*cos(longest_path_length_angle); 
	farthest_point.pose.position.y = longest_path_length*sin(longest_path_length_angle); 
	farthest_point.pose.position.z = 0.0;
	farthest_point.pose.orientation = tf::createQuaternionMsgFromYaw(longest_path_length_angle);
}

bool LongPathDetection::radial_search(double& path_length, double theta, double search_range)
{
	double x, y;
	while(ros::ok()){
		path_length += grid.info.resolution;
		if(path_length>search_range){
			path_length = search_range;
			return true;
		}
		x = path_length*cos(theta);
		y = path_length*sin(theta);
		if(grid.data[meterpoint_to_index(grid, x, y)]>0 || grid.data[meterpoint_to_index(grid, x, y)]==-1)	return false;
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "longpath_radial_search");

	LongPathDetection longpathdetection;

	ros::spin();
}

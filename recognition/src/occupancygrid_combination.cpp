#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

class OccupancyGridCombination{
	private:
		ros::NodeHandle nh;
		/*subscribe*/
		ros::Subscriber sub_grid_lidar;
		ros::Subscriber sub_grid_zed;
		ros::Subscriber sub_odom;
		/*publish*/
		ros::Publisher pub;
		/*objects*/
		nav_msgs::OccupancyGrid grid;
		// nav_msgs::OccupancyGrid grid_all_minusone;
		nav_msgs::Odometry odom;
		/*flags*/
		bool first_callback_grid = true;
		bool first_callback_odom = true;
		/*time*/
		ros::Time time_odom_now;
		ros::Time time_odom_last;
		double time_moving;
		double time_nomove;
	public:
		OccupancyGridCombination();
		void CallbackGrid(const nav_msgs::OccupancyGridConstPtr& msg);
		void CallbackOdom(const nav_msgs::OdometryConstPtr& msg);
		void MoveCells(double dt);
		void IndexToMeterpoint(nav_msgs::OccupancyGrid grid, int index, double& x, double& y);
		int MeterpointToIndex(nav_msgs::OccupancyGrid grid, double x, double y);
		bool MeterPointIsInside(nav_msgs::OccupancyGrid grid, double x, double y);
		void Publication(void);
};

void OccupancyGridCombination::CallbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	// std::cout << "- CALLBACK ODOM -" << std::endl;
	time_odom_now = ros::Time::now();
	double dt = (time_odom_now - time_odom_last).toSec();
	time_odom_last = time_odom_now;
	if(first_callback_odom)	dt = 0.0;

	if(msg->twist.twist.linear.x>1.0e-3){
		time_nomove = 0.0;
		time_moving += dt;
	}
	else{
		time_nomove += dt;
		time_moving = 0.0;
	}
	
	const double time_shrink = 3.0;	//[s]
	const double time_initialize = 5.0;	//[s]
	if(!grid.data.empty()){
		if(time_nomove>time_initialize || first_callback_odom)	initialize_around_startpoint();
		else if(time_nomove>time_shrink)	shrink_obstacle(grid_store);
		else{
			move_cells(dt);
			robot_is_running = true;
		}
	}
	// if(!first_callback_odom && !grid_store.data.empty() && robot_is_running)	move_cells(dt);

	first_callback_odom = false;
}

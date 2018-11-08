/*
 *	occupancygrid_store.cpp
 */

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

/*global variables*/
nav_msgs::OccupancyGrid grid_lidar;
nav_msgs::OccupancyGrid grid_zed;
nav_msgs::OccupancyGrid grid_store;
nav_msgs::Odometry odom;
ros::Time time_odom_now;
ros::Time time_odom_last;
bool first_callback_odom = true;
bool first_callback_grid = true;
double theta = 0.0;
double delta_x = 0.0;
double delta_y = 0.0;
bool robot_is_running = false;
double nomove_time = 0.0;
double time_moving = 0.0;

bool cell_is_inside_meter(nav_msgs::OccupancyGrid grid, double x, double y)
{
	if(fabs(x)>grid.info.width*grid.info.resolution/2.0)	return false;
	if(fabs(y)>grid.info.height*grid.info.resolution/2.0)	return false;
	return true;
}

bool cell_is_inside(nav_msgs::OccupancyGrid grid, int x, int y)
{   
	// std::cout << "CELL IS INSIDE" << std::endl;
	
	int w = grid.info.width;
	int h = grid.info.height;
	if(x<-w/2.0)  return false;
	if(x>w/2.0-1) return false;
	if(y<-h/2.0)  return false;
	if(y>h/2.0-1) return false;
	return true;
}

void index_to_point(nav_msgs::OccupancyGrid grid, int index, int& x, int& y)
{
	x = index%grid.info.width - grid.info.width/2.0;
	y = index/grid.info.width - grid.info.height/2.0;
	// std::cout << "index = " << index << std::endl;
	// std::cout << "x = " << x << std::endl;
	// std::cout << "y = " << y << std::endl;
}

int point_to_index(nav_msgs::OccupancyGrid grid, int x, int y)
{
	// std::cout << "- POINT TO INDEX -" << std::endl;
	int x_ = x + grid.info.width/2.0;
	int y_ = y + grid.info.height/2.0;
	return	y_*grid.info.width + x_;
}

// void expand_obstacle(nav_msgs::OccupancyGrid& grid)
// {
// 	const int range = 3;
// 	std::vector<int> indices_obs;
// 	for(size_t i=0;i<grid.data.size();i++)	if(grid.data[i]==100) indices_obs.push_back(i);
// 	for(size_t i=0;i<indices_obs.size();i++){
// 		int x, y;
// 		index_to_point(grid, indices_obs[i], x, y);
// 		for(int j=-range;j<=range;j++){
// 			for(int k=-range;k<=range;k++){
// 				if(cell_is_inside(grid, x+j, y+k))	grid.data[point_to_index(grid, x+j, y+k)] = grid.data[indices_obs[i]];
// 			}
// 		}
// 	}
// }

void expand_obstacle(nav_msgs::OccupancyGrid& grid)
{
	const int range = 2;
	for(size_t i=0;i<grid.data.size();i++){
		if(grid.data[i]==100){
			int x, y;
			index_to_point(grid, i, x, y);
			for(int j=-range;j<=range;j++){
				for(int k=-range;k<=range;k++){
					if(cell_is_inside(grid, x+j, y+k) && grid.data[point_to_index(grid, x+j, y+k)]!=grid.data[i])	grid.data[point_to_index(grid, x+j, y+k)] = grid.data[i]-1;
				}
			}
		}
	}
}

void partial_expand_obstacle(nav_msgs::OccupancyGrid& grid)
{
	const double range_no_expand = 2.0;	//[m]
	const int range = 2;
	for(size_t i=0;i<grid.data.size();i++){
		if(grid.data[i]==100){
			int x, y;
			index_to_point(grid, i, x, y);
			double dist = sqrt(x*grid.info.resolution*x*grid.info.resolution + y*grid.info.resolution*y*grid.info.resolution);
			if(dist>range_no_expand){
				for(int j=-range;j<=range;j++){
					for(int k=-range;k<=range;k++){
						if(cell_is_inside(grid, x+j, y+k) && grid.data[point_to_index(grid, x+j, y+k)]!=grid.data[i])	grid.data[point_to_index(grid, x+j, y+k)] = grid.data[i]-1;
					}
				}
			}
		}
	}
}

void shrink_obstacle(nav_msgs::OccupancyGrid& grid)
{
	for(size_t i=0;i<grid.data.size();i++){
		if(grid.data[i]==99)	grid.data[i] = 0;
	}
}

void ambiguity_filter(nav_msgs::OccupancyGrid& grid)	//for ambiguity of intensity
{
	// std::cout << "AMBIGUITY FILTER" << std::endl;
	
	std::vector<int> indices_obs;

	const int range = 3;
	for(size_t i=0;i<grid.data.size();i++){
		// if(grid.data[i]>0 && grid.data[i]<100){
		if(grid.data[i]>0 && grid.data[i]<99){
			int x, y;
			index_to_point(grid, i, x, y);
			int count_zerocell = 0; 
			for(int j=-range;j<=range;j++){
				for(int k=-range;k<=range;k++){
					if(cell_is_inside(grid, x+j, y+k) && grid.data[point_to_index(grid, x+j, y+k)]==0)	count_zerocell++;
				}
			}
			int num_cells = (2*range + 1)*(2*range + 1);
			const double ratio = 0.725;
			double threshold = num_cells*ratio;
			if(count_zerocell>threshold)	grid.data[i] = 0;
			// if(count_zerocell>0)	std::cout << "count_zerocell = " << count_zerocell << std::endl;
		}
	}
}

void index_to_meterpoint(nav_msgs::OccupancyGrid grid, int index, double& x, double& y)
{
	x = (index%grid.info.width - grid.info.width/2.0 + 0.5)*grid.info.resolution;
	y = (index/grid.info.width - grid.info.height/2.0 + 0.5)*grid.info.resolution;
}

int meterpoint_to_index(nav_msgs::OccupancyGrid grid, double x, double y)
{
	int x_ = x/grid.info.resolution + grid.info.width/2.0;
	int y_ = y/grid.info.resolution + grid.info.height/2.0;
	int index = y_*grid.info.width + x_;
	return index;
}

void move_cells(double dt)
{
	// std::cout << "MOVE CELLS" << std::endl;
	
	theta += odom.twist.twist.angular.z*dt;
	if(theta<-M_PI)	theta += 2*M_PI;
	if(theta>M_PI)	theta -= 2*M_PI;
	
	delta_x += odom.twist.twist.linear.x*dt * cos(theta);
	delta_y += odom.twist.twist.linear.x*dt * sin(theta);
	// std::cout << "theta = " << theta << std::endl;
	// std::cout << "delta_x = " << delta_x << std::endl;
	// std::cout << "delta_y = " << delta_y << std::endl;
	
	if(fabs(delta_x)>grid_store.info.resolution || fabs(delta_y)>grid_store.info.resolution){
		int delta_x_ = delta_x/grid_store.info.resolution;
		int delta_y_ = delta_y/grid_store.info.resolution;
		// std::cout << "delta_x_ = " << delta_x_ << std::endl;
		// std::cout << "delta_y_ = " << delta_y_ << std::endl;
		
		nav_msgs::OccupancyGrid tmp_grid = grid_store;
		for(size_t i=0;i<tmp_grid.data.size();i++)	tmp_grid.data[i] = -1;
		for(size_t i=0;i<grid_store.data.size();i++){
			double x, y;
			index_to_meterpoint(grid_store, i, x, y);
			// std::cout << i << " = " << meterpoint_to_index(grid_store, x, y) << std::endl;
			x -= delta_x_*grid_store.info.resolution;
			y -= delta_y_*grid_store.info.resolution;	
			// std::cout << "x = " << x << std::endl;
			// std::cout << "y = " << y << std::endl;
			int index_moved = meterpoint_to_index(grid_store, x, y);
			// std::cout << "index_moved = " << index_moved << std::endl;
			if(cell_is_inside_meter(grid_store, x, y))	tmp_grid.data[index_moved] = grid_store.data[i];
		}
		delta_x -= delta_x_*grid_store.info.resolution;
		delta_y -= delta_y_*grid_store.info.resolution;

		grid_store = tmp_grid;
	}
}

void initialize_around_startpoint(void)
{
	const double initialize_range_meter = 5.0;	//[m]
	int range = initialize_range_meter/grid_store.info.resolution;
	for(int i=-range;i<=range;i++){
		for(int j=-range;j<=range;j++){
			grid_store.data[point_to_index(grid_store, i, j)] = 0;
		}
	}
}

void callback_odom(const nav_msgs::OdometryConstPtr& msg)
{
	// std::cout << "- CALLBACK ODOM -" << std::endl;
	odom = *msg;
	time_odom_now = ros::Time::now();
	double dt = (time_odom_now - time_odom_last).toSec();
	time_odom_last = time_odom_now;
	
	if(odom.twist.twist.linear.x>1.0e-3){
		nomove_time = 0.0;
		time_moving += dt;
	}
	else{
		nomove_time += dt;
		time_moving = 0.0;
	}
	
	const double time_shrink = 3.0;	//[s]
	const double time_initialize = 5.0;	//[s]
	if(!grid_store.data.empty()){
		if(nomove_time>time_initialize || first_callback_odom){
			initialize_around_startpoint();
			robot_is_running = false;
		}
		else if(nomove_time>time_shrink){
			shrink_obstacle(grid_store);
			robot_is_running = false;
		}
		else{
			move_cells(dt);
			robot_is_running = true;
		}
	}
	// if(!first_callback_odom && !grid_store.data.empty() && robot_is_running)	move_cells(dt);

	first_callback_odom = false;
}

void grid_update_lidar(void)
{
	for(size_t i=0;i<grid_store.data.size();i++){
		if(grid_lidar.data[i]!=-1)	grid_store.data[i] = grid_lidar.data[i];
	}
}

void grid_update_zed(void)
{
	for(size_t i=0;i<grid_store.data.size();i++){
		if(grid_store.data[i]==-1)	grid_store.data[i] = grid_zed.data[i];
		else if(grid_store.data[i]==50 && grid_zed.data[i]==0)	grid_store.data[i] = 0;
		else if(grid_store.data[i]==0 && grid_zed.data[i]==50)	grid_store.data[i] = 25;
	}
}

void callback_grid_lidar(const nav_msgs::OccupancyGridConstPtr& msg)
{
	// std::cout << "- CALLBACK GRID -" << std::endl;
	
	grid_lidar = *msg;
	if(grid_store.data.empty())	grid_store = *msg;

	// expand_obstacle(grid_lidar);

	// ambiguity_filter(grid_now);
	grid_update_lidar();
	// ambiguity_filter(grid_store);

	// if(first_callback_grid || !robot_is_running)	initialize_around_startpoint();
	first_callback_grid = false;
}

void callback_grid_zed(const nav_msgs::OccupancyGridConstPtr& msg)
{
	// std::cout << "- CALLBACK GRID -" << std::endl;
	
	grid_zed = *msg;
	if(grid_store.data.empty())	grid_store = *msg;

	// ambiguity_filter(grid_now);
	grid_update_zed();
	// ambiguity_filter(grid_store);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "occupancygrid_store");
	ros::NodeHandle nh;

	/*sub*/
	ros::Subscriber sub_grid_lidar = nh.subscribe("/occupancygrid/lidar", 1, callback_grid_lidar);
	ros::Subscriber sub_grid_zed = nh.subscribe("/occupancygrid/zed", 1, callback_grid_zed);
	ros::Subscriber sub_odom = nh.subscribe("/tinypower/odom", 1, callback_odom);
	
	/*pub*/
	ros::Publisher pub_grid = nh.advertise<nav_msgs::OccupancyGrid>("/local_map",1);
	//ros::Publisher pub_grid = nh.advertise<nav_msgs::OccupancyGrid>("/occupancygrid/store",1);
	
	/*variables*/
	
	/*initialization*/

	/*loop*/
	ros::Rate loop_rate(40);
	while(ros::ok()){
		ros::spinOnce();
		
		const double time_expand = 3.0;	//[s]
		if(!grid_store.data.empty()){
			if(time_moving>time_expand)	expand_obstacle(grid_store);
			else	partial_expand_obstacle(grid_store);
			ambiguity_filter(grid_store);
			pub_grid.publish(grid_store);
		}
		
		loop_rate.sleep();
	}
}

/*
 *	gridmap_fusion.cpp
 */

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

/*global variables*/
nav_msgs::OccupancyGrid grid_lidar;
nav_msgs::OccupancyGrid grid_zed;
nav_msgs::OccupancyGrid grid;
nav_msgs::Odometry odom;
ros::Time time_odom_now;
ros::Time time_odom_last;
bool odom_flag = true;
bool lidar_flag = false;
bool zed_flag = false;
double nomove_time = 0.0;
double time_moving = 0.0;

void callback_grid_lidar(const nav_msgs::OccupancyGridConstPtr& msg)
{
	std::cout << "- CALLBACK GRID -" << std::endl;
	
	grid_lidar = *msg;
    /* if(!lidar_flag){grid.info = grid_lidar.info;} */
    if(grid.data.empty()) grid = *msg;
	
	if(!lidar_flag){
		if(!zed_flag){
			grid_zed = *msg;
			for(size_t i=0;i<grid_zed.data.size();i++)	grid_zed.data[i] = -1;
		}
	}

	lidar_flag = true;
}

void callback_grid_zed(const nav_msgs::OccupancyGridConstPtr& msg)
{
	std::cout << "- CALLBACK GRID -" << std::endl;
	
	grid_zed = *msg;
    if(grid.data.empty()) grid = *msg;
	
	if(!zed_flag){
		if(!lidar_flag){
			grid_lidar = *msg;
			for(size_t i=0;i<grid_lidar.data.size();i++)	grid_lidar.data[i] = -1;
		}
	}
	zed_flag = true;

}
int point_to_index(nav_msgs::OccupancyGrid grid, int x, int y)
{
	/* std::cout << "- POINT TO INDEX -" << std::endl; */
	int x_ = x + grid.info.width/2.0;
	int y_ = y + grid.info.height/2.0;
	return	y_*grid.info.width + x_;
}
void initialize_around_startpoint(void)
{
	const double initialize_range_meter = 5.0;	//[m]
	int range = initialize_range_meter/grid.info.resolution;
	for(int i=-range;i<=range;i++){
		for(int j=-range;j<=range;j++){
			grid.data[point_to_index(grid, i, j)] = 0;
		}
	}
}
void shrink_obstacle(nav_msgs::OccupancyGrid& grid)
{
	for(size_t i=0;i<grid.data.size();i++){
		if(grid.data[i]==99)	grid.data[i] = 0;
	}
}
void callback_odom(const nav_msgs::OdometryConstPtr& msg)
{
	std::cout << "- CALLBACK ODOM -" << std::endl;
	odom = *msg;
	time_odom_now = ros::Time::now();
	double dt = (time_odom_now - time_odom_last).toSec();
	time_odom_last = time_odom_now;
    if(odom_flag)  dt = 0.0;
	
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
	if(!grid.data.empty()){
		if(nomove_time>time_initialize || odom_flag){
			initialize_around_startpoint();
		}
		else if(nomove_time>time_shrink){
			shrink_obstacle(grid);
		}
	}

	odom_flag = false;
}

void combine()
{
    for(int i=0;i<grid.info.width*grid.info.height;i++){
        grid.data[i] = grid_lidar.data[i];
        switch(grid_lidar.data[i]){
            case -1:
                grid.data[i] = grid_zed.data[i];
                break;
            case 50:
                if(!grid_zed.data[i]){grid.data[i] = 0;}
                break;
        }

    }
}
void index_to_point(nav_msgs::OccupancyGrid grid, int index, int& x, int& y)
{
	x = index%grid.info.width - grid.info.width/2.0;
	y = index/grid.info.width - grid.info.height/2.0;grid_lidar;
	// std::cout << "index = " << index << std::endl;
	// std::cout << "x = " << x << std::endl;
	// std::cout << "y = " << y << std::endl;
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


void ambiguity_filter(nav_msgs::OccupancyGrid& grid)	//for ambiguity of intensity
{
	std::cout << "AMBIGUITY FILTER" << std::endl;
	
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
int main(int argc, char** argv)
{
	ros::init(argc, argv, "gridmap_fusion");
	ros::NodeHandle nh;

	/*sub*/
	ros::Subscriber sub_grid_lidar = nh.subscribe("/occupancygrid/lidar/stored", 1, callback_grid_lidar);
	ros::Subscriber sub_grid_zed = nh.subscribe("/occupancygrid/zed/stored", 1, callback_grid_zed);
	ros::Subscriber sub_odom = nh.subscribe("/tinypower/odom", 1, callback_odom);
	
	/*pub*/
	ros::Publisher pub_grid = nh.advertise<nav_msgs::OccupancyGrid>("/local_map",1);
	//ros::Publisher pub_grid = nh.advertise<nav_msgs::OccupancyGrid>("/occupancygrid/store",1);
	
	/*variables*/
	
	/*initialization*/
	const double time_expand = 3.0;	//[s]
    

	/*loop*/
	ros::Rate loop_rate(40);
	while(ros::ok()){
		// if(lidar_flag && zed_flag){
		if(!grid_lidar.data.empty() && !grid_zed.data.empty()){
			if(nomove_time>5.0)	initialize_around_startpoint();
			else	combine();
			if(time_moving>time_expand)	expand_obstacle(grid);
			else	partial_expand_obstacle(grid);
			ambiguity_filter(grid);
			pub_grid.publish(grid);
		}
		ros::spinOnce();
		
		loop_rate.sleep();
	}
}



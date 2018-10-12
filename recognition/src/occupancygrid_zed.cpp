/*
 *	occupancygrid_zed.cpp
 */

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/OccupancyGrid.h>

/*global variables*/
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_grass (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_road (new pcl::PointCloud<pcl::PointXYZ>);
nav_msgs::OccupancyGrid grid;
const double w = 11.0;	//x[m]
const double h = 11.0;	//y[m]

bool cell_is_inside(int x, int y)
{
	int w = grid.info.width;
	int h = grid.info.height;
	if(x<-w/2.0)  return false; 
	if(x>w/2.0-1) return false;
	if(y<-h/2.0)  return false;
	if(y>h/2.0-1) return false;
	return true;
}

void index_to_point(int index, int& x, int& y)
{
	x = index%grid.info.width - grid.info.width/2.0;
	y = index/grid.info.width - grid.info.height/2.0;
	// std::cout << "index = " << index << std::endl;
	// std::cout << "x = " << x << std::endl;
	// std::cout << "y = " << y << std::endl;
}

int point_to_index(int x, int y)
{
	// std::cout << "- POINT TO INDEX -" << std::endl;
	int x_ = x + grid.info.width/2.0;
	int y_ = y + grid.info.height/2.0;
	return	y_*grid.info.width + x_;
}

// void filter(void)
// {
// 	const int range = 1;
// 	for(int i=0;i<grid.info.width*grid.info.height;i++){
// 		if(grid.data[i]==-1){
// 			// std::cout << "-----" << std::endl;
// 			int x, y;
// 			index_to_point(i, x, y);
// 			int count_roadcell = 0;
// 			for(int j=-range;j<=range;j++){
// 				for(int k=-range;k<=range;k++){
// 					if(cell_is_inside(x+j, y+k) && grid.data[point_to_index(x+j, y+k)]==0)	count_roadcell++;
// 				}
// 			}
// 			if(count_roadcell>(2*range+1)*(2*range+1)-1){
// 				// std::cout << "Grid is updated" << std::endl;
// 				grid.data[i] = 0;
// 			}
// 		}
// 	}
// }

int meterpoint_to_index(double x, double y)
{
	int x_ = x/grid.info.resolution + grid.info.width/2.0;
	int y_ = y/grid.info.resolution + grid.info.height/2.0;
	int index = y_*grid.info.width + x_;
	return index;
}

bool point_is_inside(pcl::PointXYZ p)
{
	if(fabs(p.x)>w/2.0)	return false;
	if(fabs(p.y)>h/2.0)	return false;
	return true;
}

void input_grid(void)
{
	// std::cout << "- INPUT GRID -" << std::endl;
	grid.header.frame_id = cloud_grass->header.frame_id;

	for(int i=0;i<grid.info.width*grid.info.height;i++)		grid.data[i] = -1;

	for(size_t i=0;i<cloud_grass->points.size();i++){
		if(point_is_inside(cloud_grass->points[i]))	grid.data[meterpoint_to_index(cloud_grass->points[i].x, cloud_grass->points[i].y)] = 50;
	}
	for(size_t i=0;i<cloud_road->points.size();i++){
		if(point_is_inside(cloud_road->points[i]))	grid.data[meterpoint_to_index(cloud_road->points[i].x, cloud_road->points[i].y)] = 0;
	}
}

void callback_cloud_grass(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	// std::cout << "- CLOUD CALLBACK -" << std::endl;
	pcl::fromROSMsg(*msg, *cloud_grass);

	// filter();
}

void callback_cloud_road(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	// std::cout << "- CLOUD CALLBACK -" << std::endl;
	pcl::fromROSMsg(*msg, *cloud_road);

	// filter();
}

void grid_initialization(void)
{
	// grid.header.frame_id = "/localmap";
	// grid.header.frame_id = "/velodyne";
	grid.info.resolution = 0.1;
	grid.info.width = w/grid.info.resolution + 1;
	grid.info.height = h/grid.info.resolution + 1;
	grid.info.origin.position.x = -w/2.0;
	grid.info.origin.position.y = -h/2.0;
	grid.info.origin.position.z = 0.0;
	grid.info.origin.orientation.x = 0.0;
	grid.info.origin.orientation.y = 0.0;
	grid.info.origin.orientation.z = 0.0;
	grid.info.origin.orientation.w = 1.0;
	
	for(int i=0;i<grid.info.width*grid.info.height;i++)	grid.data.push_back(-1);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "occupancygrid_zed");
	ros::NodeHandle nh;

	/*sub*/
	ros::Subscriber sub_cloud_grass = nh.subscribe("/zed_grasspoints/transformed", 1, callback_cloud_grass);
	ros::Subscriber sub_cloud_road = nh.subscribe("/zed_roadpoints/transformed", 1, callback_cloud_road);
	
	/*pub*/
	ros::Publisher pub_grid = nh.advertise<nav_msgs::OccupancyGrid>("/occupancygrid/zed",1);
	
	/*variables*/
	
	/*initialization*/
	grid_initialization();

	/*loop*/
	ros::Rate loop_rate(2);
	while(ros::ok()){
		ros::spinOnce();
		if(!cloud_grass->points.empty()){
			input_grid();
			pub_grid.publish(grid);
		}
		loop_rate.sleep();
	}
}

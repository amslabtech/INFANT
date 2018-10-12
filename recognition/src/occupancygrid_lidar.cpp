/*
 *	occupancygrid_lidar.cpp
 */

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <nav_msgs/OccupancyGrid.h>

/*global variables*/
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_obstacles (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_ground (new pcl::PointCloud<pcl::PointXYZINormal>);
nav_msgs::OccupancyGrid grid;
const double w = 11.0;	//x[m]
const double h = 11.0;	//y[m]
// std::vector<double> fitting_errors;

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


void filter(void)
{
	const int range = 1;
	for(int i=0;i<grid.info.width*grid.info.height;i++){
		if(grid.data[i]==-1){
			// std::cout << "-----" << std::endl;
			int x, y;
			index_to_point(i, x, y);
			int count_roadcell = 0;
			for(int j=-range;j<=range;j++){
				for(int k=-range;k<=range;k++){
					if(cell_is_inside(x+j, y+k) && grid.data[point_to_index(x+j, y+k)]==0)	count_roadcell++;
				}
			}
			if(count_roadcell>(2*range+1)*(2*range+1)-1){
				// std::cout << "Grid is updated" << std::endl;
				grid.data[i] = 0;
			}
		}
	}
}

int meterpoint_to_index(double x, double y)
{
	int x_ = x/grid.info.resolution + grid.info.width/2.0;
	int y_ = y/grid.info.resolution + grid.info.height/2.0;
	int index = y_*grid.info.width + x_;
	return index;
}

void input_grid(void)
{
	// std::cout << "- INPUT GRID -" << std::endl;
	grid.header.frame_id = cloud->header.frame_id;
	const double threshold_intensity = 20;
	const double threshold_curvature = 1.0e-4;
	const double threshold_fitting_error = 1.0e-10;


	for(int i=0;i<grid.info.width*grid.info.height;i++)		grid.data[i] = -1;

	for(size_t i=0;i<cloud_ground->points.size();i++){
		/*intensity*/
		if(cloud_ground->points[i].intensity<threshold_intensity)	grid.data[meterpoint_to_index(cloud_ground->points[i].x, cloud_ground->points[i].y)] = 0;
		else	grid.data[meterpoint_to_index(cloud_ground->points[i].x, cloud_ground->points[i].y)] = 50;
		
		/*curvature*/
		// if(cloud_ground->points[i].curvature<threshold_curvature)	grid.data[meterpoint_to_index(cloud_ground->points[i].x, cloud_ground->points[i].y)] = 0;
		// else	grid.data[meterpoint_to_index(cloud_ground->points[i].x, cloud_ground->points[i].y)] = 50;
		// std::cout << "cloud_ground->points[i].curvature = " << cloud_ground->points[i].curvature << std::endl;
		
		/*fitting_error*/
	// 	if(fitting_errors[i]<threshold_fitting_error)	grid.data[meterpoint_to_index(cloud_ground->points[i].x, cloud_ground->points[i].y)] = 0;
	// 	else	grid.data[meterpoint_to_index(cloud_ground->points[i].x, cloud_ground->points[i].y)] = 50;
	}
	for(size_t i=0;i<cloud_obstacles->points.size();i++){
		grid.data[meterpoint_to_index(cloud_obstacles->points[i].x, cloud_obstacles->points[i].y)] = 100;
		// std::cout << "cloud_obstacles " << i << ":" << cloud_obstacles->points[i] << std::endl;
	}
}

// double compute_fitting_error(Eigen::Vector4f plane_parameters, pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud, std::vector<int> indices)
// {
// 	float sum_square_error = 0.0;
// 	for(int i=0;i<indices.size();i++){
// 		float square_error =	(plane_parameters[0]*cloud->points[indices[i]].x
// 								+plane_parameters[1]*cloud->points[indices[i]].y
// 								+plane_parameters[2]*cloud->points[indices[i]].z
// 								+plane_parameters[3])
// 								*(plane_parameters[0]*cloud->points[indices[i]].x
// 								+plane_parameters[1]*cloud->points[indices[i]].y
// 								+plane_parameters[2]*cloud->points[indices[i]].z
// 								+plane_parameters[3])
// 								/(plane_parameters[0]*plane_parameters[0]
// 								+plane_parameters[1]*plane_parameters[1]
// 								+plane_parameters[2]*plane_parameters[2]);
// 		sum_square_error += square_error/(float)indices.size();
// 	}
// 	return sum_square_error;
// }
//
// std::vector<int> kdtree_search(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud, pcl::PointXYZINormal searchpoint, double search_radius)
// {
// 	pcl::KdTreeFLANN<pcl::PointXYZINormal> kdtree;
// 	kdtree.setInputCloud(cloud);
// 	std::vector<int> pointIdxRadiusSearch;
// 	std::vector<float> pointRadiusSquaredDistance;
// 	if(kdtree.radiusSearch(searchpoint, search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance)<=0)	std::cout << "kdtree error" << std::endl;
// 	return pointIdxRadiusSearch; 
// }
//
// void normal_estimation_(void)
// {
// 	fitting_errors.clear();
// 	for(size_t i=0;i<cloud_ground->points.size();i+=10){
// 		const double search_radius = 0.1;
// 		std::vector<int> indices = kdtree_search(cloud_ground, cloud_ground->points[i], search_radius);
// 		// pcl::computePointNormal(*cloud_ground, indices, plane_parameters, curvature);
// 		Eigen::Vector4f plane_parameters;
// 		pcl::computePointNormal(*cloud_ground, indices, plane_parameters, cloud_ground->points[i].curvature);
// 		cloud_ground->points[i].normal_x = plane_parameters[0];
// 		cloud_ground->points[i].normal_y = plane_parameters[1];
// 		cloud_ground->points[i].normal_z = plane_parameters[2];
// 		fitting_errors.push_back( compute_fitting_error(plane_parameters, cloud_ground, indices) );
// 	}
// }

void normal_estimation(void)
{
	pcl::NormalEstimation<pcl::PointXYZINormal, pcl::PointXYZINormal> ne;
	ne.setInputCloud (cloud_ground);
	pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZINormal> ());
	ne.setSearchMethod (tree);
	ne.setRadiusSearch (0.1);
	ne.compute (*cloud_ground);
}

bool point_is_obstacle(pcl::PointXYZI p)
{
	const double height_min = -1.24;
	const double height_max = 0.0;
	if(p.z<height_min)	return false;
	if(p.z>height_max)	return false;
	if(fabs(p.x)>w/2.0)	return false;
	if(fabs(p.y)>h/2.0)	return false;
	return true;
}

bool point_is_ground(pcl::PointXYZI p)
{
	const double height_min = -2.0;
	const double height_max = -1.24;
	if(p.z<height_min)	return false;
	if(p.z>height_max)	return false;
	if(fabs(p.x)>w/2.0)	return false;
	if(fabs(p.y)>h/2.0)	return false;
	return true;
}

void cloud_extraction(void)	//extraction
{
	// std::cout << "- CLOUD EXTRACTION -" << std::endl;
	
	// cloud_obstacles->points.clear();
	// cloud_ground->points.clear();
	
	pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_cloud_obstacles (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr tmp_cloud_ground (new pcl::PointCloud<pcl::PointXYZINormal>);
	
	for(size_t i=0;i<cloud->points.size();i++){
		if(point_is_obstacle(cloud->points[i]))	tmp_cloud_obstacles->points.push_back(cloud->points[i]);
		else if(point_is_ground(cloud->points[i])){
			pcl::PointXYZINormal tmp;
			tmp.x = cloud->points[i].x;
			tmp.y = cloud->points[i].y;
			tmp.z = cloud->points[i].z;
			tmp.intensity = cloud->points[i].intensity;
			tmp_cloud_ground->points.push_back(tmp);
		}
	}
	*cloud_obstacles = *tmp_cloud_obstacles;
	*cloud_ground = *tmp_cloud_ground;
}

void callback_cloud(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	// std::cout << "- CLOUD CALLBACK -" << std::endl;
	pcl::fromROSMsg(*msg, *cloud);

	cloud_extraction();
	normal_estimation();
	// normal_estimation_();
	input_grid();
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
	ros::init(argc, argv, "occupancygrid_lidar");
	ros::NodeHandle nh;

	/*sub*/
	ros::Subscriber sub_cloud = nh.subscribe("/velodyne_points/transformed", 1, callback_cloud);
	// ros::Subscriber sub_cloud = nh.subscribe("/velodyne_points", 1, callback_cloud);
	
	/*pub*/
	ros::Publisher pub_grid = nh.advertise<nav_msgs::OccupancyGrid>("/occupancygrid/lidar",1);
	ros::Publisher pub_cloud_obstacles = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points/obstacles",1);
	ros::Publisher pub_cloud_ground = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points/ground",1);
	
	/*variables*/
	
	/*initialization*/
	grid_initialization();

	/*loop*/
	ros::Rate loop_rate(40);
	while(ros::ok()){
		ros::spinOnce();
		if(!cloud->points.empty()){
			sensor_msgs::PointCloud2 cloud_obstacles_;
			pcl::toROSMsg(*cloud_obstacles, cloud_obstacles_);
			cloud_obstacles_.header.frame_id = cloud->header.frame_id;
			pub_cloud_obstacles.publish(cloud_obstacles_);
			
			sensor_msgs::PointCloud2 cloud_ground_;
			pcl::toROSMsg(*cloud_ground, cloud_ground_);
			cloud_ground_.header.frame_id = cloud->header.frame_id;
			pub_cloud_ground.publish(cloud_ground_);
		
			pub_grid.publish(grid);
		}
		loop_rate.sleep();
	}
}

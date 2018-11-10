#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int16MultiArray.h>

class OccupancyGridCombination{
	private:
		ros::NodeHandle nh;
		/*subscribe*/
		ros::Subscriber sub_grid_lidar;
		ros::Subscriber sub_grid_zed;
		ros::Subscriber sub_odom;
		ros::Subscriber sub_node;
		/*publish*/
		ros::Publisher pub;
		/*objects*/
		nav_msgs::OccupancyGrid grid;
		nav_msgs::OccupancyGrid grid_lidar;
		nav_msgs::OccupancyGrid grid_zed;
		/*flags*/
		bool first_callback_grid_lidar = true;
		bool first_callback_grid_zed = true;
		bool first_callback_odom = true;
		bool expand_grass = false;
		/*time*/
		ros::Time time_odom_now;
		ros::Time time_odom_last;
		double time_moving;
		double time_nomove;
		ros::Time time_pub;
		/*node numbers*/
		std::vector<int> nodes_park;
	public:
		OccupancyGridCombination();
		void CallbackGridLidar(const nav_msgs::OccupancyGridConstPtr& msg);
		void CallbackGridZed(const nav_msgs::OccupancyGridConstPtr& msg);
		void CallbackOdom(const nav_msgs::OdometryConstPtr& msg);
		void CallbackNode(const std_msgs::Int16MultiArrayConstPtr& msg);
		void CombineGrids(void);
		void AmbiguityFilter(void);
		void ExpandObstacles(void);
		void PartialExpandObstacle(void);
		void ShrinkObstacles(void);
		int PointToIndex(nav_msgs::OccupancyGrid grid, int x, int y);
		void IndexToPoint(nav_msgs::OccupancyGrid grid, int index, int& x, int& y);
		bool CellIsInside(nav_msgs::OccupancyGrid grid, int x, int y);
		void Publication(void);
};

OccupancyGridCombination::OccupancyGridCombination()
{
	sub_grid_lidar = nh.subscribe("/occupancygrid/lidar/stored", 1, &OccupancyGridCombination::CallbackGridLidar, this);
	sub_grid_zed = nh.subscribe("/occupancygrid/zed/stored", 1, &OccupancyGridCombination::CallbackGridZed, this);
	sub_odom = nh.subscribe("/tinypower/odom", 1, &OccupancyGridCombination::CallbackOdom, this);
	sub_node = nh.subscribe("/local_node", 1, &OccupancyGridCombination::CallbackNode, this);
	pub = nh.advertise<nav_msgs::OccupancyGrid>("/local_map",1);
	time_moving = 0.0;
	time_nomove = 0.0;
	nodes_park = {18, 19, 62, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 65};
}

void OccupancyGridCombination::CallbackGridLidar(const nav_msgs::OccupancyGridConstPtr& msg)
{
	grid_lidar = *msg;
	if(grid.data.empty())	grid = *msg;

	if(first_callback_grid_lidar && first_callback_grid_zed){
		grid_zed = *msg;
		for(size_t i=0;i<grid_zed.data.size();i++)	grid_zed.data[i] = -1;
	}
		
	first_callback_grid_lidar = false;
}

void OccupancyGridCombination::CallbackGridZed(const nav_msgs::OccupancyGridConstPtr& msg)
{
	grid_zed = *msg;
	if(grid.data.empty())	grid = *msg;
	
	if(first_callback_grid_lidar && first_callback_grid_zed){
		grid_lidar = *msg;
		for(size_t i=0;i<grid_lidar.data.size();i++)	grid_lidar.data[i] = -1;
	}

	first_callback_grid_zed = false;
}

void OccupancyGridCombination::CallbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	time_odom_now = ros::Time::now();
	double dt = (time_odom_now - time_odom_last).toSec();
	time_odom_last = time_odom_now;
	if(first_callback_odom)	dt = 0.0;

	time_pub = msg->header.stamp;

	if(msg->twist.twist.linear.x>1.0e-3){
		time_nomove = 0.0;
		time_moving += dt;
	}
	else{
		time_nomove += dt;
		time_moving = 0.0;
	}

	const double time_fullexpand = 3.0;	//[s]
	const double time_shrink = 3.0;	//[s]
	const double threshold_delay_zed = 10.0;	//[s]
	double delay_zed = (ros::Time::now() - grid_zed.header.stamp).toSec();
	if(!grid_lidar.data.empty() && !grid_zed.data.empty()){
		if(delay_zed>threshold_delay_zed){
			grid = grid_lidar;
			std::cout << "zed delay is too large(" << delay_zed << "[s])" << std::endl;
		}
		else	CombineGrids();
		AmbiguityFilter();
		if(time_moving>time_fullexpand)	ExpandObstacles();
		else if(time_nomove<time_shrink)	PartialExpandObstacle();
	}

	if(!grid.data.empty())	Publication();

	first_callback_odom = false;
}

void OccupancyGridCombination::CallbackNode(const std_msgs::Int16MultiArrayConstPtr& msg)
{
	size_t count = 0;
	for(size_t i=0;i<nodes_park.size();i++){
		if(msg->data[0]==nodes_park[i]){
			expand_grass = true;
			std::cout << "robot is in the park " << std::endl;
			break;
		}
		count++;
	}
	if(count==nodes_park.size())	expand_grass = false;
}

void OccupancyGridCombination::CombineGrids(void)
{
	const bool zed_has_higher_priority = true;
	if(zed_has_higher_priority){
		for(size_t i=0;i<grid.data.size();i++){
			if(grid_lidar.data[i]==100)	grid.data[i] = grid_lidar.data[i];
			else if(grid_zed.data[i]!=-1)	grid.data[i] = grid_zed.data[i];
			else if(grid_lidar.data[i]!=-1)	grid.data[i] = grid_lidar.data[i];
		}
	}
	/*equal priority*/
	else{
		for(size_t i=0;i<grid.data.size();i++){
			if(grid_lidar.data[i]==100)	grid.data[i] = grid_lidar.data[i];
			else if(grid_lidar.data[i]==0 || grid_zed.data[i]==0)	grid.data[i] = 0;
			else if(grid_lidar.data[i]!=-1)	grid.data[i] = grid_lidar.data[i];
			else if(grid_zed.data[i]!=-1)	grid.data[i] = grid_zed.data[i];
		}
	}
}

void OccupancyGridCombination::AmbiguityFilter(void)
{
	std::vector<int> indices_obs;

	const int range = 3;
	for(size_t i=0;i<grid.data.size();i++){
		if(grid.data[i]>0 && grid.data[i]<99){
			int x, y;
			IndexToPoint(grid, i, x, y);
			int count_zerocell = 0; 
			for(int j=-range;j<=range;j++){
				for(int k=-range;k<=range;k++){
					if(CellIsInside(grid, x+j, y+k) && grid.data[PointToIndex(grid, x+j, y+k)]==0)	count_zerocell++;
				}
			}
			int num_cells = (2*range + 1)*(2*range + 1);
			const double ratio = 0.725;
			double threshold = num_cells*ratio;
			if(count_zerocell>threshold)	grid.data[i] = 0;
		}
	}
}

void OccupancyGridCombination::ExpandObstacles(void)
{
	const int range = 2;
	/*expand only obstacles*/
	if(!expand_grass){
		for(size_t i=0;i<grid.data.size();i++){
			if(grid.data[i]==100){
				int x, y;
				IndexToPoint(grid, i, x, y);
				for(int j=-range;j<=range;j++){
					for(int k=-range;k<=range;k++){
						if(CellIsInside(grid, x+j, y+k) && grid.data[PointToIndex(grid, x+j, y+k)]!=grid.data[i])	grid.data[PointToIndex(grid, x+j, y+k)] = grid.data[i]-1;
					}
				}
			}
		}
	}
	/*expand obstacles and grass*/
	else{
		for(size_t i=0;i<grid.data.size();i++){
			if(grid.data[i]==100 || grid.data[i]==50){
				int x, y;
				IndexToPoint(grid, i, x, y);
				for(int j=-range;j<=range;j++){
					for(int k=-range;k<=range;k++){
						if(CellIsInside(grid, x+j, y+k) && grid.data[PointToIndex(grid, x+j, y+k)]<99)	grid.data[PointToIndex(grid, x+j, y+k)] = grid.data[i]-1;
					}
				}
			}
		}
	}
}

void OccupancyGridCombination::PartialExpandObstacle(void)
{
	const double range_no_expand = 2.0;	//[m]
	const int range = 2;
	if(!expand_grass){
		for(size_t i=0;i<grid.data.size();i++){
			if(grid.data[i]==100){
				int x, y;
				IndexToPoint(grid, i, x, y);
				double dist = sqrt(x*grid.info.resolution*x*grid.info.resolution + y*grid.info.resolution*y*grid.info.resolution);
				if(dist>range_no_expand){
					for(int j=-range;j<=range;j++){
						for(int k=-range;k<=range;k++){
							if(CellIsInside(grid, x+j, y+k) && grid.data[PointToIndex(grid, x+j, y+k)]!=grid.data[i])	grid.data[PointToIndex(grid, x+j, y+k)] = grid.data[i]-1;
						}
					}
				}
			}
		}
	}
	else{
		for(size_t i=0;i<grid.data.size();i++){
			if(grid.data[i]==100 || grid.data[i]==50){
				int x, y;
				IndexToPoint(grid, i, x, y);
				double dist = sqrt(x*grid.info.resolution*x*grid.info.resolution + y*grid.info.resolution*y*grid.info.resolution);
				if(dist>range_no_expand){
					for(int j=-range;j<=range;j++){
						for(int k=-range;k<=range;k++){
							if(CellIsInside(grid, x+j, y+k) && grid.data[PointToIndex(grid, x+j, y+k)]<99)	grid.data[PointToIndex(grid, x+j, y+k)] = grid.data[i]-1;
						}
					}
				}
			}
		}
	}
}

void OccupancyGridCombination::ShrinkObstacles(void)
{
	for(size_t i=0;i<grid.data.size();i++){
		if(grid.data[i]==99)	grid.data[i] = 0;
		if(grid.data[i]==49)	grid.data[i] = 0;
	}
}

int OccupancyGridCombination::PointToIndex(nav_msgs::OccupancyGrid grid, int x, int y)
{
	int x_ = x + grid.info.width/2.0;
	int y_ = y + grid.info.height/2.0;
	return	y_*grid.info.width + x_;
}

void OccupancyGridCombination::IndexToPoint(nav_msgs::OccupancyGrid grid, int index, int& x, int& y)
{
	x = index%grid.info.width - grid.info.width/2.0;
	y = index/grid.info.width - grid.info.height/2.0;
}

bool OccupancyGridCombination::CellIsInside(nav_msgs::OccupancyGrid grid, int x, int y)
{   
	int w = grid.info.width;
	int h = grid.info.height;
	if(x<-w/2.0)  return false;
	if(x>w/2.0-1) return false;
	if(y<-h/2.0)  return false;
	if(y>h/2.0-1) return false;
	return true;
}

void OccupancyGridCombination::Publication(void)
{
	grid.header.stamp = time_pub;
	pub.publish(grid);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "occupancygrid_combine");

	OccupancyGridCombination occupancygrid_combine;

	ros::spin();
}

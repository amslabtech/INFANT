
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>

ros::Time time_now_odom;
ros::Time time_last_odom;
nav_msgs::Odometry odom;
bool first_callback_odom = true;
double theta = 0.0;

void callback_odom(const nav_msgs::OdometryConstPtr& msg)
{
	// std::cout << "CALLBACK ODOM" << std::endl;
	// odom = *msg;
	
	time_now_odom = ros::Time::now();
	double dt = (time_now_odom - time_last_odom).toSec();
	time_last_odom = time_now_odom;
	if(first_callback_odom)	dt = 0.0;

	theta += msg->twist.twist.angular.z*dt;
	if(theta<-M_PI)   theta += 2*M_PI;
	if(theta>M_PI)    theta -= 2*M_PI;

	odom.header.stamp = msg->header.stamp;
	odom.pose.pose.position.x += msg->twist.twist.linear.x*dt * cos(theta);
	odom.pose.pose.position.y += msg->twist.twist.linear.x*dt * sin(theta);
	odom.pose.pose.position.z = 0.0;

	tf::Quaternion q = tf::createQuaternionFromRPY(0.0, 0.0, theta);
	quaternionTFToMsg(q, odom.pose.pose.orientation);

	first_callback_odom = false;
}

void tf_broadcaster_odom(void)
{
	// std::cout << "TF BROADCASTER ODOM" << std::endl;
	static tf::TransformBroadcaster broadcaster;
	geometry_msgs::TransformStamped transform;
	// transform.header.stamp = ros::Time::now();
	transform.header.stamp = odom.header.stamp;
	transform.header.frame_id = "/odom";
	transform.child_frame_id = "/velodyne";
	transform.transform.translation.x = odom.pose.pose.position.x;
	transform.transform.translation.y = odom.pose.pose.position.y;
	transform.transform.translation.z = odom.pose.pose.position.z;	
	transform.transform.rotation = odom.pose.pose.orientation;
	broadcaster.sendTransform(transform);
}

void tf_broadcaster_localmap(void)
{
	// std::cout << "TF BROADCASTER LOCALMAP" << std::endl;
	static tf::TransformBroadcaster broadcaster;
	geometry_msgs::TransformStamped transform;
	// transform.header.stamp = ros::Time::now();
	transform.header.stamp = odom.header.stamp;
	transform.header.frame_id = "/odom";
	transform.child_frame_id = "/localmap";
	transform.transform.translation.x = odom.pose.pose.position.x;
	transform.transform.translation.y = odom.pose.pose.position.y;
	transform.transform.translation.z = odom.pose.pose.position.z;
	transform.transform.rotation.x = 0.0;
	transform.transform.rotation.y = 0.0;
	transform.transform.rotation.z = 0.0;
	transform.transform.rotation.w = 1.0;
	broadcaster.sendTransform(transform);
}

void tf_broadcaster_zed(void)
{
	// std::cout << "TF BROADCASTER LOCALMAP" << std::endl;
	static tf::TransformBroadcaster broadcaster;
	geometry_msgs::TransformStamped transform;
	// transform.header.stamp = ros::Time::now();
	transform.header.stamp = odom.header.stamp;
	transform.header.frame_id = "/velodyne";
	transform.child_frame_id = "/zed";
	transform.transform.translation.x = 0.35;
	transform.transform.translation.y = 0.0;
	transform.transform.translation.z = -0.5;
	transform.transform.rotation.x = 0.0;
	transform.transform.rotation.y = 0.0;
	transform.transform.rotation.z = 0.0;
	transform.transform.rotation.w = 1.0;
	broadcaster.sendTransform(transform);
}

void initialize_odom(nav_msgs::Odometry& odom)
{
	odom.header.frame_id = "/odom";
	odom.child_frame_id = "/velodyne";
	odom.pose.pose.position.x = 0.0;
	odom.pose.pose.position.y = 0.0;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation.x = 0.0;
	odom.pose.pose.orientation.y = 0.0;
	odom.pose.pose.orientation.z = 0.0;
	odom.pose.pose.orientation.w = 1.0;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "tf_broadcaster");
	ros::NodeHandle nh;

	ros::Subscriber sub_odom = nh.subscribe("/tinypower/odom", 1, callback_odom);

	time_now_odom = ros::Time::now();
	time_last_odom = ros::Time::now();
	initialize_odom(odom);

	ros::Rate loop_rate(100);
	while(ros::ok()){
		ros::spinOnce();
		if(!first_callback_odom){
			tf_broadcaster_odom();
			tf_broadcaster_localmap();
			tf_broadcaster_zed();
		}
		loop_rate.sleep();
	}
}

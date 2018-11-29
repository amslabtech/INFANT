#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>

class TFBroadCaster{
	private:
		ros::NodeHandle nh;
		/*subscribe*/
		ros::Subscriber sub_odom;
		/*objects*/
		tf::TransformBroadcaster broadcaster;
		nav_msgs::Odometry odom;
		double theta;
		int count_same_odom;
		/*flags*/
		bool first_callback_odom = true;
		/*time*/
		ros::Time time_odom_now;
		ros::Time time_odom_last;
	public:
		TFBroadCaster();
		void MainRoop(void);
		void InitializeOdom(nav_msgs::Odometry& odom);
		void CallbackOdom(const nav_msgs::OdometryConstPtr& msg);
		void Broadcast1(void);
		void Broadcast2(void);
		void Broadcast3(void);
};

TFBroadCaster::TFBroadCaster()
{
	InitializeOdom(odom);
	sub_odom = nh.subscribe("/tinypower/odom", 1, &TFBroadCaster::CallbackOdom, this);
	theta = 0.0;
	count_same_odom = 0;
	MainRoop();
}

void TFBroadCaster::MainRoop(void)
{
	ros::Rate loop_rate(100);
	while(ros::ok()){
		ros::spinOnce();
		if(!first_callback_odom){
			Broadcast1();
			Broadcast2();
			Broadcast3();
		}
		loop_rate.sleep();
	}
}

void TFBroadCaster::InitializeOdom(nav_msgs::Odometry& odom)
{
	odom.header.frame_id = "/odom";
	odom.child_frame_id = "/velodyne_odom";
	odom.pose.pose.position.x = 0.0;
	odom.pose.pose.position.y = 0.0;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation.x = 0.0;
	odom.pose.pose.orientation.y = 0.0;
	odom.pose.pose.orientation.z = 0.0;
	odom.pose.pose.orientation.w = 1.0;
}

void TFBroadCaster::CallbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	bool encoder_works = true;
	if(msg->twist.twist.linear.x>1.0e-3 && fabs(msg->twist.twist.linear.x-odom.twist.twist.linear.x)<1.0e-3)	count_same_odom++;
	else	count_same_odom = 0;
	if(count_same_odom>5)	encoder_works = false;

	odom = *msg;

	time_odom_now = ros::Time::now();
	double dt = (time_odom_now - time_odom_last).toSec();
	time_odom_last = time_odom_now;
	if(first_callback_odom)	dt = 0.0;

	if(encoder_works){
		theta += msg->twist.twist.angular.z*dt;
		if(theta<-M_PI)   theta += 2*M_PI;
		if(theta>M_PI)    theta -= 2*M_PI;

		odom.header.stamp = msg->header.stamp;
		odom.pose.pose.position.x += msg->twist.twist.linear.x*dt * cos(theta);
		odom.pose.pose.position.y += msg->twist.twist.linear.x*dt * sin(theta);
		odom.pose.pose.position.z = 0.0;

		tf::Quaternion q = tf::createQuaternionFromRPY(0.0, 0.0, theta);
		quaternionTFToMsg(q, odom.pose.pose.orientation);
	}

	// Broadcast1();
	// Broadcast2();
	// Broadcast3();

	first_callback_odom = false;
}

void TFBroadCaster::Broadcast1(void)
{
	geometry_msgs::TransformStamped transform;
	transform.header.stamp = odom.header.stamp;
	transform.header.frame_id = "/odom";
	transform.child_frame_id = "/velodyne_odom";
	transform.transform.translation.x = odom.pose.pose.position.x;
	transform.transform.translation.y = odom.pose.pose.position.y;
	transform.transform.translation.z = odom.pose.pose.position.z;
	transform.transform.rotation = odom.pose.pose.orientation;
	broadcaster.sendTransform(transform);
}

void TFBroadCaster::Broadcast2(void)
{
	geometry_msgs::TransformStamped transform;
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

void TFBroadCaster::Broadcast3(void)
{
	geometry_msgs::TransformStamped transform;
	transform.header.stamp = odom.header.stamp;
	transform.header.frame_id = "/velodyne_odom";
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

int main(int argc, char** argv)
{
	ros::init(argc, argv, "tf_broadcaster");

	TFBroadCaster tf_broadcaster;

	// ros::spin();
}

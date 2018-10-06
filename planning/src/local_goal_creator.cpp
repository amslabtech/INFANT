#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#define HALF_PI M_PI*0.5
#define RATIO 1.5//0.46
#define ON_LINE 0.5
#define AVOIDang 0.087
using namespace std;

const string header_frame("/map");
const string robot_frame("/matching_base_link");

float n_dist=0, radius=0;;
bool avoid_flag=true;
bool callback_flag[4]={false, false, false, false};
geometry_msgs::PoseStamped t_wp, p_wp;
float calcDistance(geometry_msgs::Pose2D& A, geometry_msgs::PoseStamped& B)
{
	return hypot(fabs(A.x-B.pose.position.x), fabs(A.y-B.pose.position.y));
}
 
float calcDistance(geometry_msgs::Pose2D& A, geometry_msgs::Pose2D& B)
{
	return hypot(fabs(A.x-B.x), fabs(A.y-B.y));
}
 
//垂線の足の計算
geometry_msgs::Pose2D nearest(	geometry_msgs::Pose2D A, 
								geometry_msgs::Pose2D B, 
								geometry_msgs::Pose2D P )
{
	geometry_msgs::Pose2D a, b;
	double r;
	//a = (next wp - now wp),b = (robo pos - now_wp)
	a.x = B.x - A.x;
	a.y = B.y - A.y;
	b.x = P.x - A.x;
	b.y = P.y - A.y;
	
	r = (a.x*b.x + a.y*b.y) / (a.x*a.x + a.y*a.y);
	
	if( r<= 0 ){
		return A;
	}else if( r>=1 ){
		return B;
	}else{
		geometry_msgs::Pose2D result;
		result.x = A.x + r*a.x;
		result.y = A.y + r*a.y;
		return result;
	}
}

void findLocalGoal(geometry_msgs::PoseStamped& goal, geometry_msgs::PoseStamped& robo, ros::Publisher& pt_on_line_pub)
{
	geometry_msgs::Pose2D A, B, P, H, I;
	A.x = p_wp.pose.position.x;
	A.y = p_wp.pose.position.y;
	B.x = t_wp.pose.position.x;
	B.y = t_wp.pose.position.y;
	P.x = robo.pose.position.x;
	P.y = robo.pose.position.y;
	
	float d =  RATIO * n_dist;
	if (d<radius) d=radius;
	
	H = nearest(A, B, P);
	float phi = atan2(B.y-A.y, B.x-A.x); //slope of track line
	I.x = H.x + d*cos(phi);
	I.y = H.y + d*sin(phi);
	
	float theta;
	// I is more far from robot than B 	
	if (calcDistance(I, robo)>calcDistance(B, robo)){
		I=B;
		theta = atan2(n_dist, calcDistance(H, I));
		cout<<"over the next waypoint"<<endl;
		cout<<"theta:"<<theta<<endl;
	}
	else {
		theta = atan2(n_dist, d); //slope of robot and point on track-line
		cout<<"theta:"<<theta<<endl;
	}
	
	/////for debug////
	sensor_msgs::PointCloud I_msg;
	I_msg.points.resize(1);
	I_msg.points[0].x=I.x;
	I_msg.points[0].y=I.y;
	I_msg.points[0].z=1;
	I_msg.header.frame_id=header_frame;
	I_msg.header.stamp=ros::Time(0);
	pt_on_line_pub.publish(I_msg);
	// cout<<"H:"<<endl<<H<<", I:"<<endl<<I;
	/////////////////
	
	
	//calculate angleABP
	geometry_msgs::Pose2D AB, AP;
	AB.x = B.x-A.x;
	AB.y = B.y-A.y;
	AP.x = P.x-A.x; 
	AP.y = P.y-A.y; 
	if (AB.x*AP.y-AB.y*AP.x>0) theta = -theta; 
	float yaw = phi + theta;
	cout<<"Iang:"<<atan2(I.y, I.x)<<endl;
	cout<<"yaw:"<<yaw<<endl;
	
	if (!avoid_flag){
		goal.pose.position.x = robo.pose.position.x + radius*cos(yaw);
		goal.pose.position.y = robo.pose.position.y + radius*sin(yaw);
		goal.pose.position.z = 0;
		H = nearest(A, B, I);
		if (calcDistance(I, goal)<ON_LINE){
			cout<<"on line"<<endl;
			goal.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, phi);
		}
		else 
    		goal.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,yaw);
	}
	else{
		goal.pose.position.x = robo.pose.position.x + radius*cos(yaw-AVOIDang);//offset
		goal.pose.position.y = robo.pose.position.y + radius*sin(yaw-AVOIDang);
		goal.pose.position.z = 0;
    	goal.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,yaw-AVOIDang);
	}
	goal.header.frame_id = header_frame; 
	goal.header.stamp = ros::Time(0);
	
	
	// cout<<"A:"<<endl<<A<<", B:"<<endl<<B<<", P:"<<endl<<P;
	// cout<<"H:"<<endl<<H<<", I:"<<endl<<I;
	// cout<<"d:"<<d<<", theta:"<<theta<<", phi:"<<phi<<", yaw:"<<yaw<<endl;
	cout<<endl;
}

inline bool checkTF(tf::StampedTransform& transform, geometry_msgs::PoseStamped& robo, tf::TransformListener& listener)
{
	try{
		//listener.waitForTransform(header_frame, robot_frame, ros::Time(0), ros::Duration(1.0));
	    listener.lookupTransform(header_frame, robot_frame, ros::Time(0), transform);
        robo.pose.position.x=transform.getOrigin().x();
        robo.pose.position.y=transform.getOrigin().y();
        robo.pose.position.z=0;
        robo.pose.orientation.z = tf::getYaw(transform.getRotation());
        // float angle = tf::getYaw(transform.getRotation());
        // robo.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,angle);
		return true;
	}
	catch (tf::TransformException ex){
		// cout<<"waiting for global and robot frame relation"<<endl;
        ROS_ERROR("%s",ex.what());
		return false;
	}
}

void AvoidFlagCallback(const std_msgs::BoolConstPtr& msg)
{
	avoid_flag=true;
}

void TargetWpCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
	t_wp=*msg;
	callback_flag[0]=true;
}

void PreviousWpCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
	p_wp=*msg;
	callback_flag[1]=true;
}


void NormalDistCallback(const std_msgs::Float32ConstPtr& msg)
{
	n_dist=msg->data;
	callback_flag[2]=true;
}

void LocalPathRadiusCallback(const std_msgs::Float32ConstPtr& msg)
{
	radius=msg->data;
	callback_flag[3]=true;
}

void LocalGoalCreator()
{
	ros::NodeHandle n;
	ros::Subscriber twp_sub = n.subscribe("/target/pose", 1, TargetWpCallback);
	ros::Subscriber pre_sub = n.subscribe("/waypoint/prev", 1, PreviousWpCallback);
	ros::Subscriber norm_dist_sub = n.subscribe("/waypoint/normal_dist", 1, NormalDistCallback);
	ros::Subscriber radius_sub = n.subscribe("/control/r", 1, LocalPathRadiusCallback);
	ros::Subscriber avoid_flag_sub = n.subscribe("/avoid_flag", 1, AvoidFlagCallback);
	ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("/local_goal", 1);
	//for debug//
	ros::Publisher pt_on_line_pub = n.advertise<sensor_msgs::PointCloud>("pt_on_line", 1);
	
    tf::StampedTransform transform;
    tf::TransformListener listener;
	geometry_msgs::PoseStamped robo, goal;

	
	ros::Rate loop_rate(10);
	while(ros::ok()){
		//get robot position
		bool tf_flag = checkTF(transform, robo, listener);
		if (tf_flag && callback_flag[0] && callback_flag[1] && callback_flag[2] && callback_flag[3]){
			findLocalGoal(goal, robo, pt_on_line_pub);
			goal_pub.publish(goal);

			//callback_flag[0]=false;
			//callback_flag[1]=false;
			//callback_flag[2]=false;
			//callback_flag[3]=false;
			avoid_flag=false;
		}
		
		ros::spinOnce();
		loop_rate.sleep();
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "local_goal_creator");
	LocalGoalCreator();
	
	ROS_INFO("Killing now!!!!!");
	return 0;
}

#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <math.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <trajectory_generation/Velocity.h>
#include <trajectory_generation/VelocityArray.h>
#include <trajectory_generation/TrajectoryGeneration.h>

#define HALF_PI M_PI*0.5
#define RATIO 1.5//0.46
#define ON_LINE 0.5
#define AVOIDang 0.087
using namespace std;

const string header_frame("/map");
const string robot_frame("/matching_base_link");
const string lg_frame("/local_goal");

float n_dist=0, radius=0;;
bool avoid_flag=true;
bool check_nodes_flag=false;//到達判定
// bool callback_flag[2]={false, false};
bool radius_callback_flag=false;
nav_msgs::OccupancyGrid grid_map;
nav_msgs::OccupancyGrid path_map;
bool map_callback_flag=false;
bool path_map_flag=false;
sensor_msgs::Joy joy_in;
// bool joy_flag=false;
float theta=0;
//----red_line----//
float Arr[200];
int arr_size=0;
bool redline_flag=false;
const int stock=5;
const int R=3; //刻み幅
const int Line_size=360/R; // 360/5
int save_line[stock][Line_size+1]={0};
const int judge_line=3;
int line_count=0;
int Arr_x=0;
//----------------//
//----road deg----//
float Road[200];
const int T=3;
const int Road_T=360/T;
int save_road[Road_T]={0};
int road_size=0;
bool road_flag=false;
//----------------//
bool gyro_flag=false;
double robot_yaw;
int goal_r=10;
std_msgs::Bool insec;
bool insec_flag=false;
bool insec_flag2=false;
bool insec_flag3=false;
bool re_route_flag=false;
float pre_yaw=0;
int joy_cnt=0;
std_msgs::Bool turn_ok;
const int map_y=200;
double track=0;
double kari_track=0;
double end_turn_angle=0;
bool end_turn_flag=false;
std_msgs::Bool end_turn;
int track_stock[120]={0}; //360/3
double track_cnt=0;
double real_robot_yaw=0;
std_msgs::Float64 reset_yaw;
bool reset_yaw_flag=false;
float linear_vel=0;
std_msgs::Float32MultiArray l_xy;
std_msgs::Bool end_navi_flag;
std_msgs::Int16 which_turn;
int local_start=0;
int local_goal=0;
bool local_node_flag=false;
int yaw_track=0;//1026
bool track_flag=false;//1026

void findLocalGoal(geometry_msgs::PoseStamped& goal, geometry_msgs::PoseStamped& robo, nav_msgs::OccupancyGrid& grid, sensor_msgs::Joy joy, float t_yaw, double track)
{
	double kari_yaw=100;
	double go_yaw=100;
	for(int i=0;i<Arr_x;i++){
		// if(fabs(kari_yaw)>fabs(Arr[i])){
		if(fabs(track-kari_yaw-real_robot_yaw)>fabs(track-Arr[i]-real_robot_yaw)){
			kari_yaw=Arr[i];
		}
	}
	if((fabs(track-kari_yaw-real_robot_yaw)>20*M_PI/180.0)||Arr_x==0){
		// if(fabs(kari_yaw)>45*M_PI/180.0){
		// kari_yaw=0;
		kari_yaw=-(real_robot_yaw-track);
		cout<<"run track"<<endl;
	}

	for(int j=0;j<road_size;j++){
		if(fabs(go_yaw-kari_yaw)>fabs(Road[j]-kari_yaw)){
			go_yaw=Road[j];
		}
	}

	// cout<<"go_yaw = "<<go_yaw<<"red_line = "<<kari_yaw<<endl;
	
	// if(!insec_flag){
	if((!insec_flag)&&(!insec_flag2)){
		if(fabs(go_yaw-pre_yaw)>5*M_PI/180.0){ //道のぶれ
			if(go_yaw>pre_yaw){
				go_yaw=pre_yaw+M_PI/180.0;
				cout<<"to left"<<endl;
			}else{
				go_yaw=pre_yaw-M_PI/180.0;
				cout<<"to right"<<endl;
			}
		}
	}

	if(end_turn_flag){
		go_yaw=0;
		end_turn_flag=false;
	}

	pre_yaw=go_yaw;

	goal.pose.position.x=goal_r*cos(go_yaw);
	goal.pose.position.y=goal_r*sin(go_yaw);
	goal.pose.position.z=0;
	theta=atan2(goal.pose.position.y, goal.pose.position.x);
	goal.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(0,0,theta);
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

void LocalPathRadiusCallback(const std_msgs::Float32ConstPtr& msg)
{
	radius=msg->data;
	// callback_flag[3]=true;
	radius_callback_flag=true;
}

void CheckNodesCallback(const std_msgs::BoolConstPtr& msg)
{
	if(msg->data){
		check_nodes_flag=true;
	}
}

void LocalMapCallback(const nav_msgs::OccupancyGridConstPtr& msg)
{
	grid_map=*msg;
	// grid_map.data.resize(133*133);
	map_callback_flag=true;
}

void PathMapCallback(const nav_msgs::OccupancyGridConstPtr& msg)
{
	path_map=*msg;
	// grid_map.data.resize(133*133);
	path_map_flag=true;
}


void RedLineCallback(const std_msgs::Int32MultiArrayConstPtr& msg)
{
	arr_size=msg->layout.data_offset;
	int Arr_count=0;
	int j=0;
	Arr_x=0;
	if(line_count>4){
		line_count=0;
	}
	if(arr_size!=100){
		for(int hh=0;hh<Line_size;hh++){
			save_line[line_count][hh]=0;
		}
		for(int h=0;h<arr_size;h++){
			 j=(msg->data[h]/2.0-180)/R;
			j=(msg->data[h]/2.0)/R;
			if(j>Line_size){
				j-=Line_size;
			}
			save_line[line_count][j]=1;
		}
		for(int l=0;l<Line_size;l++){
			Arr_count=0;
			for(int k=0;k<stock;k++){
				Arr_count+=save_line[k][l];
			}
			if(Arr_count>=judge_line){  //3回以上道があれば
				Arr[Arr_x]=(l*R*M_PI/180.0)-M_PI; //robot座標
				Arr[Arr_x]=(l*R*M_PI/180.0)-M_PI; //robot座標
				
				Arr_x++;
			}
		}
	}
	line_count++;
	redline_flag=true;
}

void RoaddegCallback(const std_msgs::Int32MultiArrayConstPtr& msg)
{
	road_size=msg->layout.data_offset;
	int road_r=0;
	if(road_size!=100){
		for(int h=0;h<Road_T;h++){
			save_road[h]=0;
		}
		for(int j=0;j<road_size;j++){
			// road_r=(msg->data[j]/2.0-180);
			road_r=(msg->data[j]/2.0-180)/T; //3度刻み
			road_r*=T;
			if(road_r>360){
				road_r-=360;
			}
			Road[j]=road_r*M_PI/180.0;
		}
	}
	
	road_flag=true;
}

void GyroCallback(const nav_msgs::OdometryConstPtr& msg)
{
	robot_yaw=msg->pose.pose.orientation.z;
	// cout<<"------------------------------------"<<endl;
	// cout<<robot_yaw<<endl;
	// cout<<"------------------------------------"<<endl;
	while(fabs(robot_yaw)>M_PI){
		if(robot_yaw>M_PI){
			robot_yaw-=2*M_PI;
		}else if(robot_yaw<-M_PI){
			robot_yaw+=2*M_PI;
		}
	}

	gyro_flag=true;
}

void EndturnCallback(const std_msgs::BoolConstPtr& msg)
{
	end_turn=*msg;
	if(end_turn.data){
		end_turn_flag=true;
	}
}

void ResetyawCallback(const std_msgs::Float64ConstPtr& msg)
{
	reset_yaw.data=0;
	reset_yaw=*msg;
	if(reset_yaw.data){
		reset_yaw_flag=true;
	}
}

void VelCallback(const std_msgs::Float32ConstPtr& msg)
{
	linear_vel=msg->data;
}

void LocalNodeCallback(const std_msgs::Int16MultiArrayConstPtr& msg)
{
	if((local_start!=msg->data[0])||(local_goal!=msg->data[1])){
		local_node_flag=true;
	}
	local_start=msg->data[0];
	local_goal=msg->data[1];
	if(msg->layout.data_offset==3){
		if(msg->data[2]==0){
			insec_flag=true;
		}
		if(msg->data[2]==1){
			insec_flag2=true;
		}
		if(msg->data[2]==2){
			insec_flag3=true;
		}
		if(msg->data[2]==3){
			re_route_flag=true;
		}

	}
}

vector<string> split(const string &str, char sep)
{
	vector<string> v;
	stringstream ss(str);
	string buffer;
	while( getline(ss, buffer, sep) ) {
		v.push_back(buffer);
	}
	return v;
}


void LocalGoalCreator()
{
	ros::NodeHandle n;
	ros::Subscriber radius_sub = n.subscribe("/control/r", 1, LocalPathRadiusCallback);
	ros::Subscriber avoid_flag_sub = n.subscribe("/avoid_flag", 1, AvoidFlagCallback);
	ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("/local_goal", 1);
	ros::Subscriber check_sub = n.subscribe("/check_nodes",1, CheckNodesCallback);
	//for debug//
	ros::Publisher pt_on_line_pub = n.advertise<sensor_msgs::PointCloud>("pt_on_line", 1);
	ros::Subscriber map_sub = n.subscribe("/local_map_real/expand",1, LocalMapCallback);
	ros::Subscriber path_map_sub = n.subscribe("/local_map_real",1, PathMapCallback);

	ros::Subscriber red_line_sub = n.subscribe("/peak/deg",1,RedLineCallback);
	
	ros::Subscriber road_deg_sub = n.subscribe("/road/deg",1,RoaddegCallback);

	ros::Subscriber gyro_sub = n.subscribe("/lcl",1,GyroCallback);
	

	ros::Publisher robo_t_pub = n.advertise<std_msgs::Float32>("/robo_t_angle",1);
	ros::Publisher turn_pub = n.advertise<std_msgs::Bool>("/turn_ok_flag",1);

	ros::Subscriber end_turn_sub = n.subscribe<std_msgs::Bool>("/end_turn",1,EndturnCallback);

	ros::Subscriber reset_yaw_sub = n.subscribe<std_msgs::Float64>("/flag/diff",1,ResetyawCallback);

	ros::Subscriber vel_sub = n.subscribe<std_msgs::Float32>("/vel_yaw",1,VelCallback);

	ros::Publisher l_xy_pub = n.advertise<std_msgs::Float32MultiArray>("/next_xy",1);

	ros::Publisher end_navi_pub = n.advertise<std_msgs::Bool>("/end_navi",1);
	
	ros::Publisher which_turn_pub = n.advertise<std_msgs::Int16>("/which_turn",1);
	ros::Subscriber local_node_sub = n.subscribe<std_msgs::Int16MultiArray>("/local_node",1,LocalNodeCallback);

	FILE *fp1;
	// fp1=fopen("./tsukuba.csv","r"); //tsukuba
	fp1=fopen("./ikuta.csv","r"); //ikuta
	char vore[300][10];
	float ne_posi[300][3];
	int j=0;
	if(fp1==NULL){
		cout<<"can't open csv"<<endl;
		exit(1);
	}
	while(fscanf(fp1,"%s %f %f %f",vore[j],&ne_posi[j][0],&ne_posi[j][1],&ne_posi[j][2])!=EOF){
		cout<<vore[j]<<" "<<ne_posi[j][0]<<" "<<ne_posi[j][1]<<" "<<ne_posi[j][2]<<endl;
		j++;
	}

	//---------次のnodeの方位----------//
	float target_yaw=-1.624;//生田矢印
	// float target_yaw=2.08814;//tsukubaスターと位置
	// float target_yaw=-2.38517;//3ばん
	// float target_yaw=-1.12985;//6ばん
	// float target_yaw=2.09479;//8ばん
	// float target_yaw=-2.64025;//19ばん
	// float target_yaw=-1.3749;//22ばん
	// float target_yaw=-0.08;
	// float target_yaw=-1.66761;
	int error_cnt=0;
	int j_cnt=0;
	tf::StampedTransform transform;
    tf::TransformListener listener;
	geometry_msgs::PoseStamped robo, goal;

	goal.pose.position.x=0;
	goal.pose.position.y=0;
	goal.pose.position.z=0;
	goal.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
	
	std_msgs::Float32 robo_t_yaw;
	/////////////sawa////////////////
	tf::TransformBroadcaster br;
	tf::Transform lg_transform;
	goal.header.frame_id=robot_frame;

	ros::Rate loop_rate(10);
	int init_cnt=0;

	which_turn.data=100;
	bool restart_flag=false;
	while(ros::ok()){
		//get robot position
		bool tf_flag=false;
		check_nodes_flag=true;
		if(check_nodes_flag){
			// bool tf_flag = checkTF(transform, robo, listener);
			tf_flag = checkTF(transform, robo, listener);
			check_nodes_flag=false;
		}
		// cout<<"tf_flag"<<tf_flag<<endl;
		turn_ok.data=false;
		j_cnt=0;
		if(local_node_flag){
			if(insec_flag||insec_flag2||insec_flag3||re_route_flag){
				error_cnt=0;
				// cout<<path_node[s_cnt]<<" "<<path_node[g_cnt]<<endl;
				l_xy.data.clear();
				restart_flag=false;
				while(j_cnt<sizeof(vore)){
					if((strcmp(vore[j_cnt],"VERTEX")==0)&&(ne_posi[j_cnt][0]==local_goal)){
						l_xy.data.push_back(ne_posi[j_cnt][1]);
						l_xy.data.push_back(ne_posi[j_cnt][2]);
						l_xy.data.push_back(ne_posi[j_cnt][0]);
						if(insec_flag){
							l_xy.data.push_back(0);
							which_turn.data=0;
						}
						if(insec_flag2){
							l_xy.data.push_back(1);
							which_turn.data=1;
						}
						if(insec_flag3){
							l_xy.data.push_back(2);
							which_turn.data=2;
						}
						if(re_route_flag){
							l_xy.data.push_back(3);
							which_turn.data=1;
						}

						// l_xy_pub.publish(l_xy);
						// insec_cnt++;
					}
					if((strcmp(vore[j_cnt],"EDGE")==0)&&(ne_posi[j_cnt][0]==local_start)&&(ne_posi[j_cnt][1]==local_goal)){
						target_yaw=ne_posi[j_cnt][2];
						error_cnt++;
						// cout<<path_node[s_cnt]<<" "<<path_node[g_cnt]<<endl;
						// cout<<j_cnt<<" "<<target_yaw<<endl;
						break;
					}
					j_cnt++;
					if(j<j_cnt){
						break;
					}
				}
				if(!error_cnt){
					j_cnt=0;
					while(j_cnt<sizeof(vore)){
						if((strcmp(vore[j_cnt],"EDGE")==0)&&(ne_posi[j_cnt][1]==local_start)&&(ne_posi[j_cnt][0]==local_goal)){
							if(ne_posi[j_cnt][2]>=0){
								target_yaw=ne_posi[j_cnt][2]-M_PI;
								error_cnt++;
								break;
							}else{
								target_yaw=ne_posi[j_cnt][2]+M_PI;
								error_cnt++;
								break;
							}
						}
						j_cnt++;
						if(j<j_cnt){
							break;
						}
					}
				}
				if(!error_cnt){
					cout<<local_start<<" "<<local_goal<<endl;
					cout<<"can't find edge"<<endl;
					exit(1);
				}
				robo_t_yaw.data=target_yaw-robot_yaw;
				if(robo_t_yaw.data>M_PI){
					robo_t_yaw.data-=2*M_PI;
				}
				if(robo_t_yaw.data<-M_PI){
					robo_t_yaw.data+=2*M_PI;
				}
				robo_t_pub.publish(robo_t_yaw); //turn
				which_turn_pub.publish(which_turn);
				turn_ok.data=true;
				// turn_pub.publish(turn_ok);
				insec_flag=false;
				insec_flag2=false;
				insec_flag3=false;
				re_route_flag=false;
				which_turn.data=100;
				l_xy_pub.publish(l_xy);
			}
		}
		
		if(joy_cnt>0){
			joy_cnt++;
		}
		if(joy_cnt>20){
			joy_cnt=0;
		}
		// cout<<tf_flag<<" "<<radius_callback_flag<<" "<<map_callback_flag<<" "<<path_map_flag<<" "<<redline_flag<<" "<<gyro_flag<<endl;
		cout<<"------------------------------------------------"<<endl;
		cout<<"start_node = "<<local_start<<" goal_node = "<<local_goal<<endl;
		cout<<"target_yaw = "<<target_yaw<<" robot_yaw = "<<robot_yaw<<endl;
		// cout<<"cnt="<<cnt<<endl;
		robo_t_yaw.data=target_yaw-robot_yaw;
		if(robo_t_yaw.data>M_PI){
			robo_t_yaw.data-=2*M_PI;
		}
		if(robo_t_yaw.data<-M_PI){
			robo_t_yaw.data+=2*M_PI;
		}
		robo_t_pub.publish(robo_t_yaw); //turn
		turn_pub.publish(turn_ok);
		// cout<<robo_t_yaw.data<<endl;

		cout<<tf_flag<<radius_callback_flag<<map_callback_flag<<path_map_flag<<redline_flag<<gyro_flag<<road_flag<<local_node_flag<<endl;
		if (tf_flag && radius_callback_flag && map_callback_flag && path_map_flag && redline_flag && gyro_flag && road_flag && local_node_flag){
		// if (tf_flag && radius_callback_flag && map_callback_flag){
			if(init_cnt==0){
				end_turn_angle=robot_yaw;
				init_cnt++;
			}
			if(end_turn_flag){
				if((local_start!=5)||(local_start!=6)){//つくちゃれ
					end_turn_angle=robot_yaw;
					// end_turn_flag=false;
					yaw_track=0;
					track_flag=false;
					track=0;
					track_cnt=0;
					for(int ww=0;ww<120;ww++){
						track_stock[ww]=0;
					}
				}
			}
			int track_sum=0;
			if(track_flag){//1026
				kari_track=0;
				if(robot_yaw-end_turn_angle>M_PI){
					kari_track=robot_yaw-end_turn_angle-2*M_PI;
					// track+=robot_yaw-end_turn_angle-2*M_PI;
				}else if(robot_yaw-end_turn_angle<-M_PI){
					kari_track=robot_yaw-end_turn_angle+2*M_PI;
					// track+=robot_yaw-end_turn_angle+2*M_PI;
				}else{
					kari_track=robot_yaw-end_turn_angle;
					// track+=robot_yaw-end_turn_angle;
				}
				track_cnt++;
				// track_stock[int(track*180/3.0/M_PI)+180/3]+=1;
				track_stock[int(kari_track*180/3.0/M_PI)+180/3]+=1;
				for(int xx=0;xx<120;xx++){
					track_sum+=track_stock[xx]*(xx-180/3)*3*M_PI/180.0;
				}
			}//1026
			real_robot_yaw=robot_yaw-end_turn_angle;
			while(fabs(real_robot_yaw)>M_PI){
				if(real_robot_yaw>M_PI){
					real_robot_yaw-=2*M_PI;
				}
				if(real_robot_yaw<-M_PI){
					real_robot_yaw+=2*M_PI;
				}
				// cout<<"bbbbbbb"<<endl;
			}
			if(track_flag){//1026
				track=track_sum/track_cnt;
			}//1026
			//1026//
			if(yaw_track<50){
				yaw_track++;
			}else{
				track_flag=true;
			}
			////////
			if(reset_yaw_flag){
				end_turn_angle+=reset_yaw.data;
				track+=reset_yaw.data;
				while(fabs(end_turn_angle)>M_PI){
					if(end_turn_angle>M_PI){
						end_turn_angle-=2*M_PI;
					}
					if(end_turn_angle<-M_PI){
						end_turn_angle+=2*M_PI;
					}
				}
				reset_yaw_flag=false;
			}

			// cout<<"track = "<<track<<"real_robot_yaw = "<<real_robot_yaw<<endl;

			findLocalGoal(goal, robo, grid_map, joy_in, target_yaw, track);
			//////////////////////sawa//////////////////
			lg_transform.setOrigin(tf::Vector3(goal.pose.position.x, goal.pose.position.y, 0.0));
			tf::Quaternion q;
			q.setRPY(0, 0, theta);
			lg_transform.setRotation(q);
			br.sendTransform(tf::StampedTransform(lg_transform, ros::Time::now(), robot_frame, lg_frame));
			goal_pub.publish(goal);

			avoid_flag=false;
			map_callback_flag=false;
			path_map_flag=false;
			redline_flag=false;
			gyro_flag=false;
			road_flag=false;
			end_turn_flag=false;
		}
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	fclose(fp1);
	// fclose(fp2);

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_local_goal");
	LocalGoalCreator();
	
	ROS_INFO("Killing now!!!!!");
	return 0;
}

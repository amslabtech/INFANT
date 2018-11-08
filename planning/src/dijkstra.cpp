#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <fstream>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Joy.h>

using namespace std;

#define INF 1000000000

// std_msgs::Int16MultiArray g_path;
std_msgs::Int16MultiArray target_node;
int estimate_start;
int estimate_goal;
bool edge_flag=false;
std_msgs::Bool insec;
bool insec_flag=false;
bool insec_flag2=false;
bool insec_flag3=false;
int joy_cnt=0;
sensor_msgs::Joy joy_in;
float pre_x=0;
float pre_y=0;
float ne_x=0;
float ne_y=0;
float estimate_ratio=0;
bool re_route_flag=false;
bool first_dijkstra=true;
int re_route_cnt=0;
int pre_start;
int pre_goal;
bool first_edge_flag=true;
int get_flag = 0;

void EdgeCallback(const std_msgs::Int16MultiArrayConstPtr& msg)
{
    // if(re_route_cnt==0){
        // re_route_cnt=1;
    estimate_start=msg->data[0];
    estimate_goal=msg->data[1];
    estimate_ratio=(msg->data[2])/100.0;
    edge_flag=true;
    // }
    if(first_edge_flag){
        pre_start=estimate_start;
        pre_goal=estimate_goal;
        first_edge_flag=false;
    }
}

void IntersectionCallback(const std_msgs::BoolConstPtr& msg)
{
    insec=*msg;
    if(insec.data){
		get_flag += 1;
	}
	if(!insec.data && get_flag >0){
        insec_flag=true;
		get_flag = 0;
    }
}

void JoyCallback(const sensor_msgs::JoyConstPtr& msg)
{
    joy_in=*msg;
    if(joy_in.buttons[13]==1){
        if(joy_cnt==0){
            insec_flag2=true;
            joy_cnt=1;
        }
    }
    if(joy_in.buttons[14]==1){
        if(joy_cnt==0){
            insec_flag3=true;
            joy_cnt=1;
        }
    }
    // if(joy_in.buttons[1]==1){
    //  if(joy_cnt==0){
    //      edge_flag=true;
    //      // estimate_start++;
    //      // estimate_goal++;
    //      estimate_ratio=0.6;
    //      joy_cnt=1;
    //  }
    // }
}

void GlobalPathCreator()
{
    ros::NodeHandle n;

    ros::Subscriber edge_sub = n.subscribe("/edge/certain",1,EdgeCallback);
    ros::Subscriber intersection_sub = n.subscribe("/intersection_flag",1,IntersectionCallback);
    ros::Subscriber joy_sub = n.subscribe("/joy",1,JoyCallback);
    ros::Publisher node_pub = n.advertise<std_msgs::Int16MultiArray>("/local_node",1);
    
    FILE *fp1;
    FILE *fp2;
    // fp1=fopen("./tsukuba.csv","r"); //tsukuba
    fp1=fopen("/home/amsl/ros_catkin_ws/src/INFANT/planning/src/ikuta.csv","r"); //ikuta
    //fp1=fopen("/home/amsl/ros_catkin_ws/src/mapping/latlng2xy/xy/xy_new.csv","r"); //ikuta
    //fp1=fopen("/home/amsl/ros_catkin_ws/src/INFANT/planning/src/xy_new.csv","r"); //ikuta
    //fp2=fopen("/home/amsl/ros_catkin_ws/src/INFANT/planning/src/dijkstra_new.path","r");
    fp2=fopen("/home/amsl/ros_catkin_ws/src/INFANT/planning/src/dijkstra.path","r");
    char VorE[300][10];
    float NE_posi[300][4];
    float Dijkstra[300][3];
    int i=0;
    int j=0;
    int E_cnt=0;
    int N_cnt=0;
    int size_NE=0;
    //VERTEX,NODE読み込み//
    if(fp1==NULL){
        cout<<"can't open csv"<<endl;
        exit(1);
    }
    while(fscanf(fp1,"%s %f %f %f",VorE[i],&NE_posi[i][0],&NE_posi[i][1],&NE_posi[i][2])!=EOF){
        if(strcmp(VorE[i],"VERTEX")==0){
            N_cnt++;
        }
        if(strcmp(VorE[i],"EDGE")==0){
            E_cnt++;
        }
        i++;
    }
    size_NE=i;
    ///////////////////////
    //コスト設定//
    while(j!=i){
        float x1=0, x2=0, y1=0, y2=0;
        int cnt=0;
        if(strcmp(VorE[j],"EDGE")==0){
            for(int k=0;k<j;k++){
                if((strcmp(VorE[k],"VERTEX")==0)&&(NE_posi[j][0]==NE_posi[k][0])){
                    x1=NE_posi[k][1];
                    y1=NE_posi[k][2];
                    cnt++;
                }
                if((strcmp(VorE[k],"VERTEX")==0)&&(NE_posi[j][1]==NE_posi[k][0])){
                    x2=NE_posi[k][1];
                    y2=NE_posi[k][2];
                    cnt++;
                }
                if(cnt==2){
                    cnt=0;
                    break;
                }
            }
            NE_posi[j][3]=sqrt(pow((x2-x1),2)+pow((y2-y1),2));
        }
        j++;
    }
    ////////////
    bool start_dijkstra=true;
    int start_cnt=0;
    int goal_cnt=1;
    int Global_path[200][2];
    int Original_path[200][2];
    int route_node[200];
    int path_cnt=0;
    int original_cnt=0;
    bool restart_flag=false;
    int global_cnt=0;
    for(int a=0;a<200;a++){
        route_node[a]=500;
    }
    //スタート，ゴール設定//
    if(fp2==NULL){
        cout<<"can't open dijkstra.path"<<endl;
        exit(1);
    }
    int rd=0;
    while(fscanf(fp2,"%d",&route_node[rd])!=EOF){
        rd++;
    }

    // ros::Rate loop_rate(10);
    ros::Rate loop_rate(20);
    while(ros::ok()){
        if(start_dijkstra){
            start_cnt=0;
            goal_cnt=1;
            // for(int q=0;q<20;q++){
            //  if(route_node[q]!=500){
            //      route_cnt++;
            //  }
            // }
            ///////////////////////
            // if(size_NE>i){
            //  re_route_flag=true;
            // }
            int dijkstra_cnt=0;
            for(int b=0;b<200;b++){
                Global_path[b][0]=500;
                Global_path[b][1]=0;
            }
            ////////////////////////
            global_cnt=0;
            while(route_node[dijkstra_cnt+1]!=500)
            {

            /////////初期化///////////
                int ii=0;
                while(ii<300){
                    Dijkstra[ii][0]=INF;
                    Dijkstra[ii][1]=INF;
                    Dijkstra[ii][2]=0;
                    ii++;
                }
                Dijkstra[route_node[dijkstra_cnt]][0]=0;//start cost=0;
                Dijkstra[route_node[dijkstra_cnt]][1]=route_node[dijkstra_cnt];
                Dijkstra[route_node[dijkstra_cnt]][2]=1;//0=未確定,1=確定,2=確定&探索済み
                // cout<<"check"<<Dijkstra[route_node[dijkstra_cnt]][0]<<" "<<Dijkstra[route_node[dijkstra_cnt]][1]<<" "<<Dijkstra[route_node[dijkstra_cnt]][2]<<endl;
                int target=route_node[dijkstra_cnt];
            //////////////////////////
/////////////////////////////////////////////////////// 
                while(1)
                {
                    int d_cnt=0;
                    // cout<<"i="<<i<<"size_NE="<<size_NE<<endl;
                    // cout<<"/////////////////"<<endl;
                    // cout<<"target="<<target<<endl;
                    while(d_cnt<size_NE+1)
                    {
                        // cout<<NE_posi[d_cnt][0]<<" "<<Dijkstra[int(NE_posi[d_cnt][1])][2]<<endl;
                        if((strcmp(VorE[d_cnt],"EDGE")==0)&&(NE_posi[d_cnt][0]==target)&&(Dijkstra[int(NE_posi[d_cnt][1])][2]==0)){

                            if((Dijkstra[target][0]+NE_posi[d_cnt][3])<Dijkstra[int(NE_posi[d_cnt][1])][0]){
                                Dijkstra[int(NE_posi[d_cnt][1])][0]=Dijkstra[target][0]+NE_posi[d_cnt][3];//コスト
                                Dijkstra[int(NE_posi[d_cnt][1])][1]=target;
                            }
                        }
                        if((strcmp(VorE[d_cnt],"EDGE")==0)&&(NE_posi[d_cnt][1]==target)&&(Dijkstra[int(NE_posi[d_cnt][0])][2]==0)){

                            if((Dijkstra[target][0]+NE_posi[d_cnt][3])<Dijkstra[int(NE_posi[d_cnt][0])][0]){
                                Dijkstra[int(NE_posi[d_cnt][0])][0]=Dijkstra[target][0]+NE_posi[d_cnt][3];//コスト
                                Dijkstra[int(NE_posi[d_cnt][0])][1]=target;
                            }
                        }
                        d_cnt++;
                    }

                    Dijkstra[target][2]=2;
                    float min_cost=INF;
                    int kari_detection=0;
                    for(int w=0;w<N_cnt;w++){
                        if(Dijkstra[w][2]==0&&Dijkstra[w][0]<min_cost){
                            kari_detection=w;
                            min_cost=Dijkstra[w][0];
                        }
                    }
                    Dijkstra[kari_detection][2]=1;
                    target=kari_detection;
                    if(Dijkstra[route_node[dijkstra_cnt+1]][2]==1){//goalが確定したら                   
                        cout<<"goal"<<endl;
                        break;
                    }
                    // cout<<"target="<<target<<endl;
                    // cout<<"size_NE="<<size_NE<<endl;
                }

                int kari_path[100];
                int kari_end=route_node[dijkstra_cnt+1];
                int swap_cnt=0;
                while(1){
                    for(int x=0;x<N_cnt+1;x++){
                        if(x==kari_end){
                            kari_path[swap_cnt]=x;
                            kari_end=int(Dijkstra[x][1]);
                            swap_cnt++;
                        }
                    }
                    if(kari_path[swap_cnt-1]==route_node[dijkstra_cnt]){
                        break;
                    }
                }
                for(int xx=0;xx<swap_cnt;xx++){
                    if(global_cnt==0){
                        Global_path[global_cnt][0]=kari_path[swap_cnt-xx-1];
                        Global_path[global_cnt][1]=1;//絶対通る
                        global_cnt++;
                        path_cnt++;
                    }
                    if(Global_path[global_cnt-1][0]!=kari_path[swap_cnt-xx-1]){
                        Global_path[global_cnt][0]=kari_path[swap_cnt-xx-1];
                        if(xx==swap_cnt-1){
                            Global_path[global_cnt][1]=1;
                        }
                        global_cnt++;
                        path_cnt++;
                    }
                }
                /////////////////////////////////////////////
                // for(int c=0;c<path_cnt;c++){
                //  for(int d=0;d<route_cnt;d++){
                //      if(Global_path[c][0]==route_node[d]){
                //          Global_path[c][1]=1;
                //      }
                //  }
                // }
                //////////////////////////////////////////////

                dijkstra_cnt++;
            }
            size_NE=N_cnt+E_cnt;

            if(first_dijkstra){
                for(int f=0;f<path_cnt;f++){
                    Original_path[f][0]=Global_path[f][0];
                    Original_path[f][1]=Global_path[f][1];
                    cout<<Global_path[f][0]<<" "<<Global_path[f][1]<<endl;
                }
                original_cnt=path_cnt;
            }

            if(!first_dijkstra){
                re_route_flag=true;
            }
////////////////////////////////////////////////
        
        }

        first_dijkstra=false;
        start_dijkstra=false;
        if(insec_flag||insec_flag2||insec_flag3){
            if(goal_cnt==path_cnt-1){
                for(int ori=0;ori<200;ori++){
                    Global_path[ori][0]=500;
                    Global_path[ori][1]=0;
                }
                for(int origin=0;origin<original_cnt;origin++){
                    Global_path[origin][0]=Original_path[origin][0];
                    Global_path[origin][1]=Original_path[origin][1];
                }
                start_cnt=0;
                goal_cnt=1;
                restart_flag=true;
            }
            else if((insec_flag||insec_flag2)&&!restart_flag){
                start_cnt++;
                goal_cnt++;
            }
            else if(insec_flag3&&!restart_flag){
                if(start_cnt==0){
                    start_cnt=path_cnt-2;
                    goal_cnt=path_cnt-1;
                }else{
                    start_cnt-=1;
                    goal_cnt-=1;
                }
            }
        }

        if(joy_cnt>0){
            joy_cnt++;
        }
        if(joy_cnt>20){
            joy_cnt=0;
        }
        // if(re_route_cnt>0){
        //  re_route_cnt++;
        // }
        // if(re_route_cnt>20){
        //  re_route_cnt=0;
        // }

        if(edge_flag){
            if(((estimate_start==Global_path[start_cnt][0])&&(estimate_goal==Global_path[goal_cnt][0]))||Global_path[start_cnt][0]==N_cnt){
                if(Global_path[start_cnt][1]!=100){
                    Global_path[start_cnt][1]=100;
                }
            }
            if((pre_start==estimate_start)&&(pre_goal==estimate_goal)){
                re_route_cnt++;
            }else{
                re_route_cnt=0;
            }
            edge_flag=false;
        }
        if(re_route_cnt>100){
            edge_flag=true;
            re_route_cnt=0;
        }
        pre_start=estimate_start;
        pre_goal=estimate_goal;

        // cout<<"N_cnt="<<N_cnt<<endl;
        // if(Global_path[0][0]==N_cnt){
        //  int check_path=0;
        //  while(Global_path[check_path+1][0]!=500)
        //  {
        //      Global_path[check_path][0]=Global_path[check_path+1][0];
        //      Global_path[check_path][1]=Global_path[check_path+1][1];
        //      check_path++;
        //  }
        //  cout<<"qqqqqqqqqqqqqqqqqqq"<<endl;
        // }

        if(edge_flag){
                            
            // int debug_cnt=0;
            // bool debug_flag=false;
            // while(debug_cnt<N_cnt){
            //  if(NE_posi[debug_cnt][0]==Global_path[start_cnt][0]){
            //      debug_flag=true;
            //  }
            //  debug_cnt++;
            // }
            // if(!debug_flag){
            //  if(Global_path[goal_cnt][0]==estimate_start){
            //      Global_path[0][0]=estimate_goal;
            //  }else{
            //      Global_path[0][0]=estimate_start;
            //  }
            // }

            // Global_path[0][1]=100;//debug reroute
            if(((estimate_start==Global_path[start_cnt][0])&&(estimate_goal==Global_path[goal_cnt][0]))||Global_path[start_cnt][0]==N_cnt){
                if(Global_path[start_cnt][1]!=100){
                    Global_path[start_cnt][1]=100;
                    for(int g=0;g<path_cnt;g++){
                        cout<<Global_path[g][0]<<" "<<Global_path[g][1]<<endl;
                    }
                }
            }
            else if(((estimate_start!=Global_path[start_cnt][0])||(estimate_goal!=Global_path[goal_cnt][0]))){
                cout<<"start dijkstra"<<endl;
                start_dijkstra=true;
                path_cnt=0;
                ///////////初期化//////////
                int new_cnt=0;
                float kari_d=0;
                int size_cnt=0;
                while(size_cnt<300){
                    if(strcmp(VorE[new_cnt],"VERTEX")==0){
                        if(NE_posi[new_cnt][0]==estimate_start){
                            pre_x=NE_posi[new_cnt][1];
                            pre_y=NE_posi[new_cnt][2];
                        }
                        if(NE_posi[new_cnt][0]==estimate_goal){
                            ne_x=NE_posi[new_cnt][1];
                            ne_y=NE_posi[new_cnt][2];
                        }
                    }
                    if(strcmp(VorE[new_cnt],"EDGE")==0){
                        if((NE_posi[new_cnt][0]==estimate_start)&&(NE_posi[new_cnt][1]==estimate_goal)){
                            kari_d=NE_posi[new_cnt][2];
                        }
                        else if((NE_posi[new_cnt][0]==estimate_goal)&&(NE_posi[new_cnt][1]==estimate_start)){
                            kari_d=NE_posi[new_cnt][2];
                        }

                    }
                    new_cnt++;
                    size_cnt++;
                }

                size_cnt=0;
                int cost_cnt=0;
                float cost1=0;
                float cost2=0;
                //ノード追加/////////////////////////////
                NE_posi[size_NE][0]=N_cnt;//ノード番号
                NE_posi[size_NE][1]=(estimate_ratio*ne_x)+((1.0-estimate_ratio)*pre_x);//ノードx
                NE_posi[size_NE][2]=(estimate_ratio*ne_y)+((1.0-estimate_ratio)*pre_y);//ノードy
                strcpy(VorE[size_NE],"VERTEX");
                while(size_cnt<300){
                    if(strcmp(VorE[cost_cnt],"EDGE")==0){
                        if((NE_posi[cost_cnt][0]==estimate_start)&&(NE_posi[cost_cnt][1]==estimate_goal)){
                            cost1=estimate_ratio*NE_posi[cost_cnt][3];
                            cost2=(1.0-estimate_ratio)*NE_posi[cost_cnt][3];
                        }
                        else if((NE_posi[cost_cnt][0]==estimate_goal)&&(NE_posi[cost_cnt][1]==estimate_start)){
                            cost1=estimate_ratio*NE_posi[cost_cnt][3];
                            cost2=(1.0-estimate_ratio)*NE_posi[cost_cnt][3];
                        }
                    }
                    cost_cnt++;
                    size_cnt++;
                }

                // cout<<NE_posi[size_NE][0]<<" "<<NE_posi[size_NE][1]<<" "<<NE_posi[size_NE][2]<<" "<<VorE[size_NE]<<endl;

                /////////////////////////////////////////
                size_NE++;
                //エッジ追加/////////////////////////////
                NE_posi[size_NE][0]=estimate_start;
                NE_posi[size_NE][1]=N_cnt;
                NE_posi[size_NE][2]=kari_d;
                NE_posi[size_NE][3]=cost1;
                strcpy(VorE[size_NE],"EDGE");
                // cout<<NE_posi[size_NE][0]<<" "<<NE_posi[size_NE][1]<<" "<<NE_posi[size_NE][2]<<" "<<VorE[size_NE]<<endl;

                size_NE++;
                NE_posi[size_NE][0]=N_cnt;
                NE_posi[size_NE][1]=estimate_goal;
                NE_posi[size_NE][2]=kari_d;
                NE_posi[size_NE][3]=cost2;
                strcpy(VorE[size_NE],"EDGE");
                /////////////////////////////////////////
                // cout<<NE_posi[size_NE][0]<<" "<<NE_posi[size_NE][1]<<" "<<NE_posi[size_NE][2]<<" "<<VorE[size_NE]<<endl;

                // size_NE=N_cnt+E_cnt;

                for(int a=0;a<20;a++){
                    route_node[a]=500;
                }
                int re_route=0;
                int re=1;
                route_node[0]=N_cnt;
                while(re_route<global_cnt){
                    if(Global_path[re_route][1]==1){
                        route_node[re]=Global_path[re_route][0];
                        // cout<<route_node[re];
                        re++;
                    }
                    re_route++;
                }
                for(int ee=0;ee<re;ee++){
                    cout<<"route_node"<<route_node[ee]<<endl;
                }

                cout<<"Dijkstra"<<endl;
            }
            edge_flag=false;
        }
    
        target_node.data.clear();
        int yaw_cnt=0;
        bool yaw_flag=false;
        while(yaw_cnt<N_cnt){
            if(NE_posi[yaw_cnt][0]==Global_path[start_cnt][0]){
                yaw_flag=true;
            }
            yaw_cnt++;
        }
        Global_path[0][1]=100;
		// cout<<"yaw_flag="<<yaw_flag<<endl;
		//cout<<"start_cnt"<<start_cnt<<endl;
		// cout<<"goal_cnt"<<goal_cnt<<endl;
		// cout<<"global_path[start]"<<Global_path[start_cnt][1]<<endl;
		// cout<<"global_path[goal]"<<Global_path[goal_cnt][0]<<endl;
        if(!yaw_flag){
            if(Global_path[goal_cnt][0]==estimate_start){
                cout<<"aaaaaaaaaaa"<<endl;
                Global_path[start_cnt][0]=estimate_goal;
                // target_node.data.push_back(estimate_goal);
                // target_node.data.push_back(estimate_start);
                target_node.data.push_back(Global_path[start_cnt][0]);
                target_node.data.push_back(Global_path[goal_cnt][0]);
            }else{
                cout<<"bbbbbbbbbbbb"<<endl;
                Global_path[start_cnt][0]=estimate_start;
                // target_node.data.push_back(estimate_start);
                // target_node.data.push_back(estimate_goal);
                target_node.data.push_back(Global_path[start_cnt][0]);
                target_node.data.push_back(Global_path[goal_cnt][0]);
            }
        }
        if(yaw_flag){
            target_node.data.push_back(Global_path[start_cnt][0]);
            target_node.data.push_back(Global_path[goal_cnt][0]);
        }
        if(insec_flag){
            target_node.data.push_back(0);
            target_node.layout.data_offset=3;
        }
        if(insec_flag2){
            target_node.data.push_back(1);
            target_node.layout.data_offset=3;
        }
        if(insec_flag3){
            target_node.data.push_back(2);
            target_node.layout.data_offset=3;
        }
        if(re_route_flag){
        cout<<"/////////////////////////"<<start_cnt<<" "<<goal_cnt<<endl;
        for(int e=0;e<path_cnt;e++){
            cout<<Global_path[e][0]<<" "<<Global_path[e][1]<<endl;
        }

            target_node.data.push_back(3);
            target_node.layout.data_offset=3;
        }

        if(!insec_flag&&!insec_flag2&&!insec_flag3&&!re_route_flag){
            target_node.layout.data_offset=2;
        }
        insec_flag=false;
        insec_flag2=false;
        insec_flag3=false;
        restart_flag=false;
        re_route_flag=false;

        // cout<<target_node<<endl;
        // cout<<Global_path[start_cnt][1]<<endl;
        node_pub.publish(target_node);
        // cout<<"/////////////////////////"<<endl;
        // for(int e=0;e<path_cnt;e++){
        //  cout<<Global_path[e][0]<<" "<<Global_path[e][1]<<endl;
        // }
        // cout<<"start_cnt="<<start_cnt<<endl;
        ros::spinOnce();
        loop_rate.sleep();
    }

    fclose(fp1);
    fclose(fp2);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dijkstra");

    GlobalPathCreator();

    return 0;
}



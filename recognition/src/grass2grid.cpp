#include <ros/ros.h>
#include <limits>
#include <sensor_msgs/Image.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.h>

sensor_msgs::Image seg_img;
sensor_msgs::Image depth_img;
nav_msgs::OccupancyGrid map;
pcl::PointCloud<pcl::PointXYZ> p;
pcl::PointCloud<pcl::PointXYZ>::Ptr p_g (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr p_r (new pcl::PointCloud<pcl::PointXYZ>);

const int pixel_num_x = 513;
const int pixel_num_y = 288;
bool seg_flag = false;
int grid[400][400] = {-1};


struct Pixel{
    int seg[3];
    //bool seg_ok;
    char seg_g_or_r;
    float depth;
    float rad_x;
    float rad_y;
};

Pixel pixel[pixel_num_y][pixel_num_x];

cv::Mat image_s;
cv::Mat image_d;

bool im_s_s = false;
void seg_callback(const sensor_msgs::ImageConstPtr& msg)
{
    seg_img = *msg;
    image_s = cv_bridge::toCvCopy(seg_img, sensor_msgs::image_encodings::BGR8)->image;
    //resize(image_s,image_s,cv::Size(),320/pixel_num_x.,180/pixel_num_y.);
    
    //cv::imshow("image_s",image_s);
    //if(!im_s_s){
    //    cv::moveWindow("image_s",60,40);
    //    im_s_s = true;
    //}

    //printf("image_s[160,160][B,G,R] = [%d,%d,%d]\n"
    //         ,image_s.at<cv::Vec3b>(160,160)[0],image_s.at<cv::Vec3b>(160,160)[1],image_s.at<cv::Vec3b>(160,160)[2]);
    //printf("height = %d\n",image_s.height);
    //printf("width = %d\n",image_s.width);

    for(int y=0;y<pixel_num_y;y++){
        for(int x=0;x<pixel_num_x;x++){
            for(int c=0;c<3;c++){
                pixel[y][x].seg[c] = image_s.at<cv::Vec3b>(y,x)[c];
            }
        }
    }
    for(int y=0;y<pixel_num_y;y++){
        for(int x=0;x<pixel_num_x;x++){
            //if( (pixel[y][x].seg[0]==0) && (pixel[y][x].seg[1]==0) && ((pixel[y][x].seg[2]==0) || pixel[y][x].seg[2]==128)) pixel[y][x].seg_ok=true;
            //else pixel[y][x].seg_ok=false;
            //if( (pixel[y][x].seg[0]==0) && (pixel[y][x].seg[1]==0) && ((pixel[y][x].seg[2]==64) || pixel[y][x].seg[2]==192)){ pixel[y][x].seg_ok=false;
            //else pixel[y][x].seg_ok=true;
            
            if( (pixel[y][x].seg[0]==0) && (pixel[y][x].seg[1]==0) && ((pixel[y][x].seg[2]==64) || pixel[y][x].seg[2]==192))pixel[y][x].seg_g_or_r = 'g';
            else if( (pixel[y][x].seg[0]==0) && (pixel[y][x].seg[1]==0) && ((pixel[y][x].seg[2]==0) || pixel[y][x].seg[2]==128))pixel[y][x].seg_g_or_r = 'r';
            
        }
    }


    seg_flag = true;
    cv::waitKey(1);
    //[B,G,R]=[0,0,128],[0,0,0]
}



bool im_d_s = false;
void depth_callback(const sensor_msgs::ImageConstPtr& msg)
{
    depth_img = *msg;

    map.header = depth_img.header;

    image_d = cv_bridge::toCvCopy(depth_img, sensor_msgs::image_encodings::TYPE_32FC1)->image;
    resize(image_d,image_d,cv::Size(),pixel_num_x/320.,pixel_num_y/180.);

    //cv::imshow("image_d",image_d);
    //if(!im_d_s){
    //    cv::moveWindow("image_d",640,40);
    //    im_d_s = true;
    //}

    //printf("image_d[90,160] = %f\n",image_d.at<float>(90,160));//(y,x)
    //printf("height = %d\n",depth_img.height);
    //printf("width = %d\n",depth_img.width);

    for(int y=0;y<pixel_num_y;y++){
        for(int x=0;x<pixel_num_x;x++){
            pixel[y][x].depth = image_d.at<float>(y,x);
        }
    }
    cv::waitKey(1);
}

float deg2rad(float deg)
{
    float rad = deg*M_PI/180;
    return rad;
}

void store_angle(float angle_x,float angle_y)
{
    float dx = angle_x/pixel_num_x;
    float dy = angle_y/pixel_num_y;
	float modify_angle_y = 10;
    float init_x = angle_x/2;
    float init_y = angle_y/2;
    for(int x=0;x<pixel_num_x;x++){
        for(int y=0;y<pixel_num_y;y++){
            pixel[y][x].rad_x = deg2rad(dx*x - init_x);
            pixel[y][x].rad_y = deg2rad(dy*y - init_y - modify_angle_y);
        }
    }
    //printf("rad[0][0] = (%.2f, %.2f)\n",pixel[0][0].rad_x,pixel[0][0].rad_y);
    //printf("rad[287][512] = (%.2f, %.2f)\n",pixel[287][512].rad_x,pixel[287][512].rad_y);
}

void init_grid(void)
{
    for(int y=0;y<map.info.height;y++){
        for(int x=0;x<map.info.width;x++){
            grid[y][x] = -1;
        }
    }
}

void store_mapdata(void)
{
    int count = 0;
    for(int x=map.info.width-1;x>=0;x--){
        for(int y=map.info.height-1;y>=0;y--){
            //printf("%d\n",y*w+x);
            map.data[count] = grid[y][x];
            count++;
        }
    }
}

void calc_object(void)
{
    
    for(int y=pixel_num_y/2;y<pixel_num_y;y++){
        for(int x=0;x<pixel_num_x;x++){
            if(!std::isnan(pixel[y][x].depth) && pixel[y][x].depth <= 15.0){
                int ob_x = (pixel[y][x].depth*cos(pixel[y][x].rad_y)*sin(pixel[y][x].rad_x) / map.info.resolution);
                int ob_y = (pixel[y][x].depth*cos(pixel[y][x].rad_y)*cos(pixel[y][x].rad_x) / map.info.resolution);
                float ob_z = -(pixel[y][x].depth*sin(pixel[y][x].rad_y) );

                if(ob_y<=map.info.width/2.0){
                    if(map.info.width/2.0>=-ob_x){
                        if(ob_x<=map.info.width/2.0){
                            //if(!pixel[y][x].seg_ok){
                            //    grid[(int)map.info.height/2 - ob_y][(int)map.info.width/2 + ob_x] = 100;
                            //    grid[(int)map.info.height/2 - ob_y+1][(int)map.info.width/2 + ob_x] = 100;
                            //    grid[(int)map.info.height/2 - ob_y+2][(int)map.info.width/2 + ob_x] = 100;
                            //    
                            //    pcl::PointXYZ pt(ob_y*map.info.resolution,-ob_x*map.info.resolution,0);
                            //    p_g->points.push_back(pt);
                            //}else {
                            //    grid[(int)map.info.height/2 - ob_y][(int)map.info.width/2 + ob_x] = 0;
                            //    grid[(int)map.info.height/2 - ob_y+1][(int)map.info.width/2 + ob_x] = 0;
                            //    grid[(int)map.info.height/2 - ob_y+2][(int)map.info.width/2 + ob_x] = 0;
                            //}
                            if(pixel[y][x].seg_g_or_r == 'g'){
                                grid[(int)map.info.height/2 - ob_y][(int)map.info.width/2 + ob_x] = 100;
                                grid[(int)map.info.height/2 - ob_y+1][(int)map.info.width/2 + ob_x] = 100;
                                grid[(int)map.info.height/2 - ob_y+2][(int)map.info.width/2 + ob_x] = 100;
                                
                                pcl::PointXYZ pt(ob_y*map.info.resolution,-ob_x*map.info.resolution,ob_z);
                                p_g->points.push_back(pt);
                            }else if(pixel[y][x].seg_g_or_r == 'r'){
                                grid[(int)map.info.height/2 - ob_y][(int)map.info.width/2 + ob_x] = 0;
                                grid[(int)map.info.height/2 - ob_y+1][(int)map.info.width/2 + ob_x] = 0;
                                grid[(int)map.info.height/2 - ob_y+2][(int)map.info.width/2 + ob_x] = 0;
                                
                                pcl::PointXYZ pt(ob_y*map.info.resolution,-ob_x*map.info.resolution,ob_z);
                                p_r->points.push_back(pt);
                            }
                        }
                    }
                }
            }
        }
    }
}

void pubPoints(ros::Publisher& pub, pcl::PointCloud<pcl::PointXYZ>& pcl_in)
{
    sensor_msgs::PointCloud2 ros_out;
    pcl::toROSMsg(pcl_in, ros_out);
    ros_out.header.frame_id = "/zed";
    ros_out.header.stamp = ros::Time::now();
    pub.publish(ros_out);
    pcl_in.points.clear();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "grass2grid");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);

    ros::Subscriber seg_sub = nh.subscribe("/deeplab/image", 100, seg_callback);
    image_transport::Subscriber depth_sub = it.subscribe("/camera/depth/resized_image", 100, depth_callback);

    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 100, true);
    ros::Publisher pc_g_pub = nh.advertise<sensor_msgs::PointCloud2>("/zed_grasspoints", 100, true);
    ros::Publisher pc_r_pub = nh.advertise<sensor_msgs::PointCloud2>("/zed_roadpoints", 100, true);

    map.header.frame_id = "zed_depth_camera";
    map.info.resolution = 0.050000;
    map.info.width = 400;
    map.info.height = 400;
    map.info.origin.position.x = -10 ;
    map.info.origin.position.y = -10 ;
    map.info.origin.position.z = 0 ;
    map.info.origin.orientation.x = 0 ;
    map.info.origin.orientation.y = 0 ;
    map.info.origin.orientation.z = 0 ;
    map.info.origin.orientation.w = 0 ;
    map.data.resize(map.info.width * map.info.height);

    ros::Rate loop_rate(10);

    // store_angle(102.5, 70.0);
    store_angle(120, 70.0);

    while(ros::ok()){
        if(seg_flag){
            init_grid();

            calc_object();
            seg_flag = false;

            store_mapdata();
            
            
            pubPoints(pc_g_pub, *p_g); 
            pubPoints(pc_r_pub, *p_r); 
            
            
            map_pub.publish(map);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

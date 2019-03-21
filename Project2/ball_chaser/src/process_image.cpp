#include<iostream>
#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core.hpp>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>


using namespace std;
using namespace cv;

// Define a global client that can request services
ros::ServiceClient client;
//Define a variable that controls the direction of rotation of the cart when the target is not found
static int count=0;
static int ball_flag=1;
// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ROS_INFO_STREAM("Driving the robot");
    ball_chaser::DriveToTarget drive_cmd;
    drive_cmd.request.linear_x = lin_x;
    drive_cmd.request.angular_z = ang_z;
    
    if (!client.call(drive_cmd)){
        ROS_ERROR("Failed to call service");
    }
}


float image_center(cv::Mat img)
{
    
    if(img.empty())
    { 
        return 2;
    }
    int LeftHalfNum=0;
    int RightHalfNum=0;
    int i,j;
    cv::Mat img_out;
    cv::cvtColor(img, img_out, CV_RGB2GRAY);  
    threshold(img_out, img_out, 250, 255.0, CV_THRESH_BINARY);
    //位于相机左半部分的白球
    for(i=0;i<img_out.cols/2;i++)
    {
        for(j=0;j<img_out.rows;j++)
        {
            if(img_out.at<uchar>(j,i)==255)
                 LeftHalfNum++;
        }
    }
    //位于相机右半部分的白球
    for(i=img_out.cols/2;i<img_out.cols;i++)
    {
        for(j=0;j<img_out.rows;j++)
        {
            if(img_out.at<uchar>(j,i)==255)
                 RightHalfNum++;
        }
    }
    //ROS_INFO("Left:%i, Right:%i", LeftHalfNum, RightHalfNum);
    if(LeftHalfNum>RightHalfNum)
         ball_flag=1;
    else if(LeftHalfNum<RightHalfNum)
         ball_flag=-1;
    //没有找到白球
    if(LeftHalfNum==RightHalfNum)
     {
         if(LeftHalfNum==0)
              return 0;
     }
     //if(LeftHalfNum+RightHalfNum>=img_out.rows*img_out.cols/5*4)
       //  return 2;
    // 返回比率，归一化
    return (LeftHalfNum-RightHalfNum)/(LeftHalfNum+RightHalfNum);
}


// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
   float ball_members;
   float x,z;
   cv_bridge::CvImagePtr cv_ptr;
   //将ROS图像转化为opencv支持的Mat类型
   try
   {
      cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
   }
   catch (cv_bridge::Exception& e)
   {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
   }
    ball_members=image_center(cv_ptr->image);
    //没有找到球
    if(ball_members==0)
    {
       x = 0.0;
       z = 0.2*ball_flag;
    }
    //图片加载失败
    else if(ball_members==2)
    {
       x= 0.0;
       z= 0.0;
    }
    //当球位于正前方，直行
    else if(ball_members<0.05&&ball_members>-0.05)
    {
       x=0.1;
       z=0.0;
    }
    //能看见球，球不在正前方
    else
    {
       z=0.5*ball_members/800;
       x=0.1;
    }
    drive_robot(x,z);
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}

#include "ros/ros.h"
#include <iostream>
#include <std_msgs/UInt8.h>
#include <cstdio>
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;

float intrinsic[3][3] = {226.700732, 0.000000, 320.050533, 0.000000, 228.263884, 180.115765, 0, 0, 1};
float distortion[1][5] = {-0.197666, 0.027916, -0.000747, 0.001263, 0.000000};

int main(int argc, char **argv)
{
    /* code for main function */
    ros::init(argc, argv, "camera");
    ros::NodeHandle node;
    image_transport::ImageTransport it(node);
    image_transport::Publisher image_pub = it.advertise("camera/image_raw",1);
    
    ros::Rate loop_rate(10);
    Mat frame;
    Mat image_resized;
    VideoCapture cap(0);

    while(!cap.isOpened() && ros::ok())
    {
        ROS_INFO("Cannot connect to camera......");
        ros::spinOnce();
        loop_rate.sleep();
    }

    Mat cameraMatrix = Mat(3,3,CV_32FC1,intrinsic);
    Mat distCoeffs = Mat(1,5,CV_32FC1,distortion);
    Mat R = Mat::eye(3, 3, CV_32F);
    Mat mapx = Mat(Size(640,360), CV_32FC1);
    Mat mapy = Mat(Size(640,360), CV_32FC1);
    initUndistortRectifyMap(cameraMatrix, distCoeffs, R, cameraMatrix, Size(640,360), CV_32FC1, mapx, mapy);

    while (ros::ok())
    {
        cap>>frame;
        resize(frame, image_resized, Size(640,360));
   
        Mat image_rect = image_resized.clone();
        remap(image_resized, image_rect, mapx, mapy, INTER_LINEAR);

        ROS_INFO("%d  %d", image_rect.cols, image_rect.rows);
        sensor_msgs::ImagePtr msg;
        if(!frame.empty())
        {
            //imshow("source",image_resized);
            //waitKey(1);
            msg  =cv_bridge::CvImage(std_msgs::Header(),"bgr8",image_rect).toImageMsg();
            image_pub.publish(msg);
        }

    	//printf("afterrosok\n");
    ros::spinOnce();	
    loop_rate.sleep();
    }

    return 0;
}
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>
#include "sensor_msgs/Image.h"

cv::SimpleBlobDetector detector;



void imageRawCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  ROS_INFO("Received image with size: %i x %i", msg->width, msg->height);
}








int main(int argc, char **argv)
{
    ros::init(argc, argv, "ball_detection");
    ros::NodeHandle n;



    ros::Subscriber sub_init = n.subscribe("/usb_cam/image_raw", 1000, imageRawCallback);



    ROS_INFO("Hello world");

    ros::spin();
    return 0;
}
/*
 * \file vision.cpp
 * \author Gary Ellingson
 *
 * \brief This is the implementation file for the Vision class
 */

#include "vision.h"
#include <sensor_msgs/image_encodings.h>

namespace vision
{

Vision::Vision():
    nh_(ros::NodeHandle()),
    nh_private_(ros::NodeHandle("~"))
{
    nh_private_.param<int>("camera1", device1, 0);
    nh_private_.param<int>("camera2", device2, 1);

    if(!v1.open(device1) )// || !v2.open(device2))
        ROS_ERROR("could not open one or both cameras");

    image_publisher_ = nh_.advertise<sensor_msgs::Image>("image", 1);

}

Vision::~Vision()
{
    v1.release(); v2.release();
}

void Vision::publish_image()
{
//    sensor_msgs::Image image;
    cv::Mat frame1, frame2;

    cv_bridge::CvImage cv_img;

    if(v1.isOpened())
    {
        v1 >> frame1;
        cv_img.image = frame1;
        cv_img.encoding = sensor_msgs::image_encodings::BGR8;
        image_publisher_.publish(cv_img.toImageMsg());
    }
    else if(v2.isOpened())
    {
        v2 >> frame2;
        cv_img.image = frame2;
        cv_img.encoding = sensor_msgs::image_encodings::BGR8;
        image_publisher_.publish(cv_img.toImageMsg());
    }
}

} //namespace vision


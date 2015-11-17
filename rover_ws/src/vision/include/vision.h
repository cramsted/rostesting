#ifndef VISION_H
#define VISION_H

#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

namespace vision
{

class Vision
{
public:
    Vision();
    ~Vision();

    void publish_image();

protected:
    // ros stuff for this node
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Publisher image_publisher_;

    int device1, device2;
    cv::VideoCapture v1, v2;

};

} //end namespace

#endif //VISION_H

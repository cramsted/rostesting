#include "ros/ros.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char **argv)
{
    cv::Mat frame;
    cv::VideoCapture video(0); // get a camera object

    int key = 0;
//    cv::namedWindow("Image window", CV_WINDOW_AUTOSIZE);
//    cv::namedWindow("Image window 2", CV_WINDOW_AUTOSIZE);
    ros::init(argc, argv, "keyframe_devide");
    ros::NodeHandle n;

    while(key != 'q')
    {
        key = cv::waitKey(1);
        video >> frame;

        imshow("output", frame);
    }
    //ros::spin();

    return 0;
}

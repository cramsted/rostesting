#ifndef NAV_GUI
#define NAV_GUI

#define EARTH_RADIUS 6378145.0

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "rover_msgs/NavGoal.h"
#include "rover_msgs/NavState.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

namespace nav_gui
{

class NavGUI
{
public:
    NavGUI();
    ~NavGUI();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber nav_goal_subscriber_;
    ros::Subscriber nav_state_subscriber_;
    ros::Subscriber image_subscriber_;
    ros::Publisher gui_image_publisher_;

    std::string nav_goal_topic_;
    std::string nav_state_topic_;
    std::string image_topic_;
    std::string gui_image_topic_;

    //callbacks
    void navGoalCallback(const rover_msgs::NavGoal &msg);
    void navStateCallback(const rover_msgs::NavState &msg);
    void imageCallback(const sensor_msgs::Image &msg);

    rover_msgs::NavGoal goal_;
    rover_msgs::NavState state_;
};
} //namespace

#endif

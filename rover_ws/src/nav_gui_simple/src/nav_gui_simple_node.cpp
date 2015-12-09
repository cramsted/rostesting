#include <ros/ros.h>
#include "nav_gui.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nav_gui_simple");
    ros::NodeHandle nh;

    nav_gui::NavGUI ng;

    ros::spin();
}

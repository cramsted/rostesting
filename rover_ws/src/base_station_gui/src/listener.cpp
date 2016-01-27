#include "ros/ros.h"
#include "gui.h"
#include "std_msgs/String.h"

//state variables

int main(int argc, char **argv){
  //init the subscriber and send recieved data to the
  //callback function
  ros::init(argc, argv, "bs_listener");
  ros::NodeHandle n;
  GUI::GUI gui;
  ros::Subscriber sub = n.subscribe("rover", 1000, gui.roverCallback);
  ros::spin();
}

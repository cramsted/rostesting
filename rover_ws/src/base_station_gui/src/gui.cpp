#include "ros/ros.h"

GUI::GUI(){
  float[3] position = {0,0,0};
  float Vw = 0;
  float theta = 0;
  float psi = 0;
  float chi = 0;
  float p = 0;
  float q = 0;
  float r = 0;
  float Vg = 0;
  double base_latitude = 0;
  double base_altitude = 0;
  double base_longitude = 0;
}

GUI::~GUI(){}

void GUI::roverCallback(const std_msgs::String::ConstPtr& msg){
  ROS_INFO("message recieved");
  position = msg->data.position;
  Vw = msg->data.Vw;
  phi = msg->data.phi;
  theta = msg->data.psi;
  chi = msg->data.chi;
  p = msg->data.p;
  q = msg->data.q;
  r = msg->data.r;
  Vg = msg->data.Vg;
  base_altitude = msg->data.base_altitude;
  base_latitude = msg->data.base_latitude;
  base_longitude = msg->data.base_longitude;
}

double get_latitude(){
  return base_latitude;
}

double get_altitude(){
  return base_altitude;
}

double get_longitude(){
  return base_longitude;
}

float get_magnitometer(){
  return psi;
}

float get_ground_speed(){
  return Vg; 
}

#ifndef GUI_H
#define GUI_H

#include <ros/ros.h>

namespace gui{

  class GUI{
  private:
    float[3] position;	//north, east, height (m)
    float Vw;		//wheelspeed (m/s)
    float phi;   //Roll angle (rad)
    float theta;   //Pitch angle (rad)
    float psi;   //Yaw angle (rad)
    float chi;   //Course angle (rad)
    float p;   //Body frame rollrate (rad/s)
    float q;   //Body frame pitchrate (rad/s)
    float r;   //Body frame yawrate (rad/s)
    float Vg;   //Groundspeed (m/s)
    double base_latitude;   // Latitude [degrees]. Positive is north of equator; negative is south.
    double base_altitude;   // Altitude [m]. Positive is above the WGS 84 ellipsoid
    double base_longitude;   //Longitude [degrees]. Positive is east of prime meridian; negative is west.

  public:
    //init func
    GUI();

    //destructor
    ~GUI();

    //callback function for the ROS listener
    void roverCallback(const std_msgs::String::ConstPtr& msg);

    // returns latitude in degrees
    // Postive is North of equator
    // Negativei is South of equator
    double get_latitude();

    // returns longitude in degrees
    // Positive is east of the Prime Meridian
    // Negative is west of the Prime Meridian
    double get_longitude();

    // returns altitude in meters
    // Positive is above the WGS 84 ellipsoid
    double get_altitude();

    // returns magnitometer in degrees in relation to North (?)
    float get_magnitometer();

    // returns ground speed of rover meters per second
    float get_ground_speed();

  };
}

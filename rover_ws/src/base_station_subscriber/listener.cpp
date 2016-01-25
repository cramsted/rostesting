#include "ros/ros.h"
#include "std_msgs/String.h"

//state variables
float32[3] position	# north, east, height (m)
float32 Vw		# wheelspeed (m/s)
float32 phi		# Roll angle (rad)
float32 theta		# Pitch angle (rad)
float32 psi		# Yaw angle (rad)
float32 chi		# Course angle (rad)
float32 p		# Body frame rollrate (rad/s)
float32 q		# Body frame pitchrate (rad/s)
float32 r		# Body frame yawrate (rad/s)
float32 Vg		# Groundspeed (m/s)
float64 base_latitude   # Latitude [degrees]. Positive is north of equator; negative is south.
float64 base_longitude  # Longitude [degrees]. Positive is east of prime meridian; negative is west.
float64 base_altitude   # Altitude [m]. Positive is above the WGS 84 ellipsoid

int main(int argc, char **argv){
  //init the subscriber and send recieved data to the
  //callback function
  ros::init(argc, argv, "bs_listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("rover", 1000, roverCallback);
  ros::spin();
}
//gps
//magitometer
void roverCallback(const std_msgs::String::ConstPtr& msg){
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

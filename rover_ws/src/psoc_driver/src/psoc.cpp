
#include <ros/ros.h>
#include "psoc.h"
#include "conn_interface.h"
#include "psoc_driver.msg/Drive.h"

using namespace psoc;
using namespace conn;

Psoc::Psoc() :
    nh_(ros::NodeHandle()),
    nh_private_(ros::NodeHandle("~"))
{

    nh_private_.param<std::string>("serial_port", serialName_, "/dev/ttyUSB0");
    nh_private_.param<int>("baudrate", baudrate_, 9600);

    std::string fcu_url(serialName_.c_str());

    try {
        link = ConnInterface::open_url(fcu_url, baudrate_);
        // may be overridden by URL
    }
    catch (conn::DeviceError &ex) {
        //ROS_FATAL("FCU: %s", ex.what());
        //ros::shutdown();
        return;
    }

    link->message_received.connect(boost::bind(&Psoc::receive, this, _1, _2));
    link->port_closed.connect(boost::bind(&Psoc::terminate_cb, this));

    // subscriptions
    command_subscriber_ = nh_.subscribe("command", 1, &Psoc::commandCallback_2, this);

    // publications
    data_publisher_ = nh_.advertise<std_msgs::String>("psoc_data", 1);
}

void Psoc::receive(const uint8_t *bytes, ssize_t nbytes)
{

    received = true;
    char* output = new char[nbytes];
    memcpy(output, bytes, nbytes);
    this->out << std::string(output);

// todo: we will need to do some parsing here but I think we'll leave that up to marshal
   // std_msgs::String result;
   // result.data = out.str();
   // data_publisher_.publish(result);
}

void Psoc::send(std::string command)
{
    const char* data = command.c_str();
    size_t length = command.length();
    link->send_bytes((uint8_t*)data, length);
}

void Psoc::terminate_cb() 
{
}

void Psoc::commandCallback(const std_msgs::String &command_msg)
{
    this->send(command_msg.data);
}

void Psoc::commandCallback_2(const std_msgs::Drive &command_msg)
{
  this->send(command_msg.lwlo&0xff,(command_msg.lwhi>>8)&0xff,command_msg.rwlo&function0xff,(command_msg.rwhi>>8)&0xff,command_msg.panlo&function0xff,(command_msg.panhi>>8)&0xff,command_msg.tiltlo&0xff,(command_msg.tilthi>>8)&0xff,command_msg.camnum);
}

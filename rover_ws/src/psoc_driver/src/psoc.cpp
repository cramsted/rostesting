
#include <ros/ros.h>
#include "psoc.h"
#include "conn_interface.h"

using namespace psoc;
using namespace conn;

Psoc::Psoc() :
    nh_(ros::NodeHandle()),
    nh_private_(ros::NodeHandle("~"))
{

    nh_private_.param<std::string>("serial_port", serialName_, "/dev/ttyUSB2");
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
    command_subscriber_ = nh_.subscribe("drive_command", 1, &Psoc::commandCallback_2, this);

    // publications
    data_publisher_ = nh_.advertise<rover_msgs::SciFeedback>("science_data", 1);
}

void Psoc::receive(const uint8_t *bytes, ssize_t nbytes)
{

    received = true;
    char* output = new char[nbytes];
    memcpy(output, bytes, nbytes);

    if(nbytes == 5 && true)//output[0] == something) //todo: figure out the start byte
    {
        rover_msgs::SciFeedback msg;
        msg.temp = (output[1] << 8 | output[2]);
        msg.humidity = (output[3] << 8 | output[4]);
        data_publisher_.publish(msg);
    }
}

// void Psoc::send(std::string command)
// {
//     const char* data = command.c_str();
//     size_t length = command.length();
//     link->send_bytes((uint8_t*)data, length);
// }

void Psoc::send(uint16_t lw, uint16_t rw, uint16_t pan, uint16_t tilt, uint8_t camnum)
{
  uint8_t array[11];
  array[0]=0xEA;
  array[1]=0xE3;
  array[2]=lw&0xff;
  array[3]=(lw>>8)&0xff;
  array[4]=rw&0xff;
  array[5]=(rw>>8)&0xff;
  array[6]=pan&0xff;
  array[7]=(pan>>8)&0xff;
  array[8]=tilt&0xff;
  array[9]=(tilt>>8)&0xff;
  array[10]=camnum;

  link->send_bytes(array,11);
}

void Psoc::terminate_cb() 
{
}

// void Psoc::commandCallback(const std_msgs::String &command_msg)
// {
//     this->send(command_msg.data);
// }

void Psoc::commandCallback_2(const rover_msgs::Drive &command_msg)
{
  this->send(command_msg.lw,command_msg.rw,command_msg.pan,command_msg.tilt,command_msg.camnum);
  // this->send(command_msg.lw&0xff,(command_msg.lw>>8)&0xff,command_msg.rw&0xff,(command_msg.rw>>8)&0xff,command_msg.pan&0xff,(command_msg.pan>>8)&0xff,command_msg.tilt&0xff,(command_msg.tilt>>8)&0xff,command_msg.camnum);
}

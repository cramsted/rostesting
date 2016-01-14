#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <conn_interface.h>
#include <string>
#include <rover_msgs/Arm.h>
#include <stdint.h>

namespace psoc_arm {

/**
 * @brief PSOC node class
 *
 * This class implement psoc_arm_node
 */
class Psoc_arm
{
public:
    explicit
    Psoc_arm();

    ~Psoc_arm() {}

    bool received;
    std::stringstream out;
    /* void send(std::string command); */
    void send(uint16_t, uint16_t);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber command_subscriber_;
    ros::Publisher data_publisher_;

    void commandCallback(const std_msgs::String& command_msg);
    void commandCallback_2(const rover_msgs::Arm& command_msg);

    std::string serialName_;
    int baudrate_;

    conn::ConnInterface::Ptr link;
    void receive(const uint8_t *bytes, ssize_t nbytes);


    void terminate_cb();

};

}; // namespace psoc

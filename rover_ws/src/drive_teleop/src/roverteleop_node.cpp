// Drive publisher
#include <ros/ros.h>
#include <psoc_driver/Drive.h>
#include <sensor_msgs/Joy.h>

sensor_msgs::Joy joy_msgs; // global variable to store incoming joy messages

using namespace std;

void joyCallback(const sensor_msgs::Joy& msgs)
{
    joy_msgs = msgs; // msgs is the incoming message
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "drive_teleop_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    ros::Publisher drive_command_pub = nh.advertise<psoc_driver::Drive>("drive_command",1);
    ros::Subscriber joy_command = nh.subscribe("joy",1,joyCallback);

    ros::Rate r(10); // 10 hz

    while (ros::ok())
    {
        ros::spinOnce(); // spin let ros sit there and run it's own while loop to listen to all of it's subscribers and wait
            // spinOnce means "check all of my subscribers, and do something if there's a message. Otherwise, I'm going to keep doing stuff"

        if(joy_msgs.axes.size() >= 4)
        {
            psoc_driver::Drive dr_msgs;
            dr_msgs.lw = joy_msgs.axes[1]*-x500 + 1500;  // positive
            dr_msgs.rw = joy_msgs.axes[4]*-500 + 1500; // negative

            drive_command_pub.publish(dr_msgs);
        }

        r.sleep();
    }

}

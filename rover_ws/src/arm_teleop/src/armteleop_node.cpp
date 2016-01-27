// Arm publisher
#include <ros/ros.h>
#include <rover_msgs/Arm.h>
#include <sensor_msgs/Joy.h>

sensor_msgs::Joy joy_msgs; // global variable to store incoming joy messages

using namespace std;

void joyCallback(const sensor_msgs::Joy& msgs)
{
    joy_msgs = msgs; // msgs is the incoming message
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "arm_teleop_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    ros::Publisher arm_command_pub = nh.advertise<rover_msgs::Arm>("arm_command",1);
    ros::Subscriber joy_command = nh.subscribe("joy",1,joyCallback);

    ros::Rate r(10); // 10 hz

    while (ros::ok())
    {
        ros::spinOnce(); // spin let ros sit there and run it's own while loop to listen to all of it's subscribers and wait
            // spinOnce means "check all of my subscribers, and do something if there's a message. Otherwise, I'm going to keep doing stuff"

        if(joy_msgs.axes.size() >= 4)
        {
            rover_msgs::Arm arm_msgs;
            arm_msgs.tur = joy_msgs.axes[1]*-500 + 1500;  // positive
            arm_msgs.sh = joy_msgs.axes[4]*-500 + 1500; // negative

            arm_command_pub.publish(arm_msgs);
        }

        r.sleep();
    }

}

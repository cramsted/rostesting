#ifndef NAV_ESTIMATOR
#define NAV_ESTIMATOR

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include "rover_msgs/NavState.h"

namespace nav_estimator
{

class NavEstimator{
public:
    NavEstimator();
    ~NavEstimator();

private:
    // ros stuff for this node
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber gps_subscriber_;
    ros::Subscriber imu_subscriber_;
    ros::Publisher state_publisher_;

    std::string gps_topic_;
    std::string imu_topic_;
    std::string state_topic_;

    //callbacks
    void gpsCallback(const sensor_msgs::NavSatFix &msg);
    void imuCallback(const sensor_msgs::Imu &msg);

    void initialize();
    void propigate();
    void update();
};
} //end namespace


#endif //NAV_ESITMATOR

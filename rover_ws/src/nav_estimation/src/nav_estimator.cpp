#include "nav_estimator.h"

namespace nav_estimator
{

NavEstimator::NavEstimator():
    nh_(ros::NodeHandle()),
    nh_private_(ros::NodeHandle("~"))
{
    nh_private_.param<std::string>("gps_topic", gps_topic_, "fix");
    nh_private_.param<std::string>("imu_topic", imu_topic_, "imu");
    nh_private_.param<std::string>("state_topic", state_topic_, "nav_state");

    gps_subscriber_ = nh_.subscribe(gps_topic_, 1, &NavEstimator::gpsCallback, this);
    imu_subscriber_ = nh_.subscribe(imu_topic_, 1, &NavEstimator::imuCallback, this);

    state_publisher_ = nh_.advertise<rover_msgs::NavState>(state_topic_, 1);
}

NavEstimator::~NavEstimator()
{}

void NavEstimator::gpsCallback(const sensor_msgs::NavSatFix& msg)
{

}

void NavEstimator::imuCallback(const sensor_msgs::Imu &msg)
{

}

}

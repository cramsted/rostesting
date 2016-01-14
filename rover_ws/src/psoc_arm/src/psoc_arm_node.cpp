/*!
 * \file psoc_node.cpp
 * \author Gary Ellingson
 *
 * \brief This file provides the entry point for the psoc node
 */

#include <ros/ros.h>
#include "psoc_arm.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "psoc_arm_node");
  ros::NodeHandle nh;

  psoc_arm::Psoc_arm psoc_arm;

  ros::spin();

  return 0;
}

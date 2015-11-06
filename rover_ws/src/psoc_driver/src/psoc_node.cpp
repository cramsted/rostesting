/*!
 * \file psoc_node.cpp
 * \author Gary Ellingson
 *
 * \brief This file provides the entry point for the psoc node
 */

#include <ros/ros.h>
#include "psoc.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "psoc_node");
  ros::NodeHandle nh;

  psoc::Psoc psoc;

  ros::spin();

  return 0;
}

/*
* grasp_objects_node.cpp
* Created on: Nov 2, 2022
* Author: Elisabeth Menendez
* RoboticsLab, UC3M
*/

#include <ros/ros.h>
#include <demo_sharon/demo_sharon.hpp>


int main(int argc, char** argv) {
  ros::init(argc, argv, "demo_sharon");
  ros::NodeHandle nodeHandle("~");
  demo_sharon::DemoSharon demoSharon(nodeHandle);

  ros::spin();
  return 0;
}
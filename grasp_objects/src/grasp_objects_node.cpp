/*
* grasp_objects_node.cpp
* Created on: Nov 2, 2022
* Author: Elisabeth Menendez
* RoboticsLab, UC3M
*/

#include <ros/ros.h>
#include <grasp_objects/grasp_objects.hpp>


int main(int argc, char** argv) {
  ros::init(argc, argv, "grasp_objects");
  ros::NodeHandle nodeHandle("~");
  grasp_objects::GraspObjects graspObjects(nodeHandle);

  ros::spin();
  return 0;
}
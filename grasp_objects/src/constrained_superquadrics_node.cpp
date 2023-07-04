/*
* constrained_superquadrics_node.cpp
* Author: Elisabeth Menendez
* RoboticsLab, UC3M
*/

#include <ros/ros.h>
#include <grasp_objects/constrained_superquadrics.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "constrained_superquadrics");
  ros::NodeHandle nodeHandle("~");
  ConstrainedSuperquadrics constrainedSuperquadrics(nodeHandle);
  ros::Rate r(10);
  while(ros::ok()){

    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
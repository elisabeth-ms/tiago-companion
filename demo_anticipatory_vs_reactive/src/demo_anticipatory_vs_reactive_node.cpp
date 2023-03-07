/*
* demo_anticipatory_vs_reactive_node.cpp
* Created on: Nov 2, 2022
* Author: Elisabeth Menendez
* RoboticsLab, UC3M
*/

#include <ros/ros.h>
#include <demo_anticipatory_vs_reactive/demo_anticipatory_vs_reactive.hpp>


int main(int argc, char** argv) {
  ros::init(argc, argv, "demo_anticipatory_vs_reactive");
  ros::NodeHandle nodeHandle("~");
  demo_anticipatory_vs_reactive::DemoAnticipatoryVsReactive demoAnticipatoryVsReactive(nodeHandle);

  
  return 0;
}
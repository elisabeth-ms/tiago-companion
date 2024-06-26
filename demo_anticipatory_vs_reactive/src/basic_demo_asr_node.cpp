/*
* demo_anticipatory_vs_reactive_node.cpp
* Created on: Nov 2, 2022
* Author: Elisabeth Menendez
* RoboticsLab, UC3M
*/

#include <ros/ros.h>
#include <demo_anticipatory_vs_reactive/basic_demo_asr.hpp>


int main(int argc, char** argv) {
  ros::init(argc, argv, "basic_demo_asr_node");
  ros::NodeHandle nodeHandle("~");
  demo_anticipatory_vs_reactive::BasicDemoAsr basicDemoAsr(nodeHandle);

  
  return 0;
}
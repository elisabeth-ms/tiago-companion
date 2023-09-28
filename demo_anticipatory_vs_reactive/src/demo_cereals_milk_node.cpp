/*
* demo_anticipatory_vs_reactive_node.cpp
* Created on: Nov 2, 2022
* Author: Elisabeth Menendez
* RoboticsLab, UC3M
*/

#include <ros/ros.h>
#include <demo_anticipatory_vs_reactive/demo_cereals_milk.hpp>


int main(int argc, char** argv) {
  ros::init(argc, argv, "demo_cereals_milk_node");
  ros::NodeHandle nodeHandle("~");
  demo_cereals_milk::DemoCerealsMilk demoCerealsMilk(nodeHandle);

  
  return 0;
}
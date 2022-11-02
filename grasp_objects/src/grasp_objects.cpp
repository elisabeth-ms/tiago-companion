#include "grasp_objects/grasp_objects.hpp"

namespace grasp_objects{
    GraspObjects::GraspObjects(ros::NodeHandle nh):
    nodeHandle_(nh){
            ROS_INFO("[Graspobjects] Node started.");
    }

    GraspObjects::~GraspObjects(){
    }

}
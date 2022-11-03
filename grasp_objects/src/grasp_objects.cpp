#include "grasp_objects/grasp_objects.hpp"

namespace grasp_objects{
    GraspObjects::GraspObjects(ros::NodeHandle nh):
    nodeHandle_(nh){
        ROS_INFO("[GraspObjects] Node started.");
        init();
    }

    GraspObjects::~GraspObjects(){
    }

    void GraspObjects::init(){
        ROS_INFO("[GraspObjects] init().");
        std::string pointCloudTopicName;
        nodeHandle_.param("grasp_objects/eps_angle/value", epsAnglePlaneSegmentation_, float(0.06));
        nodeHandle_.param("grasp_objects/distance_threshold/value",distanceThresholdPlaneSegmentation_, float(0.04));
        nodeHandle_.param("subscribers/point_cloud/topic", pointCloudTopicName, std::string("/xtion/depth_registered/points"));

        pointCloudSubscriber_ = nodeHandle_.subscribe(pointCloudTopicName, 10, &GraspObjects::pointCloudCallback, this);
        outPointCloudPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("/transformed_cloud",5);
    }


    void GraspObjects::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &pointCloud_msg){
        ROS_INFO("[GraspObjects] pointCloudCallback()");
        sensor_msgs::PointCloud2 pcOut;

        pcl_ros::transformPointCloud("/base_footprint", *pointCloud_msg, pcOut,listener_);
        pcOut.header.frame_id = "/base_footprint";
        outPointCloudPublisher_.publish(pcOut);


    }

}
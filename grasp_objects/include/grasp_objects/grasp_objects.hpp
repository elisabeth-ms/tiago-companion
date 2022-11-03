#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>

namespace grasp_objects{
class GraspObjects{
    public:
    
    explicit GraspObjects(ros::NodeHandle nh);

    ~GraspObjects();

    void init();

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &pointCloud_msg);

    private:
    //! ROS node handle.
    ros::NodeHandle nodeHandle_;
    ros::Subscriber pointCloudSubscriber_;
    ros::Publisher outPointCloudPublisher_;
    tf::TransformListener listener_;
    tf::StampedTransform transform_;
    float epsAnglePlaneSegmentation_;
    float distanceThresholdPlaneSegmentation_;

};
}
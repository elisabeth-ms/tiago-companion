#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/lccp_segmentation.h>

using SuperVoxelAdjacencyList = pcl::LCCPSegmentation<pcl::PointXYZRGB>::SupervoxelAdjacencyList;
namespace grasp_objects{
class GraspObjects{
    public:
    
    explicit GraspObjects(ros::NodeHandle nh);

    ~GraspObjects();

    void init();

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &pointCloud_msg);
    void supervoxelOversegmentation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud,
                                    pcl::PointCloud<pcl::PointXYZL>::Ptr &lccp_labeled_cloud);

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
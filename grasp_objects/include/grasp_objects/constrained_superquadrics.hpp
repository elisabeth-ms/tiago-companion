// Purpose: Header file for the class ConstrainedSuperquadrics.
// Author: Elisabeth Menendez

// ROS HEADERS
#include <ros/ros.h>
// sensor_msgs HEADERS
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

// image headers
#include <image_transport/image_transport.h>
#include <depth_image_proc/depth_conversions.h>
#include <image_geometry/pinhole_camera_model.h>

// pcl_ros HEADERS
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

// pcl library HEADERS
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/lccp_segmentation.h>
#include <pcl/common/centroid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

// COMPANION_MSGS HEADERS
#include <companion_msgs/ActivateSupercuadricsComputation.h>

// STD HEADERS
#include <iostream>
#include <thread>
#include <mutex>
#include <future> // For std::future and std::async

// SUPERQUADRIC MODEL LIBRARY HEADERS

#include <SuperquadricLibModel/superquadric.h>
#include <SuperquadricLibModel/pointCloud.h>
#include <SuperquadricLibModel/superquadricEstimator.h>

struct Object
{
    pcl::PointCloud<pcl::PointXYZRGB> object_cloud;
    pcl::PointCloud<pcl::PointXYZRGB> object_cloud_projected;
    pcl::PointCloud<pcl::PointXYZRGB> object_cloud_hull;
    float max_height = 0.0;
    int label; /**< label assigned by the lccp algorithm*/
    uint32_t r;
    uint32_t g;
    uint32_t b;
};

struct ThreadData
{
    int thread_id;
    int object_id;
    pcl::PointCloud<pcl::PointXYZRGB> object_cloud;
};

class ConstrainedSuperquadrics
{
public:
    explicit ConstrainedSuperquadrics(ros::NodeHandle nh);

    ~ConstrainedSuperquadrics();

    void compressedDepthImageCallback(const sensor_msgs::ImageConstPtr &compressedImage_msg);

    bool activateConstrainedSuperquadrics(companion_msgs::ActivateSupercuadricsComputation::Request &req, companion_msgs::ActivateSupercuadricsComputation::Response &res);

private:
    ros::NodeHandle nodeHandle_;
    image_transport::Subscriber compressedDepthImageSubscriber_;
    std::string compressedDepthImageTopicName = "/xtion/depth_registered/image_raw";
    ros::Publisher pointCloudPublisher_;

    bool activate_ = false;
    std::mutex mtxActivate_;
    int count_ = 0;
    ros::ServiceServer activateService_;
    image_geometry::PinholeCameraModel model_;

    tf::TransformListener listener_;
    tf::StampedTransform transformCameraWrtBase_;

    float epsAnglePlaneSegmentation_ = 6.0;
    float distanceThresholdPlaneSegmentation_ = 0.03;

    bool single_superq_ = true;            // THIS IS FOR THE SUPERQUADRIC MODEL ESTIMATOR
    std::string object_class_ = "default"; // THIS IS FOR THE SUPERQUADRIC MODEL ESTIMATOR
    SuperqModel::SuperqEstimatorApp estim_;
    float tolSuperq_ = 0.0001;
    int optimizerPoints_ = 1000;
    bool randomSampling_ = true;
    int maxIter_ = 100000000;
    int fractionPc_ = 4;
    float thresholdAxis_ = 0.8;
    float thresholdSection1_ = 0.7;
    float thresholdSection2_ = 0.3;
    int th_points_ = 100;
    bool merge_model_ = true;
    int minimumPoints_ = 500; // TODO: CHECK IF THIS PARAMETER IS REALLY NEEDED, I USE IT IN pclPointCloudToSuperqPointCloud

    std::vector<float> table_dimensions_;

    std::string cameraInfoTopicName_ = "/xtion/depth_registered/camera_info";
    std::vector<Object> detectedObjects_;

    bool pclPointCloudToSuperqPointCloud(const pcl::PointCloud<pcl::PointXYZRGB> &object_cloud, SuperqModel::PointCloud &point_cloud);

    void updateDetectedObjectsPointCloud(const pcl::PointCloud<pcl::PointXYZL>::Ptr &lccp_labeled_cloud);

    void getSuperquadricFromPointCloud(SuperqModel::PointCloud point_cloud, std::vector<SuperqModel::Superquadric> &superqs);

    std::vector<SuperqModel::Superquadric> optimizeSuperquadric(ThreadData thread_data);
};

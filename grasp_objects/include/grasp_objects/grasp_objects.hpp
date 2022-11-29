#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <image_transport/image_transport.h>
#include <tf/transform_listener.h>
#include <kdl_conversions/kdl_msg.h>
#include <image_geometry/pinhole_camera_model.h>
#include <depth_image_proc/depth_conversions.h>
#include <cv_bridge/cv_bridge.h>

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
#include <SuperquadricLibModel/superquadricEstimator.h>

#include <mutex>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

#include "sharon_msgs/SuperquadricMultiArray.h"
#include "sharon_msgs/ActivateSupercuadricsComputation.h"
#include "sharon_msgs/GetSuperquadrics.h"
#include "sharon_msgs/ComputeGraspPoses.h"
#include "sharon_msgs/BoundingBoxes.h"
#include "sharon_msgs/GetBboxes.h"

#define DEFAULT_MIN_NPOINTS 100
#define MAX_OBJECT_WIDTH_GRASP 0.16

using SuperVoxelAdjacencyList = pcl::LCCPSegmentation<pcl::PointXYZRGB>::SupervoxelAdjacencyList;
namespace grasp_objects{
    struct Object
    {
        pcl::PointCloud<pcl::PointXYZRGB> object_cloud;
        int label; /**< label assigned by the lccp algorithm*/
        uint32_t r;
        uint32_t g;
        uint32_t b;
    };

    struct ObjectSuperquadric
    {
        pcl::PointCloud<pcl::PointXYZRGBA> cloud;
        int label;
        std::vector<SuperqModel::Superquadric> superqs;
    };


    class GraspObjects{
        public:
        
        explicit GraspObjects(ros::NodeHandle nh);

        ~GraspObjects();

        void init();

        // void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &pointCloud_msg);

        void compressedDepthImageCallback(const  sensor_msgs::ImageConstPtr &compressedImage_msg);

        void setCameraParams(const sensor_msgs::CameraInfo &cameraInfo_msg);

        void getPixelCoordinates(const pcl::PointXYZ &p, int &xpixel, int &ypixel);


        void supervoxelOversegmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr &inputPointCloud,
                                        pcl::PointCloud<pcl::PointXYZL>::Ptr &lccp_labeled_cloud);
        
        void updateDetectedObjectsPointCloud(const pcl::PointCloud<pcl::PointXYZL>::Ptr &lccp_labeled_cloud);


        bool pclPointCloudToSuperqPointCloud(const pcl::PointCloud<pcl::PointXYZRGB> &object_cloud, SuperqModel::PointCloud &point_cloud);

        void getSuperquadricFromPointCloud(SuperqModel::PointCloud point_cloud, std::vector<SuperqModel::Superquadric> &superqs);

        void createPointCloudFromSuperquadric(const std::vector<SuperqModel::Superquadric> &superqs, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloudSuperquadric,
                                             int indexDetectedObjects);


        bool activateSuperquadricsComputation(sharon_msgs::ActivateSupercuadricsComputation::Request & req, sharon_msgs::ActivateSupercuadricsComputation::Response & res);

        bool computeGraspPoses(sharon_msgs::ComputeGraspPoses::Request & req, sharon_msgs::ComputeGraspPoses::Response & res);

        void computeGraspingPosesObject(const std::vector<SuperqModel::Superquadric> &superqs, geometry_msgs::PoseArray &graspingPoses);

        bool getSuperquadrics(sharon_msgs::GetSuperquadrics::Request &req, sharon_msgs::GetSuperquadrics::Response &res);

        bool getBboxes(sharon_msgs::GetBboxes::Request &req, sharon_msgs::GetBboxes::Response &res);

        bool createBoundingBox2DFromSuperquadric(const sharon_msgs::Superquadric &superq, sharon_msgs::BoundingBox & bbox);

        private:
        //! ROS node handle.
        ros::NodeHandle nodeHandle_;
        ros::Subscriber pointCloudSubscriber_;
        image_transport::Subscriber compressedDepthImageSubscriber_;
        ros::Subscriber cameraInfoSubscriber_;
        ros::Publisher outPointCloudPublisher_;
        ros::Publisher outPointCloudSuperqsPublisher_;
        ros::Publisher superquadricsPublisher_;
        ros::Publisher graspPosesPublisher_;
        ros::Publisher bbox3dPublisher_;


        ros::ServiceServer serviceActivateSuperquadricsComputation_; 
        ros::ServiceServer serviceComputeGraspPoses_;
        ros::ServiceServer serviceGetSuperquadrics_;
        ros::ServiceServer serviceGetBboxesSuperquadrics_;


        tf::TransformListener listener_;
        tf::StampedTransform transform_;
        float epsAnglePlaneSegmentation_;
        float distanceThresholdPlaneSegmentation_;
        float tolSuperq_;
        int optimizerPoints_;
        bool randomSampling_;
        int minimumPoints_;
        int maxIter_;
        int fractionPc_;
        float thresholdAxis_;
        float thresholdSection1_;
        float thresholdSection2_;
        int th_points_;
        std::string object_class_;
        bool single_superq_;
        bool merge_model_;

        int height_ = 480;
        int width_ = 640;
        std::map<std::string,double> sq_model_params_;
        SuperqModel::SuperqEstimatorApp estim_;

        std::vector<Object> detectedObjects_;
        std::vector<ObjectSuperquadric> superquadricObjects_;
        sharon_msgs::SuperquadricMultiArray superquadricsMsg_;

        bool activate_ = false;
        std::mutex mtxActivate_;

        float focalLengthX_, focalLengthY_;
        float principalPointX_, principalPointY_; 

        tf::StampedTransform transformCameraWrtBase_;
        image_geometry::PinholeCameraModel model_;


};
}
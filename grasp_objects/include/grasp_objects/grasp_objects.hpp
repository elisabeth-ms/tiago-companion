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
#include <SuperquadricLibModel/superquadricEstimator.h>
#include <sharon_msgs/SuperquadricMultiArray.h>

#define DEFAULT_MIN_NPOINTS 100


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

        void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &pointCloud_msg);
        void supervoxelOversegmentation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud,
                                        pcl::PointCloud<pcl::PointXYZL>::Ptr &lccp_labeled_cloud);
        
        void updateDetectedObjectsPointCloud(const pcl::PointCloud<pcl::PointXYZL>::Ptr &lccp_labeled_cloud);


        bool pclPointCloudToSuperqPointCloud(const pcl::PointCloud<pcl::PointXYZRGB> &object_cloud, SuperqModel::PointCloud &point_cloud);

        void getSuperquadricFromPointCloud(SuperqModel::PointCloud point_cloud, std::vector<SuperqModel::Superquadric> &superqs);

        void createPointCloudFromSuperquadric(const std::vector<SuperqModel::Superquadric> &superqs, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloudSuperquadric,
                                             int indexDetectedObjects);


        private:
        //! ROS node handle.
        ros::NodeHandle nodeHandle_;
        ros::Subscriber pointCloudSubscriber_;
        ros::Publisher outPointCloudPublisher_;
        ros::Publisher outPointCloudSuperqsPublisher_;
        ros::Publisher superquadricsPublisher_;
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
        std::map<std::string,double> sq_model_params_;
        SuperqModel::SuperqEstimatorApp estim_;

        std::vector<Object> detectedObjects_;
        std::vector<ObjectSuperquadric> superquadricObjects_;




};
}
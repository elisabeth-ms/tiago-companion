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
        nodeHandle_.param("grasp_objects/eps_angle/value", epsAnglePlaneSegmentation_, float(0.01));
        nodeHandle_.param("grasp_objects/distance_threshold/value",distanceThresholdPlaneSegmentation_, float(0.01));
        nodeHandle_.param("subscribers/point_cloud/topic", pointCloudTopicName, std::string("/xtion/depth_registered/points"));

        pointCloudSubscriber_ = nodeHandle_.subscribe(pointCloudTopicName, 10, &GraspObjects::pointCloudCallback, this);
        outPointCloudPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("/transformed_cloud",5);
    }

    void GraspObjects::supervoxelOversegmentation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud, pcl::PointCloud<pcl::PointXYZL>::Ptr &lccp_labeled_cloud)
    {

        // ------------------------------- Compute normals of the the input cloud ------------------------------------------------- //

        // Create the normal estimation class, and pass the input dataset to it
        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
        ne.setInputCloud(inputPointCloud);

        // Create an empty kdtree representation, and pass it to the normal estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
        ne.setSearchMethod(tree);
        // Output datasets
        pcl::PointCloud<pcl::Normal>::Ptr input_normals_ptr(new pcl::PointCloud<pcl::Normal>);

        // Use all neighbors in a sphere of radius 3cm (TODO: Pass as parameter)
        ne.setRadiusSearch(0.03);
        // Compute the features
        ne.compute(*input_normals_ptr);

        // yInfo("Input cloud has %d normals", input_normals_ptr->size());

        // TODO: Change to yarp params
        float voxel_resolution = 0.01f;
        float seed_resolution = 0.02f;
        float color_importance = 0.0f;
        float spatial_importance = 2.0f;
        float normal_importance = 2.0f;
        bool use_single_cam_transform = false;
        bool use_supervoxel_refinement = false;

        // LCCPSegmentation Stuff
        float concavity_tolerance_threshold = 20;
        float smoothness_threshold = 0.2;
        std::uint32_t min_segment_size = 10;
        bool use_extended_convexity = false;
        bool use_sanity_criterion = true;

        unsigned int k_factor = 0;
        if (use_extended_convexity)
            k_factor = 1;

        pcl::SupervoxelClustering<pcl::PointXYZRGB> super(voxel_resolution, seed_resolution);
        super.setUseSingleCameraTransform(use_single_cam_transform);
        super.setInputCloud(inputPointCloud);
        super.setNormalCloud(input_normals_ptr);
        super.setColorImportance(color_importance);
        super.setSpatialImportance(spatial_importance);
        super.setNormalImportance(normal_importance);
        std::map<std::uint32_t, pcl::Supervoxel<pcl::PointXYZRGB>::Ptr> supervoxel_clusters;

        if (use_supervoxel_refinement)
        {
            // PCL_INFO ("Refining supervoxels\n");
            super.refineSupervoxels(2, supervoxel_clusters);
        }

        // PCL_INFO ("Extracting supervoxels\n");
        super.extract(supervoxel_clusters);

        std::stringstream temp;
        temp << "  Nr. Supervoxels: " << supervoxel_clusters.size() << "\n";
        PCL_INFO(temp.str().c_str());
        // yInfo() << "Max label: " << super.getMaxLabel();
        // PCL_INFO ("Getting supervoxel adjacency\n");
        std::multimap<std::uint32_t, std::uint32_t> supervoxel_adjacency;
        super.getSupervoxelAdjacency(supervoxel_adjacency);

        pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud = super.getLabeledCloud();

        /// Get the cloud of supervoxel centroid with normals and the colored cloud with supervoxel coloring (this is used for visulization)
        pcl::PointCloud<pcl::PointNormal>::Ptr sv_centroid_normal_cloud = pcl::SupervoxelClustering<pcl::PointXYZRGB>::makeSupervoxelNormalCloud(supervoxel_clusters);

        // pcl::io::savePCDFile ("svcloud.pcd", *sv_centroid_normal_cloud, true);

        PCL_INFO("Starting Segmentation\n");
        // yInfo() << "supervoxel clusters: " << supervoxel_clusters.size();

        pcl::LCCPSegmentation<pcl::PointXYZRGB> lccp;
        lccp.reset();
        lccp.setConcavityToleranceThreshold(concavity_tolerance_threshold);
        lccp.setSanityCheck(use_sanity_criterion);
        lccp.setSmoothnessCheck(true, voxel_resolution, seed_resolution, smoothness_threshold);
        lccp.setKFactor(k_factor);
        lccp.setInputSupervoxels(supervoxel_clusters, supervoxel_adjacency);
        lccp.setMinSegmentSize(min_segment_size);
        lccp.segment();
        SuperVoxelAdjacencyList sv_adjacency_list;
        lccp.getSVAdjacencyList(sv_adjacency_list); // Needed for visualization

        // lccp.getSegmentAdjacencyMap(supervoxel_adjacency);

        // PCL_INFO ("Interpolation voxel cloud -> input cloud and relabeling\n");

        // PCL_INFO ("Get labeled cloud done\n");

        lccp_labeled_cloud = sv_labeled_cloud->makeShared();
        // PCL_INFO ("makeShared\n");

        lccp.relabelCloud(*lccp_labeled_cloud);

        // yInfo() << "supervoxel clusters: " << supervoxel_clusters.size();

        // PCL_INFO ("relabel\n");
    }


    void GraspObjects::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &pointCloud_msg){
        ROS_INFO("[GraspObjects] pointCloudCallback()");
        sensor_msgs::PointCloud2 pcOut;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_without_table (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl_ros::transformPointCloud("/base_footprint", *pointCloud_msg, pcOut,listener_);


        pcl::PCLPointCloud2* cloudFiltered = new pcl::PCLPointCloud2;
        pcl::PCLPointCloud2ConstPtr cloudFilteredPtr(cloudFiltered);
        // Convert to PCL data type
        pcl_conversions::toPCL(pcOut, *cloudFiltered);

        // Perform the actual filtering
        pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
        sor.setInputCloud(cloudFilteredPtr);
        sor.setLeafSize (0.005f, 0.005f, 0.005f);
        sor.filter(*cloudFiltered);

        // Create the filtering object
        pcl::PassThrough<pcl::PCLPointCloud2> pass;
        pass.setInputCloud (cloudFilteredPtr);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0.2, 1.5);
        //pass.setFilterLimitsNegative (true);
        pass.filter(*cloudFiltered);

        pcl::fromPCLPointCloud2 (*cloudFiltered, *cloud_without_table);
        
        // Coefficients and inliners objects for tge ransac plannar model
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        // Optional
        seg.setOptimizeCoefficients(true);
        // Mandatory
        seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(2000);
        seg.setDistanceThreshold(distanceThresholdPlaneSegmentation_);
        seg.setAxis(Eigen::Vector3f::UnitX());
        seg.setEpsAngle(epsAnglePlaneSegmentation_);

        seg.setInputCloud(cloud_without_table);
        seg.segment(*inliers, *coefficients);
         // Create the filtering object
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;

        extract.setInputCloud(cloud_without_table);
        extract.setIndices(inliers);
        extract.setNegative(true); // Extract the inliers
        extract.filter(*cloud_without_table);

        pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud;
        supervoxelOversegmentation(cloud_without_table, lccp_labeled_cloud);

        // Convert to ROS data type
        pcl::toPCLPointCloud2 (*lccp_labeled_cloud, *cloudFiltered);
        pcl_conversions::moveFromPCL(*cloudFiltered, pcOut);
        pcOut.header.frame_id = "/base_footprint";
        outPointCloudPublisher_.publish(pcOut);


    }

}
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
        nodeHandle_.param("grasp_objects/tol_superq/value", tolSuperq_, float(1e-4));
        nodeHandle_.param("grasp_objects/optimizer_points/value", optimizerPoints_, int(100));
        nodeHandle_.param("grasp_objects/random_sampling/value", randomSampling_, bool(true));
        nodeHandle_.param("grasp_objects/max_iter/value", maxIter_, int(10000000));
        nodeHandle_.param("grasp_objects/minimum_points/value", minimumPoints_, int(50));
        nodeHandle_.param("grasp_objects/fraction_pc/value", fractionPc_, int(4));
        nodeHandle_.param("grasp_objects/threshold_axis/value", thresholdAxis_, float(0.7));
        nodeHandle_.param("grasp_objects/threshold_section1/value", thresholdSection1_, float(0.6));
        nodeHandle_.param("grasp_objects/threshold_section2/value", thresholdSection2_, float(0.03));
        nodeHandle_.param("grasp_objects/object_class/value", object_class_, std::string("default"));
        nodeHandle_.param("grasp_objects/single_superq/value", single_superq_, bool(true));
        nodeHandle_.param("grasp_objects/merge_model/value", merge_model_, bool(true));
        nodeHandle_.param("grasp_objects/th_points/value", th_points_, int(40));


        estim_.SetNumericValue("tol", tolSuperq_);
        estim_.SetIntegerValue("print_level", 0);
        estim_.SetStringValue("object_class", object_class_);
        estim_.SetIntegerValue("optimizer_points", optimizerPoints_);
        estim_.SetBoolValue("random_sampling", randomSampling_);

        estim_.SetBoolValue("merge_model", merge_model_);
        estim_.SetIntegerValue("minimum_points", minimumPoints_);
        estim_.SetIntegerValue("fraction_pc", fractionPc_);
        estim_.SetNumericValue("threshold_axis", thresholdAxis_);
        estim_.SetNumericValue("threshold_section1", thresholdSection1_);
        estim_.SetNumericValue("threshold_section2", thresholdSection2_);

        pointCloudSubscriber_ = nodeHandle_.subscribe(pointCloudTopicName, 10, &GraspObjects::pointCloudCallback, this);

        outPointCloudPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("/transformed_cloud",5);
        outPointCloudSuperqsPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("/grasp_objects/superquadrics_cloud",5);
        superquadricsPublisher_ = nodeHandle_.advertise<sharon_msgs::SuperquadricMultiArray>("/grasp_objects/superquadrics", 5);
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
        ne.setRadiusSearch(0.035);
        // Compute the features
        ne.compute(*input_normals_ptr);


        // TODO: Change to ROS params
        float voxel_resolution = 0.05f;
        float seed_resolution = 0.02f;
        float color_importance = 0.0f;
        float spatial_importance = 2.0f;
        float normal_importance = 2.0f;
        bool use_single_cam_transform = false;
        bool use_supervoxel_refinement = false;

        // LCCPSegmentation Stuff
        float concavity_tolerance_threshold = 20;
        float smoothness_threshold = 0.4;
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
        // PCL_INFO ("Getting supervoxel adjacency\n");
        std::multimap<std::uint32_t, std::uint32_t> supervoxel_adjacency;
        super.getSupervoxelAdjacency(supervoxel_adjacency);

        pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud = super.getLabeledCloud();

        /// Get the cloud of supervoxel centroid with normals and the colored cloud with supervoxel coloring (this is used for visulization)
        pcl::PointCloud<pcl::PointNormal>::Ptr sv_centroid_normal_cloud = pcl::SupervoxelClustering<pcl::PointXYZRGB>::makeSupervoxelNormalCloud(supervoxel_clusters);

        // pcl::io::savePCDFile ("svcloud.pcd", *sv_centroid_normal_cloud, true);

        PCL_INFO("Starting Segmentation\n");

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


        // PCL_INFO ("relabel\n");
    }

    void GraspObjects::updateDetectedObjectsPointCloud(const pcl::PointCloud<pcl::PointXYZL>::Ptr &lccp_labeled_cloud)
    {
        detectedObjects_.clear();
        for (int i = 0; i < lccp_labeled_cloud->points.size(); ++i)
        {

            uint32_t idx = lccp_labeled_cloud->points[i].label;

            // in this way we enlarges the vector everytime we encounter a greater label. So we don't need to pass all
            //  labeeld point cloud to see what is the greater label, and then to resize the vector.
            if (idx >= detectedObjects_.size()) // keep in mind that there is also the label 0!
            {
                detectedObjects_.resize(idx + 1);
            }
            // if (detected_objects[idx].object_cloud.empty())
            // {
            //     detected_objects[idx].r = rand() % 256;
            //     detected_objects[idx].g = rand() % 256;
            //     detected_objects[idx].b = rand() % 256;
            // }
            pcl::PointXYZRGB tmp_point_rgb;
            tmp_point_rgb.x = lccp_labeled_cloud->points[i].x;
            tmp_point_rgb.y = lccp_labeled_cloud->points[i].y;
            tmp_point_rgb.z = lccp_labeled_cloud->points[i].z;
            tmp_point_rgb.r = rand() % 256;
            tmp_point_rgb.g = rand() % 256;
            tmp_point_rgb.b = rand() % 256;

            detectedObjects_[idx].object_cloud.points.push_back(tmp_point_rgb);
            detectedObjects_[idx].label = (int)idx;
        }

        // remove segments with too few points
        // it will removes te ones with few points or the ones with no points (these are created because of the labels of lccp)
        int size = detectedObjects_.size();
        ROS_INFO("[GraspObjects] size: %d", detectedObjects_.size());

        int i = 0;
        while (i < size)
        {
            if (detectedObjects_[i].object_cloud.size() < this->th_points_)
            {
                detectedObjects_.erase(detectedObjects_.begin() + i);
                size = detectedObjects_.size();
            }
            else
            {
                detectedObjects_[i].object_cloud.width = detectedObjects_[i].object_cloud.size();
                detectedObjects_[i].object_cloud.height = 1;
                i++;
            }
        }
        ROS_INFO("[GraspObjects] size: %d", detectedObjects_.size());
    }


    void GraspObjects::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &pointCloud_msg){
        ROS_INFO("[GraspObjects] pointCloudCallback()");
        sensor_msgs::PointCloud2 pcOut;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_without_table (new pcl::PointCloud<pcl::PointXYZRGB>);

        tf::StampedTransform transform;
        
        listener_.lookupTransform("/base_footprint", "/xtion_rgb_optical_frame", ros::Time(0), transform);
        


        pcl_ros::transformPointCloud(std::string("/base_footprint"),transform, *pointCloud_msg, pcOut);


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

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudSuperquadric(new pcl::PointCloud<pcl::PointXYZRGBA>);
        superquadricObjects_.clear();
        if (lccp_labeled_cloud->points.size() != 0)
        {
            
            sharon_msgs::SuperquadricMultiArray superquadricsMsg;
            superquadricsMsg.header.stamp = ros::Time::now();
            updateDetectedObjectsPointCloud(lccp_labeled_cloud);

            std::vector<std::vector<double>> graspingPoses;

            for (unsigned int idx = 0; idx < detectedObjects_.size(); idx++)
            {
                SuperqModel::PointCloud point_cloud;
                pclPointCloudToSuperqPointCloud(detectedObjects_[idx].object_cloud, point_cloud);
                ROS_INFO("pointCloud points: %d", point_cloud.n_points);
                std::vector<SuperqModel::Superquadric> superqs;
                getSuperquadricFromPointCloud(point_cloud, superqs);
                sharon_msgs::Superquadric superquadric;
                auto params = superqs[0].getSuperqParams();
                superquadric.a1 = params[0];
                superquadric.a2 = params[1];
                superquadric.a3 = params[2];
                superquadric.e1 = params[3];
                superquadric.e2 = params[4];
                superquadric.x = params[5];
                superquadric.y = params[6];
                superquadric.z = params[7];
                superquadric.roll = params[8];
                superquadric.pitch = params[9];
                superquadric.yaw = params[10];

                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr auxCloudSuperquadric(new pcl::PointCloud<pcl::PointXYZRGBA>);

                // This is only for visulazition of the superquadrics
                createPointCloudFromSuperquadric(superqs, auxCloudSuperquadric, idx);
                *cloudSuperquadric += *auxCloudSuperquadric;

                ObjectSuperquadric objectSuperquadric;
                objectSuperquadric.label = detectedObjects_[idx].label;
                objectSuperquadric.superqs = superqs;
                objectSuperquadric.cloud = *auxCloudSuperquadric;

                superquadricObjects_.push_back(objectSuperquadric);
                superquadricsMsg.superquadrics.push_back(superquadric);

            }
            // Convert to ROS data type
            sensor_msgs::PointCloud2 pcOutSupeqs;
            pcl::PCLPointCloud2* cloudAux = new pcl::PCLPointCloud2;

            pcl::toPCLPointCloud2 (*cloudSuperquadric, *cloudAux);
            pcl_conversions::moveFromPCL(*cloudAux, pcOutSupeqs);
            pcOutSupeqs.header.frame_id = "/base_footprint";
            outPointCloudSuperqsPublisher_.publish(pcOutSupeqs);
            superquadricsPublisher_.publish(superquadricsMsg);
        }

    }


    void GraspObjects::createPointCloudFromSuperquadric(const std::vector<SuperqModel::Superquadric> &superqs, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloudSuperquadric, int indexDetectedObjects)
    {
        double step = 0.005;
        auto params = superqs[0].getSuperqParams();

        std::cout << "a0: " << params[0] << " a1: " << params[1] << " a2: " << params[2] << std::endl;

        for (double x = 0; x <= params[0]; x += step)
        {
            for (double y = 0; y <= params[1]; y += step)
            {
                for (double z = 0; z <= params[2]; z += step)
                {
                    double f = pow(abs(pow(abs(x / params[0]), 2.0 / params[4]) + pow(abs(y / params[1]), 2.0 / params[4])), params[4] / params[3]) + pow(abs(z / params[2]), 2.0 / params[3]);
                    if (f <= 1.0)
                    {
                        pcl::PointXYZRGBA p;

                        p.x = x;
                        p.y = y;
                        p.z = z;
                        p.r = detectedObjects_[indexDetectedObjects].object_cloud.points[0].r;
                        p.g = detectedObjects_[indexDetectedObjects].object_cloud.points[0].g;
                        p.b = detectedObjects_[indexDetectedObjects].object_cloud.points[0].b;
                        cloudSuperquadric->push_back(*(pcl::PointXYZRGBA *)(&p));

                        p.z = -z;
                        cloudSuperquadric->push_back(*(pcl::PointXYZRGBA *)(&p));

                        p.x = -x;
                        p.y = y;
                        p.z = z;
                        cloudSuperquadric->push_back(*(pcl::PointXYZRGBA *)(&p));
                        p.z = -z;
                        cloudSuperquadric->push_back(*(pcl::PointXYZRGBA *)(&p));

                        p.x = x;
                        p.y = -y;
                        p.z = z;
                        cloudSuperquadric->push_back(*(pcl::PointXYZRGBA *)(&p));
                        p.z = -z;
                        cloudSuperquadric->push_back(*(pcl::PointXYZRGBA *)(&p));

                        p.x = -x;
                        p.y = -y;
                        p.z = z;
                        cloudSuperquadric->push_back(*(pcl::PointXYZRGBA *)(&p));

                        p.z = -z;
                        cloudSuperquadric->push_back(*(pcl::PointXYZRGBA *)(&p));
                    }
                }
            }
        }

        Eigen::AngleAxisf rollAngle(params[8], Eigen::Vector3f::UnitZ());
        Eigen::AngleAxisf yawAngle(params[9], Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf pitchAngle(params[10], Eigen::Vector3f::UnitZ());

        Eigen::Quaternionf q = rollAngle * yawAngle * pitchAngle;
        Eigen::Matrix3f rotationMatrix = q.matrix();

        Eigen::Vector3f v(params[5], params[6], params[7]);
        Eigen::Matrix4f transform = Eigen::Affine3f(Eigen::Translation3f(v)).matrix();

        transform.block<3, 3>(0, 0) = rotationMatrix;
        pcl::transformPointCloud(*cloudSuperquadric, *cloudSuperquadric, transform);
    }


    bool GraspObjects::pclPointCloudToSuperqPointCloud(const pcl::PointCloud<pcl::PointXYZRGB> &object_cloud, SuperqModel::PointCloud &point_cloud)
    {
        std::vector<std::vector<unsigned char>> acquired_colors;

        // yarp::sig::Vector point(3);
        std::vector<unsigned char> c;
        c.resize(3);
        std::deque<Eigen::Vector3d> eigen_points;


        for (size_t idx = 0; idx < object_cloud.size(); idx++)
        {
            Eigen::Vector3d point(object_cloud[idx].x, object_cloud[idx].y, object_cloud[idx].z);

            c[0] = object_cloud[idx].r;
            c[1] = object_cloud[idx].g;
            c[2] = object_cloud[idx].b;

            eigen_points.push_back(point);
            acquired_colors.push_back(c);
        }


        // for (size_t i = 0; i < acquired_points.size(); i++)
        // {
        //     eigen_points.push_back(yarp::eigen::toEigen(acquired_points[i]));
        // }

        if (eigen_points.size() >= minimumPoints_)
        {
            point_cloud.setPoints(eigen_points);
            point_cloud.setColors(acquired_colors);
            return true;
        }
        else
            return false;
    }

    void GraspObjects::getSuperquadricFromPointCloud(SuperqModel::PointCloud point_cloud, std::vector<SuperqModel::Superquadric> &superqs)
    {

        /*  ------------------------------  */
        /*  ------> Compute superq <------  */
        /*  ------------------------------  */

        ROS_INFO("[grasp_objects]: compute superq");
        if (single_superq_ || object_class_ != "default")
        {
            ROS_INFO("!!!!!");
            estim_.SetStringValue("object_class", object_class_);
            superqs = estim_.computeSuperq(point_cloud);
        }
    }

}
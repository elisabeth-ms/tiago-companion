#include "grasp_objects/constrained_superquadrics.hpp"

ConstrainedSuperquadrics::ConstrainedSuperquadrics(ros::NodeHandle nh) : nodeHandle_(nh)
{
    ROS_INFO("[ConstrainedSuperquadrics] Node started.");
    image_transport::ImageTransport it(nodeHandle_);
    pointCloudPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("pointcloud", 1);
    ROS_INFO("[ConstrainedSuperquadrics] Waiting to get the camera info...");
    sensor_msgs::CameraInfoConstPtr cameraInfoMsg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(cameraInfoTopicName_);
    model_.fromCameraInfo(cameraInfoMsg);

    compressedDepthImageSubscriber_ = it.subscribe(compressedDepthImageTopicName, 10, &ConstrainedSuperquadrics::compressedDepthImageCallback, this, image_transport::TransportHints("raw"));
    activateService_ = nodeHandle_.advertiseService("activateConstrainedSuperquadrics", &ConstrainedSuperquadrics::activateConstrainedSuperquadrics, this);
    count_ = 0;

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
}

ConstrainedSuperquadrics::~ConstrainedSuperquadrics()
{
}

void ConstrainedSuperquadrics::compressedDepthImageCallback(const sensor_msgs::ImageConstPtr &depth_msg)
{
    // ROS_INFO("[GraspObjects] Receiving the compressed depth image");
    bool activate;
    mtxActivate_.lock();
    activate = activate_;
    mtxActivate_.unlock();
    if (activate && count_ < 1)
    {
        count_++;
        ROS_INFO("COUNT %d", count_);
        sensor_msgs::PointCloud2 pcOut;

        sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);
        cloud_msg->header = depth_msg->header;
        cloud_msg->height = depth_msg->height;
        cloud_msg->width = depth_msg->width;
        cloud_msg->is_dense = false;
        cloud_msg->is_bigendian = false;

        sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
        pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

        depth_image_proc::convert<float>(depth_msg, cloud_msg, model_); // TYPE TYPE_32FC1

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_without_table(new pcl::PointCloud<pcl::PointXYZ>);

        listener_.lookupTransform("/base_footprint", "/xtion_rgb_optical_frame", ros::Time(0), transformCameraWrtBase_);

        pcl_ros::transformPointCloud(std::string("/base_footprint"), transformCameraWrtBase_, *cloud_msg, pcOut);

        pcl::PCLPointCloud2 *cloudFiltered = new pcl::PCLPointCloud2;
        pcl::PCLPointCloud2ConstPtr cloudFilteredPtr(cloudFiltered);
        // Convert to PCL data type
        pcl_conversions::toPCL(pcOut, *cloudFiltered);

        // Perform the actual filtering
        pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
        sor.setInputCloud(cloudFilteredPtr);
        sor.setLeafSize(0.005f, 0.005f, 0.005f);
        sor.filter(*cloudFiltered);

        // Create the filtering object
        pcl::PassThrough<pcl::PCLPointCloud2> pass;
        pass.setInputCloud(cloudFilteredPtr);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(0.3, 1.6);
        // pass.setFilterLimitsNegative (true);
        pass.filter(*cloudFiltered);

        // Create the filtering object
        pcl::PassThrough<pcl::PCLPointCloud2> passZ;
        passZ.setInputCloud(cloudFilteredPtr);
        passZ.setFilterFieldName("z");
        passZ.setFilterLimits(0.3, 1.6);
        // pass.setFilterLimitsNegative (true);
        passZ.filter(*cloudFiltered);

        pcl::fromPCLPointCloud2(*cloudFiltered, *cloud_without_table);

        // Coefficients and inliners objects for tge ransac plannar model
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;
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
        pcl::ExtractIndices<pcl::PointXYZ> extract;

        extract.setInputCloud(cloud_without_table);
        extract.setIndices(inliers);
        extract.setNegative(true); // Extract the inliers
        extract.filter(*cloud_without_table);

        pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud(new pcl::PointCloud<pcl::PointXYZL>);
        // supervoxelOversegmentation(cloud_without_table, lccp_labeled_cloud);

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud_without_table);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.04); // 2cm
        ec.setMinClusterSize(100);

        ec.setMaxClusterSize(25000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_without_table);
        ec.extract(cluster_indices);

        int j = 0;
        for (const auto &cluster : cluster_indices)
        {
            j++;
            for (const auto &idx : cluster.indices)
            {
                pcl::PointXYZL point;
                point.x = cloud_without_table->points[idx].x;
                point.y = cloud_without_table->points[idx].y;
                point.z = cloud_without_table->points[idx].z;
                point.label = j;
                lccp_labeled_cloud->push_back(point);
            }
        } //*

        // // Convert to ROS data type
        // pcl::toPCLPointCloud2(*lccp_labeled_cloud, *cloudFiltered);
        // pcl_conversions::moveFromPCL(*cloudFiltered, pcOut);
        // pcOut.header.frame_id = "/base_footprint";

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudSuperquadric(new pcl::PointCloud<pcl::PointXYZRGBA>);
        if (lccp_labeled_cloud->points.size() != 0)
        {

            updateDetectedObjectsPointCloud(lccp_labeled_cloud);

            std::vector<std::vector<double>> graspingPoses;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr allPoints;

            ros::Time begin = ros::Time::now();

            // // This is for testing if the dynamic number of threads are working
            // detectedObjects_.clear();
            // for (int test_i = 0; test_i < 14; test_i++)
            // {
            //     Object object;
            //     object.label = test_i;
            //     detectedObjects_.push_back(object);
            // }

            unsigned int number_threads = 4;
            unsigned int quotient = detectedObjects_.size() / number_threads;  // Use number_threads threads during quotient iterations
            unsigned int remainder = detectedObjects_.size() % number_threads; // Then use remainder threads for the remaining objects

            ROS_INFO("Number of threads: %d", number_threads);
            ROS_INFO("Quotient: %d", quotient);
            ROS_INFO("Remainder: %d", remainder);
            for (unsigned int iteration = 0; iteration < quotient; iteration++)
            {
                std::vector<std::future<std::vector<SuperqModel::Superquadric>>> futures;
                for (unsigned int idx = 0; idx < number_threads; idx++)
                {
                    ThreadData data;
                    data.thread_id = idx;
                    data.object_id = detectedObjects_[iteration * number_threads + idx].label;
                    data.object_cloud = detectedObjects_[iteration * number_threads + idx].object_cloud;
                    futures.push_back(std::async(std::launch::async, &ConstrainedSuperquadrics::optimizeSuperquadric, this, data));
                }
                for (unsigned int idx = 0; idx < number_threads; idx++)
                {
                    std::vector<SuperqModel::Superquadric> superquadrics = futures[idx].get();
                }
            }
            if (remainder != 0)
            {
                std::vector<std::future<std::vector<SuperqModel::Superquadric>>> futures;
                for (unsigned int idx = 0; idx < remainder; idx++)
                {

                    ThreadData data;
                    data.thread_id = idx;
                    data.object_id = detectedObjects_[quotient * number_threads + idx].label;
                    data.object_cloud = detectedObjects_[quotient * number_threads + idx].object_cloud;
                    futures.push_back(std::async(std::launch::async, &ConstrainedSuperquadrics::optimizeSuperquadric, this, data));
                }
                for (unsigned int idx = 0; idx < remainder; idx++)
                {
                    std::vector<SuperqModel::Superquadric> superquadrics = futures[idx].get();
                }
            }

            ros::Time end = ros::Time::now();
            ROS_INFO("Time for optimization: %f", (end - begin).toSec());

            // Convert to ROS data type
            pcl::toPCLPointCloud2(*lccp_labeled_cloud, *cloudFiltered);
            pcl_conversions::moveFromPCL(*cloudFiltered, pcOut);
            pcOut.header.frame_id = "/base_footprint";
            pointCloudPublisher_.publish(pcOut);
        }
    }
}

bool ConstrainedSuperquadrics::activateConstrainedSuperquadrics(companion_msgs::ActivateSupercuadricsComputation::Request &req, companion_msgs::ActivateSupercuadricsComputation::Response &res)
{

    mtxActivate_.lock();
    activate_ = req.activate;
    count_ = 0;
    ROS_INFO("[ConstrainedSuperquadrics] Activate superquadrics Computation: %d", activate_);
    res.success = true;
    mtxActivate_.unlock();
    return true;
}

void ConstrainedSuperquadrics::updateDetectedObjectsPointCloud(const pcl::PointCloud<pcl::PointXYZL>::Ptr &lccp_labeled_cloud)
{
    detectedObjects_.clear();
    for (int i = 0; i < lccp_labeled_cloud->points.size(); ++i)
    {

        uint32_t idx = lccp_labeled_cloud->points[i].label;
        if (idx > 0)
        {

            // in this way we enlarges the vector everytime we encounter a greater label. So we don't need to pass all
            //  labeeld point cloud to see what is the greater label, and then to resize the vector.
            if (idx >= detectedObjects_.size()) // keep in mind that there is also the label 0!
            {
                detectedObjects_.resize(idx + 1);
            }

            pcl::PointXYZRGB tmp_point_rgb;
            tmp_point_rgb.x = lccp_labeled_cloud->points[i].x;
            tmp_point_rgb.y = lccp_labeled_cloud->points[i].y;
            tmp_point_rgb.z = lccp_labeled_cloud->points[i].z;
            tmp_point_rgb.r = rand() % 256;
            tmp_point_rgb.g = rand() % 256;
            tmp_point_rgb.b = rand() % 256;

            if (lccp_labeled_cloud->points[i].z > detectedObjects_[idx].max_height)
            {
                detectedObjects_[idx].max_height = lccp_labeled_cloud->points[i].z;
            }

            detectedObjects_[idx].object_cloud.points.push_back(tmp_point_rgb);
            detectedObjects_[idx].label = (int)idx;
        }
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

std::vector<SuperqModel::Superquadric> ConstrainedSuperquadrics::optimizeSuperquadric(ThreadData thread_data)
{
    ROS_INFO("[ConstrainedSuperquadrics] optimizeSuperquadric");
    ROS_INFO("[ConstrainedSuperquadrics] THREAD: %d", thread_data.thread_id);
    ROS_INFO("[ConstrainedSuperquadrics] LABEL: %d", thread_data.object_id);
    ROS_INFO("[ConstrainedSuperquadrics] OBJECT CLOUD SIZE: %d", thread_data.object_cloud.size());

    SuperqModel::PointCloud point_cloud;
    bool enough_points = pclPointCloudToSuperqPointCloud(thread_data.object_cloud, point_cloud);
    ROS_INFO("[ConstrainedSuperquadrics] enough_points: %d", enough_points);
    std::vector<SuperqModel::Superquadric> superqs;
    getSuperquadricFromPointCloud(point_cloud, superqs); // CAN I USE THE SAME ESTIMATOR OBJECT FOR ALL THE THREADS?

    std::vector<SuperqModel::Superquadric> superquadrics;
    SuperqModel::Superquadric superquadric;
    Eigen::Vector3d centroid(thread_data.thread_id, thread_data.object_id, 0);
    superquadric.setSuperqCenter(centroid);
    superquadrics.push_back(superquadric);
    return superquadrics;
}

bool ConstrainedSuperquadrics::pclPointCloudToSuperqPointCloud(const pcl::PointCloud<pcl::PointXYZRGB> &object_cloud, SuperqModel::PointCloud &point_cloud)
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

void ConstrainedSuperquadrics::getSuperquadricFromPointCloud(SuperqModel::PointCloud point_cloud, std::vector<SuperqModel::Superquadric> &superqs)
{

    /*  ------------------------------  */
    /*  ------> Compute superq <------  */
    /*  ------------------------------  */


    ROS_INFO("[ConstrainedSuperquadrics]: compute superq");
    if (single_superq_ || object_class_ != "default")
    {
        ROS_INFO("!!!!!");
        estim_.SetStringValue("object_class", object_class_);
        superqs = estim_.computeSuperq(point_cloud);
    }
}
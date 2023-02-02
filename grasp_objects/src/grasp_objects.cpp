#include "grasp_objects/grasp_objects.hpp"

namespace grasp_objects
{
    GraspObjects::GraspObjects(ros::NodeHandle nh) : nodeHandle_(nh)
    {
        ROS_INFO("[GraspObjects] Node started.");
        init();
    }

    GraspObjects::~GraspObjects()
    {
    }

    void GraspObjects::init()
    {
        ROS_INFO("[GraspObjects] init().");
        std::string pointCloudTopicName, cameraInfoTopicName, compressedDepthImageTopicName;
        ros::param::get("grasp_objects/eps_angle", epsAnglePlaneSegmentation_);
        ros::param::get("grasp_objects/distance_threshold", distanceThresholdPlaneSegmentation_);
        ros::param::get("grasp_objects/tol_superq", tolSuperq_);
        ros::param::get("grasp_objects/optimizer_points", optimizerPoints_);
        ros::param::get("grasp_objects/random_sampling", randomSampling_);
        ros::param::get("grasp_objects/max_iter", maxIter_);
        ros::param::get("grasp_objects/minimum_points", minimumPoints_);
        ros::param::get("grasp_objects/fraction_pc", fractionPc_);
        ros::param::get("grasp_objects/threshold_axis", thresholdAxis_);
        ros::param::get("grasp_objects/threshold_section1", thresholdSection1_);
        ros::param::get("grasp_objects/threshold_section2", thresholdSection2_);
        ros::param::get("grasp_objects/object_class", object_class_);
        ros::param::get("grasp_objects/single_superq", single_superq_);
        ros::param::get("grasp_objects/merge_model", merge_model_);
        ros::param::get("grasp_objects/th_points", th_points_);

        nodeHandle_.param("subscribers/point_cloud/topic", pointCloudTopicName, std::string("/xtion/depth/points"));
        nodeHandle_.param("subscribers/camera_info/topic", cameraInfoTopicName, std::string("/xtion/depth/camera_info"));
        // nodeHandle_.param("subscribers/compressed_depth_image/topic", compressedDepthImageTopicName, std::string("/xtion/depth/image_rect"));
        nodeHandle_.param("subscribers/compressed_depth_image/topic", compressedDepthImageTopicName, std::string("/xtion/depth_registered/image_raw"));

        image_transport::ImageTransport it(nodeHandle_);
        ROS_INFO("[GraspObjects] grasp_objects/eps_angle set to %f", epsAnglePlaneSegmentation_);
        ROS_INFO("[GraspObjects] grasp_objects/distance_threshold set to %f", distanceThresholdPlaneSegmentation_);
        ROS_INFO("[GraspObjects] grasp_objects/tol_superq set to %f", tolSuperq_);
        ROS_INFO("[GraspObjects] grasp_objects/optimizer_points set to %d", optimizerPoints_);
        ROS_INFO("[GraspObjects] grasp_objects/random_sampling set to %d", randomSampling_);
        ROS_INFO("[GraspObjects] grasp_objects/max_iter set to %d", maxIter_);
        ROS_INFO("[GraspObjects] grasp_objects/minimum_points set to %d", minimumPoints_);
        ROS_INFO("[GraspObjects] grasp_objects/fraction_pc set to %d", fractionPc_);
        ROS_INFO("[GraspObjects] grasp_objects/threshold_axis set to %f", thresholdAxis_);
        ROS_INFO("[GraspObjects] grasp_objects/threshold_section1 set to %f", thresholdSection1_);
        ROS_INFO("[GraspObjects] grasp_objects/threshold_section2 set to %f", thresholdSection2_);
        ROS_INFO("[GraspObjects] grasp_objects/object_class set to %s", object_class_.c_str());
        ROS_INFO("[GraspObjects] grasp_objects/single_superq set to %d", single_superq_);
        ROS_INFO("[GraspObjects] grasp_objects/merge_model set to %d", merge_model_);
        ROS_INFO("[GraspObjects] grasp_objects/th_points set to %d", th_points_);
        ROS_INFO("[GraspObjects] subscribers/point_cloud/topic set to %s", pointCloudTopicName.c_str());
        ROS_INFO("[GraspObjects] subscribers/camera_info/topic set to %s", cameraInfoTopicName.c_str());

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

        ROS_INFO("[GraspObjects] Waiting to get the camera info...");
        sensor_msgs::CameraInfoConstPtr cameraInfoMsg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(cameraInfoTopicName);
        model_.fromCameraInfo(cameraInfoMsg);

        setCameraParams(*cameraInfoMsg);

        // pointCloudSubscriber_ = nodeHandle_.subscribe(pointCloudTopicName, 20, &GraspObjects::pointCloudCallback, this);
        // compressedDepthImageSubscriber_ = it.subscribe(compressedDepthImageTopicName, 10, &GraspObjects::compressedDepthImageCallback, this, image_transport::TransportHints("compressedDepth"));

        compressedDepthImageSubscriber_ = it.subscribe(compressedDepthImageTopicName, 10, &GraspObjects::compressedDepthImageCallback, this, image_transport::TransportHints("raw"));

        outPointCloudPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("/transformed_cloud", 20);
        outPointCloudAddedPointsPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("/added_points", 20);
        outPointCloudSuperqsPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("/grasp_objects/superquadrics_cloud", 20);
        superquadricsPublisher_ = nodeHandle_.advertise<sharon_msgs::SuperquadricMultiArray>("/grasp_objects/superquadrics", 20);
        graspPosesPublisher_ = nodeHandle_.advertise<geometry_msgs::PoseArray>("/grasp_objects/poses", 20);
        bbox3dPublisher_ = nodeHandle_.advertise<visualization_msgs::MarkerArray>("/grasp_objects/bbox3d", 20);

        serviceActivateSuperquadricsComputation_ = nodeHandle_.advertiseService("/grasp_objects/activate_superquadrics_computation", &GraspObjects::activateSuperquadricsComputation, this);
        serviceComputeGraspPoses_ = nodeHandle_.advertiseService("/grasp_objects/compute_grasp_poses", &GraspObjects::computeGraspPoses, this);
        serviceGetSuperquadrics_ = nodeHandle_.advertiseService("/grasp_objects/get_superquadrics", &GraspObjects::getSuperquadrics, this);
        serviceGetBboxesSuperquadrics_ = nodeHandle_.advertiseService("/grasp_objects/get_bboxes_superquadrics", &GraspObjects::getBboxes, this);
    }

    bool GraspObjects::createBoundingBox2DFromSuperquadric(const sharon_msgs::Superquadric &superq, sharon_msgs::BoundingBox &bbox)
    {
        // Bbox 3d wrt object's frame

        pcl::PointCloud<pcl::PointXYZ>::Ptr bbox3d(new pcl::PointCloud<pcl::PointXYZ>);
        for (float x = -1; x <= 1; x += 2)
        {
            for (float y = -1; y <= 1; y += 2)
            {
                for (float z = -1; z <= 1; z += 2)
                {
                    pcl::PointXYZ p;

                    p.x = x * superq.a1;
                    p.y = y * superq.a2;
                    p.z = z * superq.a3;

                    bbox3d->push_back(*(pcl::PointXYZ *)(&p));
                }
            }
        }

        // Bbox 3d wrt world frame
        Eigen::AngleAxisf rollAngle(superq.roll, Eigen::Vector3f::UnitZ());
        Eigen::AngleAxisf yawAngle(superq.pitch, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf pitchAngle(superq.yaw, Eigen::Vector3f::UnitZ());

        Eigen::Quaternionf q = rollAngle * yawAngle * pitchAngle;
        Eigen::Matrix3f rotationMatrix = q.matrix();

        Eigen::Vector3f v(superq.x, superq.y, superq.z);
        Eigen::Matrix4f transform = Eigen::Affine3f(Eigen::Translation3f(v)).matrix();

        transform.block<3, 3>(0, 0) = rotationMatrix;
        pcl::transformPointCloud(*bbox3d, *bbox3d, transform);

        Eigen::Matrix4f outMatrixTransform;
        pcl_ros::transformAsMatrix(transformCameraWrtBase_, outMatrixTransform);

        pcl::transformPointCloud(*bbox3d, *bbox3d, outMatrixTransform.inverse());

        visualization_msgs::MarkerArray markerArray;
        visualization_msgs::Marker marker;

        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.header.frame_id = "xtion_depth_optical_frame";
        marker.header.stamp = ros::Time::now();

        ROS_INFO("Bbox has %d points", bbox3d->points.size());
        for (int idx = 0; idx <= bbox3d->points.size() - 1; idx += 1)
        {
            if (idx % 2 == 0)
            {
                // marker.id = idx;
                ROS_INFO("Line %d and %d", idx, idx + 1);
                geometry_msgs::Point pointRos;
                pointRos.x = bbox3d->points[idx].x;
                pointRos.y = bbox3d->points[idx].y;
                pointRos.z = bbox3d->points[idx].z;
                marker.points.push_back(pointRos);

                pointRos.x = bbox3d->points[idx + 1].x;
                pointRos.y = bbox3d->points[idx + 1].y;
                pointRos.z = bbox3d->points[idx + 1].z;
                marker.points.push_back(pointRos);

                marker.pose.orientation.w = 1.0;
            }
            if (idx <= 1 || ((idx >= 4) && (idx < 6)))
            {
                geometry_msgs::Point pointRos;
                pointRos.x = bbox3d->points[idx].x;
                pointRos.y = bbox3d->points[idx].y;
                pointRos.z = bbox3d->points[idx].z;
                marker.points.push_back(pointRos);

                pointRos.x = bbox3d->points[idx + 2].x;
                pointRos.y = bbox3d->points[idx + 2].y;
                pointRos.z = bbox3d->points[idx + 2].z;
                marker.points.push_back(pointRos);

                marker.pose.orientation.w = 1.0;
            }
            if (idx <= 3)
            {
                geometry_msgs::Point pointRos;

                pointRos.x = bbox3d->points[idx].x;
                pointRos.y = bbox3d->points[idx].y;
                pointRos.z = bbox3d->points[idx].z;
                marker.points.push_back(pointRos);

                pointRos.x = bbox3d->points[idx + 4].x;
                pointRos.y = bbox3d->points[idx + 4].y;
                pointRos.z = bbox3d->points[idx + 4].z;
                marker.points.push_back(pointRos);

                marker.pose.orientation.w = 1.0;
            }
        }
        marker.scale.x = 0.005;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        markerArray.markers.push_back(marker);

        bbox3dPublisher_.publish(markerArray);

        int tlx1 = width_, tly1 = height_, brx1 = 0, bry1 = 0;
        for (size_t idx = 0; idx < bbox3d->size(); idx++)
        {
            pcl::PointXYZ p(bbox3d->points[idx].x, bbox3d->points[idx].y, bbox3d->points[idx].z);
            int xpixel, ypixel;
            // yInfo()<<"xpixel: "<<xpixel<<" ypixel: "<<ypixel;
            getPixelCoordinates(p, xpixel, ypixel);
            if (xpixel < tlx1)
            {
                tlx1 = xpixel;
            }
            if (xpixel > brx1)
            {
                brx1 = xpixel;
            }
            if (ypixel < tly1)
            {
                tly1 = ypixel;
            }
            if (ypixel > bry1)
            {
                bry1 = ypixel;
            }
        }
        bbox.tlx = tlx1;
        bbox.tly = tly1;
        bbox.brx = brx1;
        bbox.bry = bry1;
        bbox.id = superq.id;

        return true;
    }

    void GraspObjects::getPixelCoordinates(const pcl::PointXYZ &p, int &xpixel, int &ypixel)
    {
        ROS_INFO("p: %f %f %f ", p.x, p.y, p.z);
        // yInfo() << "intrinsics.focalLengthX:" << intrinsics.focalLengthX << " intrinsics.principalPointX: " << intrinsics.principalPointX;
        // yInfo() << "intrinsics.focalLengthY:" << intrinsics.focalLengthY << " intrinsics.principalPointY: " << intrinsics.principalPointY;

        ROS_INFO("focalLengthX_: %f principalPointX_: %f", focalLengthX_, principalPointX_);
        ROS_INFO("%f", p.x * focalLengthX_);
        xpixel = (int)(p.x * focalLengthX_ / p.z + principalPointX_);
        ypixel = (int)(p.y * focalLengthY_ / p.z + principalPointY_);
        ROS_INFO("xpixel: %d ypixel %d", xpixel, ypixel);

        // yInfo() << "xpixel: " << xpixel << "ypixel: " << ypixel;
        if (xpixel > width_)
            xpixel = width_;
        if (ypixel > height_)
            ypixel = height_;
        if (xpixel < 0)
            xpixel = 0;
        if (ypixel < 0)
            ypixel = 0;
    }

    bool GraspObjects::getSuperquadrics(sharon_msgs::GetSuperquadrics::Request &req, sharon_msgs::GetSuperquadrics::Response &res)
    {
        ROS_INFO("[GraspObjects] GetSuperquadrics().");
        sharon_msgs::SuperquadricMultiArray superquadrics;
        geometry_msgs::PoseArray graspingPoses;

        res.superquadrics = superquadricsMsg_;

        return true;
    }

    bool GraspObjects::getBboxes(sharon_msgs::GetBboxes::Request &req, sharon_msgs::GetBboxes::Response &res)
    {
        ROS_INFO("[GraspObjects] getBboxes().");
        sharon_msgs::BoundingBoxes boundingBoxes;
        boundingBoxes.header.stamp = ros::Time::now();

        for (int i = 0; i < superquadricsMsg_.superquadrics.size(); i++)
        {
            ROS_INFO("[GraspObjects] SQ: %d", i);
            sharon_msgs::BoundingBox bbox;
            createBoundingBox2DFromSuperquadric(superquadricsMsg_.superquadrics[i], bbox);
            boundingBoxes.bounding_boxes.push_back(bbox);
        }
        res.bounding_boxes = boundingBoxes;
    }

    bool GraspObjects::computeGraspPoses(sharon_msgs::ComputeGraspPoses::Request &req, sharon_msgs::ComputeGraspPoses::Response &res)
    {
        ROS_INFO("[GraspObjects] computeGraspPoses().");
        geometry_msgs::PoseArray graspingPoses;

        int idx = -1;
        ROS_INFO("superquadricObjects_.size(): %d", superquadricObjects_.size());
        for (int i = 0; i < superquadricObjects_.size(); i++)
        {
            ROS_INFO("superquadricObjects_[%d].label: %d, id: %d", i, superquadricObjects_[i].label, req.id);
            if (superquadricObjects_[i].label == req.id)
            {
                idx = i;
                break;
            }
        }
        if (idx == -1)
        {
            res.success = false;
        }
        else
        {
            computeGraspingPosesObject(superquadricObjects_[idx].superqs, graspingPoses);
            res.poses = graspingPoses;
            res.success = true;
        }

        graspPosesPublisher_.publish(graspingPoses);

        return true;
    }

    void GraspObjects::computeGraspingPosesObject(const std::vector<SuperqModel::Superquadric> &superqs, geometry_msgs::PoseArray &graspingPoses)
    {

        graspingPoses.header.frame_id = "base_footprint";
        graspingPoses.header.stamp = ros::Time::now();

        auto params = superqs[0].getSuperqParams();

        Eigen::AngleAxisf rollAngle(params[8], Eigen::Vector3f::UnitZ());
        Eigen::AngleAxisf yawAngle(params[9], Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf pitchAngle(params[10], Eigen::Vector3f::UnitZ());

        Eigen::Quaternionf q = rollAngle * yawAngle * pitchAngle;

        KDL::Rotation rot_aux = KDL::Rotation::Quaternion(q.x(), q.y(), q.z(), q.w());
        KDL::Frame frame_object_wrt_world(rot_aux);
        frame_object_wrt_world.p[0] = params[5];
        frame_object_wrt_world.p[1] = params[6];
        frame_object_wrt_world.p[2] = params[7];

        KDL::Frame frame_grasping_wrt_world;
        std::cout << "axes length: " << 2 * params[0] << " " << 2 * params[1] << ": " << 2 * params[2] << std::endl;

        float step = 0.008;
        if (2 * params[2] <= MAX_OBJECT_WIDTH_GRASP)
        {
            KDL::Vector zobject;
            KDL::Vector yobject;
            KDL::Vector xobject;
            KDL::Rotation rot;
            KDL::Frame frameTCP, frame_grasping_wrt_object;
            std::vector<double> tcpX;

            for (float yaxes = -1.0; yaxes <= 1.0; yaxes += 2)
            {
                yobject = KDL::Vector(0, yaxes, 0);
                for (float x = 0; x <= params[0] / 1.4; x += step)
                {
                    float xaxes = 1.0;
                    zobject = KDL::Vector(xaxes, 0.0, 0.0);
                    xobject = yobject * zobject;
                    rot = KDL::Rotation(xobject, yobject, zobject);
                    frameTCP = KDL::Frame(rot);
                    frameTCP = KDL::Frame(KDL::Rotation::RotZ(M_PI / 2.0)) * frameTCP;
                    frameTCP = KDL::Frame(KDL::Rotation::RotX(M_PI / 2.0)) * frameTCP;
                    frame_grasping_wrt_object = frameTCP;
                    for (int sign = 1.0; sign >= -1; sign -= 2.0)
                    {
                        frame_grasping_wrt_object.p[0] = sign * x;
                        frame_grasping_wrt_object.p[1] = -yaxes * params[1];
                        frame_grasping_wrt_object.p[2] = 0;
                        frame_grasping_wrt_world = frame_object_wrt_world * frame_grasping_wrt_object;
                        // Check if is trying to grasp from the bottom of the objet
                        KDL::Vector unitx = frame_grasping_wrt_world.M.UnitX();
                        KDL::Vector unity = frame_grasping_wrt_world.M.UnitY();
                        KDL::Vector unitz = frame_grasping_wrt_world.M.UnitZ();

                        KDL::Vector axesz(1, 0, 0);
                        float angle = atan2((unitz * axesz).Norm(), dot(unitz, axesz));
                        // printf("angle: %f\n", angle * M_1_PI / 180.0);
                        if (angle > 20 * M_PI / 180.0)
                        {
                            geometry_msgs::Pose pose;
                            tf::poseKDLToMsg(frame_grasping_wrt_world, pose);
                            // printf("%f %f %f %f %f %f\n", tcpX[0], tcpX[1], tcpX[2], tcpX[3], tcpX[4], tcpX[5]);
                            graspingPoses.poses.push_back(pose);
                        }
                    }
                }
            }

            float yaxes = 1.0;

            for (float xaxes = -1.0; xaxes <= 1.0; xaxes += 2)
            {
                xobject = KDL::Vector(xaxes, 0, 0);
                for (float y = 0; y <= params[1] / 1.4; y += step)
                // for (float y = -params[1] / 2.0; y <= params[1] / 2.0; y += step)
                {
                    zobject = KDL::Vector(0, yaxes, 0.0);
                    yobject = zobject * xobject;
                    rot = KDL::Rotation(xobject, yobject, zobject);
                    frameTCP = KDL::Frame(rot);
                    frameTCP = KDL::Frame(KDL::Rotation::RotX(-M_PI / 2.0)) * frameTCP;

                    for (int sign = 1; sign >= -1; sign -= 2)
                    {
                        frame_grasping_wrt_object = frameTCP;
                        frame_grasping_wrt_object.p[0] = -xaxes * params[0];
                        frame_grasping_wrt_object.p[1] = sign * y;
                        frame_grasping_wrt_object.p[2] = 0;
                        frame_grasping_wrt_world = frame_object_wrt_world * frame_grasping_wrt_object;
                        // Check if is trying to grasp from the bottom of the objet
                        KDL::Vector unitx = frame_grasping_wrt_world.M.UnitX();
                        KDL::Vector unity = frame_grasping_wrt_world.M.UnitY();
                        KDL::Vector unitz = frame_grasping_wrt_world.M.UnitZ();

                        KDL::Vector axesz(1, 0, 0);
                        float angle = atan2((unitz * axesz).Norm(), dot(unitz, axesz));
                        // printf("angle: %f\n", angle * 180.0 / M_PI);
                        if (angle > 20 * M_PI / 180.0)
                        {
                            geometry_msgs::Pose pose;
                            tf::poseKDLToMsg(frame_grasping_wrt_world, pose);
                            // printf("%f %f %f %f %f %f\n", tcpX[0], tcpX[1], tcpX[2], tcpX[3], tcpX[4], tcpX[5]);
                            graspingPoses.poses.push_back(pose);
                        }
                    }
                }
            }
        }

        if (2 * params[1] <= MAX_OBJECT_WIDTH_GRASP)
        {
            KDL::Vector zobject;
            KDL::Vector yobject;
            KDL::Vector xobject;
            KDL::Rotation rot;
            KDL::Frame frameTCP, frame_grasping_wrt_object;
            std::vector<double> tcpX;

            for (float yaxes = -1.0; yaxes <= 1.0; yaxes += 2)
            {
                yobject = KDL::Vector(0, yaxes, 0);
                for (float x = 0; x <= params[0] / 1.4; x += step)
                {
                    float xaxes = -1.0;
                    zobject = KDL::Vector(0.0, 0.0, xaxes);
                    xobject = yobject * zobject;
                    rot = KDL::Rotation(xobject, yobject, zobject);
                    frameTCP = KDL::Frame(rot);
                    frameTCP = KDL::Frame(KDL::Rotation::RotZ(M_PI / 2.0)) * frameTCP;
                    frameTCP = KDL::Frame(KDL::Rotation::RotX(M_PI / 2.0)) * frameTCP;
                    frame_grasping_wrt_object = frameTCP;
                    for (int sign = 1.0; sign >= -1; sign -= 2.0)
                    {
                        frame_grasping_wrt_object.p[0] = sign * x;
                        frame_grasping_wrt_object.p[1] = 0;
                        frame_grasping_wrt_object.p[2] = yaxes * params[2];
                        frame_grasping_wrt_world = frame_object_wrt_world * frame_grasping_wrt_object;
                        // Check if is trying to grasp from the bottom of the objet
                        KDL::Vector unitx = frame_grasping_wrt_world.M.UnitX();
                        KDL::Vector unity = frame_grasping_wrt_world.M.UnitY();
                        KDL::Vector unitz = frame_grasping_wrt_world.M.UnitZ();

                        KDL::Vector axesz(1, 0, 0);
                        float angle = atan2((unitz * axesz).Norm(), dot(unitz, axesz));
                        // printf("angle: %f\n", angle * M_1_PI / 180.0);
                        if (angle > 20 * M_PI / 180.0)
                        {
                            geometry_msgs::Pose pose;
                            tf::poseKDLToMsg(frame_grasping_wrt_world, pose);
                            // printf("%f %f %f %f %f %f\n", tcpX[0], tcpX[1], tcpX[2], tcpX[3], tcpX[4], tcpX[5]);
                            graspingPoses.poses.push_back(pose);
                        }
                    }
                }
            }

            float yaxes = 1.0;

            for (float xaxes = -1.0; xaxes <= 1.0; xaxes += 2)
            {
                xobject = KDL::Vector(xaxes, 0, 0);
                for (float y = 0; y <= params[1] / 1.4; y += step)
                // for (float y = -params[1] / 2.0; y <= params[1] / 2.0; y += step)
                {
                    zobject = KDL::Vector(0, yaxes, 0.0);
                    yobject = zobject * xobject;
                    rot = KDL::Rotation(xobject, yobject, zobject);
                    frameTCP = KDL::Frame(rot);
                    frameTCP = KDL::Frame(KDL::Rotation::RotX(-M_PI / 2.0)) * frameTCP;

                    for (int sign = 1; sign >= -1; sign -= 2)
                    {
                        frame_grasping_wrt_object = frameTCP;
                        frame_grasping_wrt_object.p[0] = -xaxes * params[0];
                        frame_grasping_wrt_object.p[1] = sign * y;
                        frame_grasping_wrt_object.p[2] = 0;
                        frame_grasping_wrt_world = frame_object_wrt_world * frame_grasping_wrt_object;
                        // Check if is trying to grasp from the bottom of the objet
                        KDL::Vector unitx = frame_grasping_wrt_world.M.UnitX();
                        KDL::Vector unity = frame_grasping_wrt_world.M.UnitY();
                        KDL::Vector unitz = frame_grasping_wrt_world.M.UnitZ();

                        KDL::Vector axesz(1, 0, 0);
                        float angle = atan2((unitz * axesz).Norm(), dot(unitz, axesz));
                        printf("angle: %f\n", angle * 180.0 / M_PI);
                        if (angle > 20 * M_PI / 180.0)
                        {

                            geometry_msgs::Pose pose;
                            tf::poseKDLToMsg(frame_grasping_wrt_world, pose);
                            // printf("%f %f %f %f %f %f\n", tcpX[0], tcpX[1], tcpX[2], tcpX[3], tcpX[4], tcpX[5]);
                            graspingPoses.poses.push_back(pose);
                        }
                    }
                }
            }
        }
        
    }

    bool GraspObjects::activateSuperquadricsComputation(sharon_msgs::ActivateSupercuadricsComputation::Request &req, sharon_msgs::ActivateSupercuadricsComputation::Response &res)
    {

        mtxActivate_.lock();
        activate_ = req.activate;
        count_ = 0;
	ROS_INFO("[GraspObjects] Activate superquadrics Computation: %d", activate_);
        res.success = true;
        mtxActivate_.unlock();
        return true;
    }

    void GraspObjects::supervoxelOversegmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr &inputPointCloud, pcl::PointCloud<pcl::PointXYZL>::Ptr &lccp_labeled_cloud)
    {

        // ------------------------------- Compute normals of the the input cloud ------------------------------------------------- //

        // Create the normal estimation class, and pass the input dataset to it
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(inputPointCloud);

        // Create an empty kdtree representation, and pass it to the normal estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        ne.setSearchMethod(tree);
        // Output datasets
        pcl::PointCloud<pcl::Normal>::Ptr input_normals_ptr(new pcl::PointCloud<pcl::Normal>);

        // Use all neighbors in a sphere of radius 3cm (TODO: Pass as parameter)
        ne.setRadiusSearch(0.035);
        // Compute the features
        ne.compute(*input_normals_ptr);

        // TODO: Change to ROS params
        float voxel_resolution = 0.01f;
        float seed_resolution = 0.005f;
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

        pcl::SupervoxelClustering<pcl::PointXYZ> super(voxel_resolution, seed_resolution);
        super.setUseSingleCameraTransform(use_single_cam_transform);
        super.setInputCloud(inputPointCloud);
        super.setNormalCloud(input_normals_ptr);
        super.setColorImportance(color_importance);
        super.setSpatialImportance(spatial_importance);
        super.setNormalImportance(normal_importance);
        std::map<std::uint32_t, pcl::Supervoxel<pcl::PointXYZ>::Ptr> supervoxel_clusters;

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

        // pcl::io::savePCDFile ("svcloud.pcd", *sv_centroid_normal_cloud, true);

        PCL_INFO("Starting Segmentation\n");

        pcl::LCCPSegmentation<pcl::PointXYZ> lccp;
        lccp.reset();
        lccp.setConcavityToleranceThreshold(concavity_tolerance_threshold);
        lccp.setSanityCheck(use_sanity_criterion);
        lccp.setSmoothnessCheck(true, voxel_resolution, seed_resolution, smoothness_threshold);
        lccp.setKFactor(k_factor);
        lccp.setInputSupervoxels(supervoxel_clusters, supervoxel_adjacency);
        lccp.setMinSegmentSize(min_segment_size);
        lccp.segment();

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
            if (idx > 0)
            {

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

    void GraspObjects::compressedDepthImageCallback(const sensor_msgs::ImageConstPtr &depth_msg)
    {
        // ROS_INFO("[GraspObjects] Receiving the compressed depth image");
        bool activate;
        mtxActivate_.lock();
        activate = activate_;
        mtxActivate_.unlock();
        if (activate && count_<2)
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

            pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud;
            supervoxelOversegmentation(cloud_without_table, lccp_labeled_cloud);

            // Convert to ROS data type
            pcl::toPCLPointCloud2(*lccp_labeled_cloud, *cloudFiltered);
            pcl_conversions::moveFromPCL(*cloudFiltered, pcOut);
            pcOut.header.frame_id = "/base_footprint";
            outPointCloudPublisher_.publish(pcOut);

            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudSuperquadric(new pcl::PointCloud<pcl::PointXYZRGBA>);
            superquadricObjects_.clear();
            if (lccp_labeled_cloud->points.size() != 0)
            {

                sharon_msgs::SuperquadricMultiArray aux;
                superquadricsMsg_ = aux;
                superquadricsMsg_.header.stamp = ros::Time::now();
                updateDetectedObjectsPointCloud(lccp_labeled_cloud);

                std::vector<std::vector<double>> graspingPoses;
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr allPoints;

                for (unsigned int idx = 0; idx < detectedObjects_.size(); idx++)
                {

                    // addPointsToObjectCloud(idx, 0.5, 0.02, 0.0005);
                    // ROS_INFO("addPointsToObjectCloud DONE");
                    // for(int i=0; i<detectedObjects_[idx].object_cloud.points.size(); i++){
                    //     pcl::PointXYZRGB p = detectedObjects_[idx].object_cloud.points[i];
                    //     allPoints->points.push_back(p);
                    // }

                    SuperqModel::PointCloud point_cloud;
                    pclPointCloudToSuperqPointCloud(detectedObjects_[idx].object_cloud, point_cloud);
                    ROS_INFO("pointCloud points: %d", point_cloud.n_points);
                    std::vector<SuperqModel::Superquadric> superqs;
                    getSuperquadricFromPointCloud(point_cloud, superqs);
                    sharon_msgs::Superquadric superquadric;
                    auto params = superqs[0].getSuperqParams();
                    superquadric.id = detectedObjects_[idx].label;
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
                    superquadricsMsg_.superquadrics.push_back(superquadric);
                }

                ROS_INFO("AQUI!!");
                // Convert to ROS data type
                sensor_msgs::PointCloud2 pcOutSupeqs;
                pcl::PCLPointCloud2 *cloudAux = new pcl::PCLPointCloud2;

                pcl::toPCLPointCloud2(*cloudSuperquadric, *cloudAux);
                pcl_conversions::moveFromPCL(*cloudAux, pcOutSupeqs);
                pcOutSupeqs.header.frame_id = "/base_footprint";
                outPointCloudSuperqsPublisher_.publish(pcOutSupeqs);
                superquadricsPublisher_.publish(superquadricsMsg_);

                // Convert to ROS data type
                // pcl::PCLPointCloud2 *allPointsPC2 = new pcl::PCLPointCloud2;
                // pcl::toPCLPointCloud2(*allPoints, *allPointsPC2);
                // sensor_msgs::PointCloud2 pcOutAllPoints;
                // pcl_conversions::moveFromPCL(*allPointsPC2, pcOutAllPoints);
                // pcOutAllPoints.header.frame_id = "/base_footprint";

                // outPointCloudAddedPointsPublisher_.publish(pcOutAllPoints);
            }
        }
    }

    void GraspObjects::addPointsToObjectCloud(int idx, float minHeight, float distanceTop, float distanceBtwPoints)
    {

        pcl::PointXYZRGB minPt, maxPt;
        pcl::getMinMax3D(detectedObjects_[idx].object_cloud, minPt, maxPt);
        std::cout << "Max z: " << maxPt.z << std::endl;
        std::cout << "Min z: " << minPt.z << std::endl;
        std::vector<int> indexes;
        ROS_INFO("%d", detectedObjects_[idx].object_cloud.size());

        if (maxPt.z > minHeight)
        {
            for (int i = 0; i < detectedObjects_[idx].object_cloud.size(); i++)
            {
                pcl::PointXYZRGB p = detectedObjects_[idx].object_cloud.points[i];
                if (p.z >= (maxPt.z - distanceTop))
                {
                    indexes.push_back(i);
                    std::cout << "we have a point z: " << p.z << std::endl;
                }
            }
            for (int j = 0; j < indexes.size(); j++)
            {
                pcl::PointXYZRGB auxp;
                pcl::PointXYZRGB p = detectedObjects_[idx].object_cloud.points[indexes[j]];
                auxp.x = p.x;
                auxp.y = p.y;
                auxp.z = p.z;
                auxp.r = p.r;
                auxp.g = p.g;
                auxp.b = p.b;
                auxp.z += 20*distanceBtwPoints;
                while (auxp.z >= minHeight)
                {
                    detectedObjects_[idx].object_cloud.points.push_back(auxp);
                    auxp.z -= distanceBtwPoints;
                }
            }
            ROS_INFO("%d", detectedObjects_[idx].object_cloud.size());
        }
    }

    // void GraspObjects::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &pointCloud_msg)
    // {
    //     bool activate;
    //     mtxActivate_.lock();
    //     activate = activate_;
    //     mtxActivate_.unlock();
    //     if (activate)
    //     {
    //         sensor_msgs::PointCloud2 pcOut;
    //         pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_without_table(new pcl::PointCloud<pcl::PointXYZRGB>);

    //         listener_.lookupTransform("/base_footprint", "/xtion_rgb_optical_frame", ros::Time(0), transformCameraWrtBase_);

    //         pcl_ros::transformPointCloud(std::string("/base_footprint"), transformCameraWrtBase_, *pointCloud_msg, pcOut);

    //         pcl::PCLPointCloud2 *cloudFiltered = new pcl::PCLPointCloud2;
    //         pcl::PCLPointCloud2ConstPtr cloudFilteredPtr(cloudFiltered);
    //         // Convert to PCL data type
    //         pcl_conversions::toPCL(pcOut, *cloudFiltered);

    //         // Perform the actual filtering
    //         pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    //         sor.setInputCloud(cloudFilteredPtr);
    //         sor.setLeafSize(0.005f, 0.005f, 0.005f);
    //         sor.filter(*cloudFiltered);

    //         // Create the filtering object
    //         pcl::PassThrough<pcl::PCLPointCloud2> pass;
    //         pass.setInputCloud(cloudFilteredPtr);
    //         pass.setFilterFieldName("x");
    //         pass.setFilterLimits(0.0, 1.5);
    //         // pass.setFilterLimitsNegative (true);
    //         pass.filter(*cloudFiltered);

    //         pcl::fromPCLPointCloud2(*cloudFiltered, *cloud_without_table);

    //         // Coefficients and inliners objects for tge ransac plannar model
    //         pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    //         pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    //         // Create the segmentation object
    //         pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    //         // Optional
    //         seg.setOptimizeCoefficients(true);
    //         // Mandatory
    //         seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
    //         seg.setMethodType(pcl::SAC_RANSAC);
    //         seg.setMaxIterations(2000);
    //         seg.setDistanceThreshold(distanceThresholdPlaneSegmentation_);
    //         seg.setAxis(Eigen::Vector3f::UnitX());
    //         seg.setEpsAngle(epsAnglePlaneSegmentation_);

    //         seg.setInputCloud(cloud_without_table);
    //         seg.segment(*inliers, *coefficients);
    //         // Create the filtering object
    //         pcl::ExtractIndices<pcl::PointXYZRGB> extract;

    //         extract.setInputCloud(cloud_without_table);
    //         extract.setIndices(inliers);
    //         extract.setNegative(true); // Extract the inliers
    //         extract.filter(*cloud_without_table);

    //         pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud;
    //         supervoxelOversegmentation(cloud_without_table, lccp_labeled_cloud);

    //         // Convert to ROS data type
    //         pcl::toPCLPointCloud2(*lccp_labeled_cloud, *cloudFiltered);
    //         pcl_conversions::moveFromPCL(*cloudFiltered, pcOut);
    //         pcOut.header.frame_id = "/base_footprint";
    //         outPointCloudPublisher_.publish(pcOut);

    //         pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudSuperquadric(new pcl::PointCloud<pcl::PointXYZRGBA>);
    //         superquadricObjects_.clear();
    //         if (lccp_labeled_cloud->points.size() != 0)
    //         {

    //             sharon_msgs::SuperquadricMultiArray aux;
    //             superquadricsMsg_ = aux;
    //             superquadricsMsg_.header.stamp = ros::Time::now();
    //             updateDetectedObjectsPointCloud(lccp_labeled_cloud);

    //             std::vector<std::vector<double>> graspingPoses;

    //             for (unsigned int idx = 0; idx < detectedObjects_.size(); idx++)
    //             {
    //                 SuperqModel::PointCloud point_cloud;
    //                 pclPointCloudToSuperqPointCloud(detectedObjects_[idx].object_cloud, point_cloud);
    //                 ROS_INFO("pointCloud points: %d", point_cloud.n_points);
    //                 std::vector<SuperqModel::Superquadric> superqs;
    //                 getSuperquadricFromPointCloud(point_cloud, superqs);
    //                 sharon_msgs::Superquadric superquadric;
    //                 auto params = superqs[0].getSuperqParams();
    //                 superquadric.id = detectedObjects_[idx].label;
    //                 superquadric.a1 = params[0];
    //                 superquadric.a2 = params[1];
    //                 superquadric.a3 = params[2];
    //                 superquadric.e1 = params[3];
    //                 superquadric.e2 = params[4];
    //                 superquadric.x = params[5];
    //                 superquadric.y = params[6];
    //                 superquadric.z = params[7];
    //                 superquadric.roll = params[8];
    //                 superquadric.pitch = params[9];
    //                 superquadric.yaw = params[10];

    //                 pcl::PointCloud<pcl::PointXYZRGBA>::Ptr auxCloudSuperquadric(new pcl::PointCloud<pcl::PointXYZRGBA>);

    //                 // This is only for visulazition of the superquadrics
    //                 createPointCloudFromSuperquadric(superqs, auxCloudSuperquadric, idx);
    //                 *cloudSuperquadric += *auxCloudSuperquadric;

    //                 ObjectSuperquadric objectSuperquadric;
    //                 objectSuperquadric.label = detectedObjects_[idx].label;
    //                 objectSuperquadric.superqs = superqs;
    //                 objectSuperquadric.cloud = *auxCloudSuperquadric;

    //                 superquadricObjects_.push_back(objectSuperquadric);
    //                 superquadricsMsg_.superquadrics.push_back(superquadric);
    //             }
    //             // Convert to ROS data type
    //             sensor_msgs::PointCloud2 pcOutSupeqs;
    //             pcl::PCLPointCloud2 *cloudAux = new pcl::PCLPointCloud2;

    //             pcl::toPCLPointCloud2(*cloudSuperquadric, *cloudAux);
    //             pcl_conversions::moveFromPCL(*cloudAux, pcOutSupeqs);
    //             pcOutSupeqs.header.frame_id = "/base_footprint";
    //             outPointCloudSuperqsPublisher_.publish(pcOutSupeqs);
    //             superquadricsPublisher_.publish(superquadricsMsg_);
    //         }
    //     }
    // }

    void GraspObjects::setCameraParams(const sensor_msgs::CameraInfo &cameraInfo_msg)
    {
        height_ = cameraInfo_msg.height;
        width_ = cameraInfo_msg.width;
        focalLengthX_ = cameraInfo_msg.P[0];
        focalLengthY_ = cameraInfo_msg.P[5];
        principalPointX_ = cameraInfo_msg.P[2];
        principalPointY_ = cameraInfo_msg.P[6];

        ROS_INFO("[GraspObjects] Camera params ----");
        ROS_INFO("[GraspObjects] Width: %d", width_);
        ROS_INFO("[GraspObjects] Height: %d", height_);
        ROS_INFO("[GraspObjects] Focal Length: (%f, %f)", focalLengthX_, focalLengthY_);
        ROS_INFO("[GraspObjects] Principal point: (%f, %f)", principalPointX_, principalPointY_);
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

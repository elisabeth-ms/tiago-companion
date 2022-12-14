#include "demo_sharon/demo_sharon.hpp"

namespace demo_sharon
{
    DemoSharon::DemoSharon(ros::NodeHandle nh) : nodeHandle_(nh)
    {
        ROS_INFO("[DemoSharon] Node started.");

        // Need it asynspinner for moveit
        ros::AsyncSpinner spinner(2);
        spinner.start();
        init(); // Tiagos head and torso are at the initial positions

        if (useAsr_ && !useGlasses_)
        {
            demoOnlyASR();
        }

        else if (!useAsr_ && useGlasses_)
        {
            demoOnlyGlasses();
        }
        else if (useAsr_ && useGlasses_)
        {
            demoGlassesASR();
        }
    }

    DemoSharon::~DemoSharon()
    {
    }

    // void DemoSharon::orderGraspingPoses(bool rightArm, const geometry_msgs::PoseArray &graspingPoses, geometry_msgs::PoseArray &orderedGraspingPoses){
    //     if(rightArm){

    //     }
    // }

    void DemoSharon::demoGlassesASR()
    {
        state_ = INITIALIZE;
        bool firstInState = true;
        while (ros::ok())
        {

            switch (state_)
            {

            case INITIALIZE:
            {

                waitingForGlassesCommand_ = false;
                glassesCommandReceived_ = false;

                waitingForAsrCommand_ = false;
                asrCommandReceived_ = false;

                removeCollisionObjectsPlanningScene();

                geometry_msgs::Pose tablePose;
                tablePose.orientation.w = 1.0;
                tablePose.position.x = tablePosition_[0];
                tablePose.position.y = tablePosition_[1];
                tablePose.position.z = tablePosition_[2];

                addTablePlanningScene(tableDimensions_, tablePose, "table1");

                tablePose.position.x = tablePosition2_[0];
                tablePose.position.y = tablePosition2_[1];
                tablePose.position.z = tablePosition2_[2];
                addTablePlanningScene(tableDimensions2_, tablePose, "table2");

                if (!initializeRightArmPosition(initRightArmPositions_))
                {
                    return;
                }

                if (!initializeLeftArmPosition(initLeftArmPositions_))
                {
                    return;
                }

                initializeTorsoPosition(initTorsoPosition_);

                initializeHeadPosition(initHeadPositions_);

                ros::Duration(1.5).sleep(); // sleep for 2 seconds

                ROS_INFO("[DemoSharon] Start the computation of the superquadrics.");

                // Start computation of the superquadrics from the pointcloud
                activateSuperquadricsComputation(true);
                ros::Duration(3.5).sleep(); // sleep for 2 seconds

                // Stop computation of the superquadrics from the pointcloud. Our objects don't move, so there is no need to
                // continuously recompute the superquadrics
                ROS_INFO("[DemoSharon] Stop the computation of the superquadrics.");
                activateSuperquadricsComputation(false);
                ros::Duration(0.5).sleep(); // sleep for 2 seconds

                ROS_INFO("[DemoSharon] Get the computed superquadrics.");
                // Get the previously computed superquadrics
                if (!getSuperquadrics())
                { // If it's empty, there is no objects to grasp
                    return;
                }
                ROS_INFO("[DemoSharon] We have %d supequadrics.", (int)superquadricsMsg_.superquadrics.size());
                ros::Duration(0.5).sleep(); // sleep for 2 seconds

                if (!getBoundingBoxesFromSupercuadrics())
                {
                    return;
                }
                for (int i = 0; i < bboxesMsg_.bounding_boxes.size(); i++)
                    ROS_INFO("id: %d tlx: %d tly: %d brx: %d bry:%d", bboxesMsg_.bounding_boxes[i].id,
                             bboxesMsg_.bounding_boxes[i].tlx, bboxesMsg_.bounding_boxes[i].tly,
                             bboxesMsg_.bounding_boxes[i].brx, bboxesMsg_.bounding_boxes[i].bry);

                sqCategories_.clear();
                darknet_ros_msgs::BoundingBoxesConstPtr darknetBBoxesMsg = ros::topic::waitForMessage<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes");
                yoloBBoxesMsg_ = *darknetBBoxesMsg;

                for (int i = 0; i < yoloBBoxesMsg_.bounding_boxes.size(); i++)
                {
                    darknet_ros_msgs::BoundingBox bboxMsg = yoloBBoxesMsg_.bounding_boxes[i];
                    std::array<int, 4> bboxYolo = {(int)bboxMsg.xmin, (int)bboxMsg.ymin, (int)bboxMsg.xmax, (int)bboxMsg.ymax};
                    float IoU = 0.0;
                    int currentSqId = -1;
                    for (int j = 0; j < bboxesMsg_.bounding_boxes.size(); j++)
                    {
                        std::array<int, 4> bboxSq = {bboxesMsg_.bounding_boxes[j].tlx, bboxesMsg_.bounding_boxes[j].tly,
                                                     bboxesMsg_.bounding_boxes[j].brx, bboxesMsg_.bounding_boxes[j].bry};

                        ROS_INFO("yolo: %d superq: %d", i, j);
                        float currentIoU = 0.0;
                        computeIntersectionOverUnion(bboxYolo, bboxSq, currentIoU);

                        if (currentIoU > IoU)
                        {
                            IoU = currentIoU;
                            currentSqId = bboxesMsg_.bounding_boxes[j].id;
                        }
                    }
                    if (IoU > 0)
                    {
                        SqCategory sqCategory;
                        sqCategory.idSq = currentSqId;
                        sqCategory.category = bboxMsg.Class;
                        sqCategories_.push_back(sqCategory);
                    }
                }

                for (int idx = 0; idx < sqCategories_.size(); idx++)
                {
                    ROS_INFO("id: %d category: %s", sqCategories_[idx].idSq, sqCategories_[idx].category.c_str());
                }

                addSupequadricsPlanningScene();
                state_ = WAIT_FOR_COMMAND;
            }
            break;
            case WAIT_FOR_COMMAND:
            {
                waitingForGlassesCommand_ = true;
                waitingForAsrCommand_ = true;
                foundGlasses_ = false;
                indexGlassesSqCategory_ = -1;
                ROS_INFO("[DemoSharon] Wait for user's command.");
                if (!glassesCommandReceived_ && asrCommandReceived_)
                {
                    ROS_WARN("[DemoSharon] The gaze should be faster than the asr.");
                    return;
                }
                if (glassesCommandReceived_ && !asrCommandReceived_)
                {
                    ROS_INFO("[DemoSharon] Gaze command received.");
                    // Además que el objeto sea uno de los detectados

                    for (int i = 0; i < sqCategories_.size(); i++)
                    {
                        if (sqCategories_[i].category.find(glassesCategory_, 0) != std::string::npos)
                        {
                            foundGlasses_ = true;
                            indexGlassesSqCategory_ = i;
                            indexSqCategory_ = indexGlassesSqCategory_;
                            break;
                        }
                    }
                    if (indexGlassesSqCategory_ >= 0)
                    {
                        ROS_INFO("[DemoSharon] Gaze command category %s is  available in the table.", sqCategories_[indexGlassesSqCategory_].category.c_str());
                        waitingForGlassesCommand_ = false;
                        waitingForAsrCommand_ = true;
                        state_ = COMPUTE_GRASP_POSES;
                    }
                    else
                    {
                        ROS_WARN("[DemoSharon] Gaze command category %s is NOT available in the table.", sqCategories_[indexGlassesSqCategory_].category.c_str());
                        waitingForGlassesCommand_ = true;
                        waitingForAsrCommand_ = true;
                    }
                }
            }
            break;
            case COMPUTE_GRASP_POSES:
            {
                // HERE WE SHOULD CREATE A THREAD THAT CAN BE KILLED IF NEEDED
                if (firstInState)
                {
                    threadComputeGraspPoses_ = new std::thread([this]
                                                               { this->computeGraspPosesThread(); });
                    firstInState = false;
                }
            }
            break;
            case -1:
            {
                ROS_INFO("YES!! KILLED COMPUTE GRASP POSES");
            }
            break;
            }
        }

        // removeCollisionObjectsPlanningScene();

        // geometry_msgs::Pose tablePose;
        // tablePose.orientation.w = 1.0;
        // tablePose.position.x = tablePosition_[0];
        // tablePose.position.y = tablePosition_[1];
        // tablePose.position.z = tablePosition_[2];

        // addTablePlanningScene(tableDimensions_, tablePose, "table1");

        // if (!initializeRightArmPosition(initRightArmPositions_))
        // {
        //     return;
        // }

        // if (!initializeLeftArmPosition(initLeftArmPositions_))
        // {
        //     return;
        // }

        // initializeTorsoPosition(initTorsoPosition_);

        // initializeHeadPosition(initHeadPositions_);

        // ROS_INFO("[DemoSharon] Start the computation of the superquadrics.");
        // // Start computation of the superquadrics from the pointcloud
        // activateSuperquadricsComputation(true);
        // ros::Duration(0.5).sleep(); // sleep for 2 seconds

        // // Stop computation of the superquadrics from the pointcloud. Our objects don't move, so there is no need to
        // // continuously recompute the superquadrics
        // ROS_INFO("[DemoSharon] Stop the computation of the superquadrics.");
        // activateSuperquadricsComputation(false);
        // ros::Duration(0.5).sleep(); // sleep for 2 seconds

        // ROS_INFO("[DemoSharon] Get the computed superquadrics.");
        // // Get the previously computed superquadrics
        // if (!getSuperquadrics())
        // { // If it's empty, there is no objects to grasp
        //     return;
        // }
        // ROS_INFO("[DemoSharon] We have %d supequadrics.", (int)superquadricsMsg_.superquadrics.size());
        // ros::Duration(0.5).sleep(); // sleep for 2 seconds

        // if (!getBoundingBoxesFromSupercuadrics())
        // {
        //     return;
        // }
        // for (int i = 0; i < bboxesMsg_.bounding_boxes.size(); i++)
        //     ROS_INFO("id: %d tlx: %d tly: %d brx: %d bry:%d", bboxesMsg_.bounding_boxes[i].id,
        //              bboxesMsg_.bounding_boxes[i].tlx, bboxesMsg_.bounding_boxes[i].tly,
        //              bboxesMsg_.bounding_boxes[i].brx, bboxesMsg_.bounding_boxes[i].bry);

        // sqCategories_.clear();
        // darknet_ros_msgs::BoundingBoxesConstPtr darknetBBoxesMsg = ros::topic::waitForMessage<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes");
        // yoloBBoxesMsg_ = *darknetBBoxesMsg;

        // for (int i = 0; i < yoloBBoxesMsg_.bounding_boxes.size(); i++)
        // {
        //     darknet_ros_msgs::BoundingBox bboxMsg = yoloBBoxesMsg_.bounding_boxes[i];
        //     std::array<int, 4> bboxYolo = {(int)bboxMsg.xmin, (int)bboxMsg.ymin, (int)bboxMsg.xmax, (int)bboxMsg.ymax};
        //     float IoU = 0.0;
        //     int currentSqId = -1;
        //     for (int j = 0; j < bboxesMsg_.bounding_boxes.size(); j++)
        //     {
        //         std::array<int, 4> bboxSq = {bboxesMsg_.bounding_boxes[j].tlx, bboxesMsg_.bounding_boxes[j].tly,
        //                                      bboxesMsg_.bounding_boxes[j].brx, bboxesMsg_.bounding_boxes[j].bry};

        //         ROS_INFO("yolo: %d superq: %d", i, j);
        //         float currentIoU = 0.0;
        //         computeIntersectionOverUnion(bboxYolo, bboxSq, currentIoU);

        //         if (currentIoU > IoU)
        //         {
        //             IoU = currentIoU;
        //             currentSqId = bboxesMsg_.bounding_boxes[j].id;
        //         }
        //     }
        //     if (IoU > 0)
        //     {
        //         SqCategory sqCategory;
        //         sqCategory.idSq = currentSqId;
        //         sqCategory.category = bboxMsg.Class;
        //         sqCategories_.push_back(sqCategory);
        //     }
        // }

        // for (int idx = 0; idx < sqCategories_.size(); idx++)
        // {
        //     ROS_INFO("id: %d category: %s", sqCategories_[idx].idSq, sqCategories_[idx].category.c_str());
        // }

        // addSupequadricsPlanningScene();

        // waitingForGlassesCommand_ = true;
        // int indexSq = -1;
        // ROS_INFO("[DemoSharon] Wait for message in /comms_glasses_server/data");

        // while (!glassesCommandReceived_ && ros::ok())
        // {
        // }

        // if (glassesCommandReceived_)
        // {
        //     // Además que el objeto sea uno de los detectados
        //     waitingForGlassesCommand_ = false;
        //     waitingForAsrCommand_ = true;
        //     foundGlasses_ = false;
        //     for (int i = 0; i < sqCategories_.size(); i++)
        //     {
        //         if (sqCategories_[i].category.find(glassesCategory_, 0) != std::string::npos)
        //         {
        //             foundGlasses_ = true;
        //             indexGlassesSqCategory_ = i;
        //             break;
        //         }
        //     }

        //     mtxASR_.lock();
        //     if (foundAsr_)
        //     {
        //         if (sqCategories_[indexGlassesSqCategory_].category.find(asr_, 0) != std::string::npos)
        //         {
        //             ROS_INFO("ASR command received is the same: %s", asr_.c_str());
        //             indexSq = indexGlassesSqCategory_;
        //         }
        //         else
        //         {
        //             indexSq = indexSqCategory_;
        //         }
        //         waitingForGlassesCommand_ = false;
        //     }
        //     mtxASR_.unlock();
        // }
    }

    void DemoSharon::demoOnlyGlasses()
    {

        waitingForGlassesCommand_ = false;
        glassesCommandReceived_ = false;

        removeCollisionObjectsPlanningScene();

        geometry_msgs::Pose tablePose;
        tablePose.orientation.w = 1.0;
        tablePose.position.x = tablePosition_[0];
        tablePose.position.y = tablePosition_[1];
        tablePose.position.z = tablePosition_[2];

        addTablePlanningScene(tableDimensions_, tablePose, "table1");

        if (!initializeRightArmPosition(initRightArmPositions_))
        {
            return;
        }

        if (!initializeLeftArmPosition(initLeftArmPositions_))
        {
            return;
        }

        initializeTorsoPosition(initTorsoPosition_);

        initializeHeadPosition(initHeadPositions_);

        ROS_INFO("[DemoSharon] Start the computation of the superquadrics.");
        // Start computation of the superquadrics from the pointcloud
        activateSuperquadricsComputation(true);
        ros::Duration(0.5).sleep(); // sleep for 2 seconds

        // Stop computation of the superquadrics from the pointcloud. Our objects don't move, so there is no need to
        // continuously recompute the superquadrics
        ROS_INFO("[DemoSharon] Stop the computation of the superquadrics.");
        activateSuperquadricsComputation(false);
        ros::Duration(0.5).sleep(); // sleep for 2 seconds

        ROS_INFO("[DemoSharon] Get the computed superquadrics.");
        // Get the previously computed superquadrics
        if (!getSuperquadrics())
        { // If it's empty, there is no objects to grasp
            return;
        }
        ROS_INFO("[DemoSharon] We have %d supequadrics.", (int)superquadricsMsg_.superquadrics.size());
        ros::Duration(0.5).sleep(); // sleep for 2 seconds

        if (!getBoundingBoxesFromSupercuadrics())
        {
            return;
        }
        for (int i = 0; i < bboxesMsg_.bounding_boxes.size(); i++)
            ROS_INFO("id: %d tlx: %d tly: %d brx: %d bry:%d", bboxesMsg_.bounding_boxes[i].id,
                     bboxesMsg_.bounding_boxes[i].tlx, bboxesMsg_.bounding_boxes[i].tly,
                     bboxesMsg_.bounding_boxes[i].brx, bboxesMsg_.bounding_boxes[i].bry);

        sqCategories_.clear();
        darknet_ros_msgs::BoundingBoxesConstPtr darknetBBoxesMsg = ros::topic::waitForMessage<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes");
        yoloBBoxesMsg_ = *darknetBBoxesMsg;

        for (int i = 0; i < yoloBBoxesMsg_.bounding_boxes.size(); i++)
        {
            darknet_ros_msgs::BoundingBox bboxMsg = yoloBBoxesMsg_.bounding_boxes[i];
            std::array<int, 4> bboxYolo = {(int)bboxMsg.xmin, (int)bboxMsg.ymin, (int)bboxMsg.xmax, (int)bboxMsg.ymax};
            float IoU = 0.0;
            int currentSqId = -1;
            for (int j = 0; j < bboxesMsg_.bounding_boxes.size(); j++)
            {
                std::array<int, 4> bboxSq = {bboxesMsg_.bounding_boxes[j].tlx, bboxesMsg_.bounding_boxes[j].tly,
                                             bboxesMsg_.bounding_boxes[j].brx, bboxesMsg_.bounding_boxes[j].bry};

                ROS_INFO("yolo: %d superq: %d", i, j);
                float currentIoU = 0.0;
                computeIntersectionOverUnion(bboxYolo, bboxSq, currentIoU);

                if (currentIoU > IoU)
                {
                    IoU = currentIoU;
                    currentSqId = bboxesMsg_.bounding_boxes[j].id;
                }
            }
            if (IoU > 0)
            {
                SqCategory sqCategory;
                sqCategory.idSq = currentSqId;
                sqCategory.category = bboxMsg.Class;
                sqCategories_.push_back(sqCategory);
            }
        }

        for (int idx = 0; idx < sqCategories_.size(); idx++)
        {
            ROS_INFO("id: %d category: %s", sqCategories_[idx].idSq, sqCategories_[idx].category.c_str());
        }

        addSupequadricsPlanningScene();

        waitingForGlassesCommand_ = true;
        ROS_INFO("[DemoSharon] Wait for message in /comms_glasses_server/data");
        while (!glassesCommandReceived_ && ros::ok())
        {
        }

        if (glassesCommandReceived_)
        {
            // Además que el objeto sea uno de los detectados
            waitingForGlassesCommand_ = false;

            foundGlasses_ = false;
            for (int i = 0; i < sqCategories_.size(); i++)
            {
                if (sqCategories_[i].category.find(glassesCategory_, 0) != std::string::npos)
                {
                    foundGlasses_ = true;
                    indexGlassesSqCategory_ = i;
                    break;
                }
            }
        }

        if (indexGlassesSqCategory_ >= 0)
        {

            ROS_INFO("Planning to move %s to a target pose expressed in %s", groupRightArmTorsoPtr_->getEndEffectorLink().c_str(), groupRightArmTorsoPtr_->getPlanningFrame().c_str());

            sharon_msgs::ComputeGraspPoses srvGraspingPoses;
            srvGraspingPoses.request.id = sqCategories_[indexGlassesSqCategory_].idSq;
            geometry_msgs::PoseArray graspingPoses, orderedGraspingPoses;
            groupRightArmTorsoPtr_->setStartStateToCurrentState();

            if (clientComputeGraspPoses_.call(srvGraspingPoses))
            {
                ROS_INFO("[DemoSharon] ComputeGraspPoses: %d", (bool)srvGraspingPoses.response.success);
                graspingPoses = srvGraspingPoses.response.poses;
            }
            ROS_INFO("[DemoSharon] NumberPoses: %d", (int)graspingPoses.poses.size());

            int indexFeasible = -1;
            bool successGoToReaching = goToAFeasibleReachingPose(graspingPoses, indexFeasible);

            while (!groupRightArmTorsoPtr_->getMoveGroupClient().getState().isDone() && ros::ok())
            {
                ROS_INFO("WAITING.....");
            }

            if (successGoToReaching)
            {

                // Open gripper
                moveGripper(openGripperPositions_, "right");
                std::vector<std::string> objectIds;
                objectIds.push_back("object_" + std::to_string(sqCategories_[indexGlassesSqCategory_].idSq));
                planningSceneInterface_.removeCollisionObjects(objectIds);
                ros::Duration(1.0).sleep(); // sleep for 1 seconds

                goToGraspingPose(graspingPoses.poses[indexFeasible]);
                while (!groupRightArmTorsoPtr_->getMoveGroupClient().getState().isDone() && ros::ok())
                {
                    ROS_INFO("WAITING.....");
                }

                // ros::Duration(1.0).sleep(); // sleep for 1 seconds
                moveGripper(closeGripperPositions_, "right");
                ros::Duration(0.1).sleep(); // sleep for 1 seconds
                goUp(groupRightArmTorsoPtr_, 0.2);

                releaseGripper_ = false;
                while (ros::ok() && !releaseGripper_)
                {
                    ROS_INFO("Waiting for command to release the gripper...");
                    ros::Duration(0.1).sleep(); // sleep for 1 seconds
                }
                moveGripper(openGripperPositions_, "right");
                ros::Duration(1.0).sleep(); // sleep for 1 seconds
            }

            ROS_INFO("[DemoSharon] Done!");
        }
    }

    void DemoSharon::demoOnlyASR()
    {

        waitingForAsrCommand_ = false;
        asrCommandReceived_ = false;
        removeCollisionObjectsPlanningScene();

        geometry_msgs::Pose tablePose;
        tablePose.orientation.w = 1.0;
        tablePose.position.x = tablePosition_[0];
        tablePose.position.y = tablePosition_[1];
        tablePose.position.z = tablePosition_[2];

        addTablePlanningScene(tableDimensions_, tablePose, "table1");

        tablePose.position.x = tablePosition2_[0];
        tablePose.position.y = tablePosition2_[1];
        tablePose.position.z = tablePosition2_[2];
        addTablePlanningScene(tableDimensions2_, tablePose, "table2");

        if (!initializeRightArmPosition(initRightArmPositions_))
        {
            return;
        }

        if (!initializeLeftArmPosition(initLeftArmPositions_))
        {
            return;
        }

        initializeTorsoPosition(initTorsoPosition_);

        initializeHeadPosition(initHeadPositions_);

        ros::Duration(1.5).sleep(); // sleep for 2 seconds

        ROS_INFO("[DemoSharon] Start the computation of the superquadrics.");
        // Start computation of the superquadrics from the pointcloud
        activateSuperquadricsComputation(true);
        ros::Duration(3.5).sleep(); // sleep for 2 seconds

        // Stop computation of the superquadrics from the pointcloud. Our objects don't move, so there is no need to
        // continuously recompute the superquadrics
        ROS_INFO("[DemoSharon] Stop the computation of the superquadrics.");
        activateSuperquadricsComputation(false);
        ros::Duration(0.5).sleep(); // sleep for 2 seconds

        ROS_INFO("[DemoSharon] Get the computed superquadrics.");
        // Get the previously computed superquadrics
        if (!getSuperquadrics())
        { // If it's empty, there is no objects to grasp
            return;
        }
        ROS_INFO("[DemoSharon] We have %d supequadrics.", (int)superquadricsMsg_.superquadrics.size());
        ros::Duration(0.5).sleep(); // sleep for 2 seconds

        if (!getBoundingBoxesFromSupercuadrics())
        {
            return;
        }
        for (int i = 0; i < bboxesMsg_.bounding_boxes.size(); i++)
            ROS_INFO("id: %d tlx: %d tly: %d brx: %d bry:%d", bboxesMsg_.bounding_boxes[i].id,
                     bboxesMsg_.bounding_boxes[i].tlx, bboxesMsg_.bounding_boxes[i].tly,
                     bboxesMsg_.bounding_boxes[i].brx, bboxesMsg_.bounding_boxes[i].bry);

        sqCategories_.clear();
        darknet_ros_msgs::BoundingBoxesConstPtr darknetBBoxesMsg = ros::topic::waitForMessage<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes");
        yoloBBoxesMsg_ = *darknetBBoxesMsg;

        for (int i = 0; i < yoloBBoxesMsg_.bounding_boxes.size(); i++)
        {
            darknet_ros_msgs::BoundingBox bboxMsg = yoloBBoxesMsg_.bounding_boxes[i];
            std::array<int, 4> bboxYolo = {(int)bboxMsg.xmin, (int)bboxMsg.ymin, (int)bboxMsg.xmax, (int)bboxMsg.ymax};
            float IoU = 0.0;
            int currentSqId = -1;
            for (int j = 0; j < bboxesMsg_.bounding_boxes.size(); j++)
            {
                std::array<int, 4> bboxSq = {bboxesMsg_.bounding_boxes[j].tlx, bboxesMsg_.bounding_boxes[j].tly,
                                             bboxesMsg_.bounding_boxes[j].brx, bboxesMsg_.bounding_boxes[j].bry};

                ROS_INFO("yolo: %d superq: %d", i, j);
                float currentIoU = 0.0;
                computeIntersectionOverUnion(bboxYolo, bboxSq, currentIoU);

                if (currentIoU > IoU)
                {
                    IoU = currentIoU;
                    currentSqId = bboxesMsg_.bounding_boxes[j].id;
                }
            }
            if (IoU > 0)
            {
                SqCategory sqCategory;
                sqCategory.idSq = currentSqId;
                sqCategory.category = bboxMsg.Class;
                sqCategories_.push_back(sqCategory);
            }
        }

        for (int idx = 0; idx < sqCategories_.size(); idx++)
        {
            ROS_INFO("id: %d category: %s", sqCategories_[idx].idSq, sqCategories_[idx].category.c_str());
        }

        addSupequadricsPlanningScene();

        waitingForAsrCommand_ = true;
        ROS_INFO("[DemoSharon] Wait for message in asr_node/data");
        bool auxASR = false;
        while (ros::ok() && !auxASR)
        {
            mtxASR_.lock();
            auxASR = foundAsr_;
            mtxASR_.unlock();
            if (auxASR)
            {
                break;
            }
        }
        ROS_INFO("CHECK!");

        if (indexSqCategory_ >= 0)
        {
            waitingForAsrCommand_ = false;
            ROS_INFO("Planning to move %s to a target pose expressed in %s", groupRightArmTorsoPtr_->getEndEffectorLink().c_str(), groupRightArmTorsoPtr_->getPlanningFrame().c_str());

            sharon_msgs::ComputeGraspPoses srvGraspingPoses;
            srvGraspingPoses.request.id = sqCategories_[indexSqCategoryAsr_].idSq;
            geometry_msgs::PoseArray graspingPoses, orderedGraspingPoses;
            groupRightArmTorsoPtr_->setStartStateToCurrentState();

            if (clientComputeGraspPoses_.call(srvGraspingPoses))
            {
                ROS_INFO("[DemoSharon] ComputeGraspPoses: %d", (bool)srvGraspingPoses.response.success);
                graspingPoses = srvGraspingPoses.response.poses;
            }
            ROS_INFO("[DemoSharon] NumberPoses: %d", (int)graspingPoses.poses.size());

            int indexFeasible = -1;
            bool successGoToReaching = goToAFeasibleReachingPose(graspingPoses, indexFeasible);

            actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction> &moveGroupClient = groupRightArmTorsoPtr_->getMoveGroupClient();
            while (!moveGroupClient.getState().isDone() && ros::ok())
            {
                ROS_INFO("WAITING.....");
                ros::Duration(0.1).sleep();
            }

            if (successGoToReaching)
            {

                // Open gripper
                moveGripper(openGripperPositions_, "right");
                std::vector<std::string> objectIds;
                objectIds.push_back("object_" + std::to_string(sqCategories_[indexSqCategoryAsr_].idSq));
                planningSceneInterface_.removeCollisionObjects(objectIds);
                ros::Duration(1.0).sleep(); // sleep for 1 seconds
                goToGraspingPose(graspingPoses.poses[indexFeasible]);
                while (!groupRightArmTorsoPtr_->getMoveGroupClient().getState().isDone() && ros::ok())
                {
                    ROS_INFO("WAITING.....");
                    ros::Duration(0.1).sleep();
                }

                ros::Duration(0.2).sleep(); // sleep for 1 seconds
                moveGripper(closeGripperPositions_, "right");
                ros::Duration(0.2).sleep(); // sleep for 1 seconds
                goUp(groupRightArmTorsoPtr_, 0.1);

                releaseGripper_ = false;
                while (ros::ok() && !releaseGripper_)
                {
                    ROS_INFO("Waiting for command to release the gripper...");
                    ros::Duration(0.1).sleep(); // sleep for 1 seconds
                }
                moveGripper(openGripperPositions_, "right");
                ros::Duration(1.0).sleep(); // sleep for 1 seconds
            }
            if (!initializeRightArmPosition(initRightArmPositions_))
            {
                return;
            }

            ROS_INFO("[DemoSharon] Done!");
        }
    }

    void DemoSharon::moveGripper(const float positions[2], std::string name)
    {
        follow_joint_control_client_Ptr auxGripperClient;

        if (name == "right")
        {
            auxGripperClient = rightGripperClient_;
        }
        else if (name == "left")
        {
            auxGripperClient = leftGripperClient_;
        }
        control_msgs::FollowJointTrajectoryGoal gripperGoal;
        ROS_INFO("Setting gripper position: (%f ,%f)", positions[0], positions[1]);
        waypointGripperGoal(name, gripperGoal, positions, 0.5);

        // Sends the command to start the given trajectory now
        gripperGoal.trajectory.header.stamp = ros::Time(0);
        auxGripperClient->sendGoal(gripperGoal);

        // Wait for trajectory execution
        while (!(auxGripperClient->getState().isDone()) && ros::ok())
        {
            ros::Duration(0.1).sleep(); // sleep for 1 seconds
        }
        ROS_INFO("Gripper set to position: (%f, %f)", positions[0], positions[1]);
    }

    bool DemoSharon::goUp(moveit::planning_interface::MoveGroupInterface *groupArmTorsoPtr, float upDistance)
    {
        groupRightArmTorsoPtr_->setMaxVelocityScalingFactor(0.2);
        geometry_msgs::PoseStamped currentPose = groupArmTorsoPtr->getCurrentPose();
        KDL::Frame frameEndWrtBase;
        tf::poseMsgToKDL(currentPose.pose, frameEndWrtBase);
        frameEndWrtBase.p[2] += upDistance;

        geometry_msgs::Pose upPose;
        tf::poseKDLToMsg(frameEndWrtBase, upPose);
        groupArmTorsoPtr->setPoseTarget(upPose);

        moveit::planning_interface::MoveItErrorCode code = groupArmTorsoPtr->plan(plan_);
        bool successPlanning = (code == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (successPlanning)
        {
            moveit::planning_interface::MoveItErrorCode e = groupArmTorsoPtr->execute(plan_);
            if (e == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            {
                ROS_INFO("[DemoSharon] Success in moving the grasped object up.");
                return true;
            }
            else
            {
                ROS_INFO("[DemoSharon] Error in moving the grasped object up.");
                return false;
            }
        }
        else
        {
            ROS_INFO("[DemoSharon] No feasible up pose!");
            return false;
        }
    }

    bool DemoSharon::goToGraspingPose(const geometry_msgs::Pose &graspingPose)
    {
        groupRightArmTorsoPtr_->setMaxVelocityScalingFactor(0.1);
        KDL::Frame frameEndWrtBase;
        tf::poseMsgToKDL(graspingPose, frameEndWrtBase);
        KDL::Frame frameToolWrtEnd;
        frameToolWrtEnd.p[0] = -DISTANCE_TOOL_LINK_GRIPPER_LINK;
        KDL::Frame frameToolWrtBase = frameEndWrtBase * frameToolWrtEnd;

        geometry_msgs::Pose toolPose;

        tf::poseKDLToMsg(frameToolWrtBase, toolPose);
        groupRightArmTorsoPtr_->setPoseTarget(toolPose);

        moveit::planning_interface::MoveItErrorCode code = groupRightArmTorsoPtr_->plan(plan_);
        bool successPlanning = (code == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (successPlanning)
        {
            moveit::planning_interface::MoveItErrorCode e = groupRightArmTorsoPtr_->asyncMove();
            if (e == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            {
                ROS_INFO("[DemoSharon] Success in moving the robot to the grasping pose.");
                return true;
            }
            else
            {
                ROS_INFO("[DemoSharon] Error in moving the robot to the grasping pose.");
                return false;
            }
        }
        else
        {
            ROS_INFO("[DemoSharon] No feasible grasping pose!");
            return false;
        }
    }

    bool DemoSharon::goToAFeasibleReachingPose(const geometry_msgs::PoseArray &graspingPoses, int &indexFeasible)
    {

        bool successPlanning = false;
        robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematicModel_));
        kinematic_state->setToDefaultValues();
        for (int idx = 0; idx < graspingPoses.poses.size(); idx++)
        {
            ROS_INFO("[DemoSharon] idx: %d", idx);
            ROS_INFO("Grasping Pose[%d]: %f %f %f", idx, graspingPoses.poses[idx].position.x, graspingPoses.poses[idx].position.y, graspingPoses.poses[idx].position.z);

            KDL::Frame frameEndWrtBase;
            tf::poseMsgToKDL(graspingPoses.poses[idx], frameEndWrtBase);
            KDL::Frame frameReachingWrtEnd;
            frameReachingWrtEnd.p[0] = -reachingDistance_ - DISTANCE_TOOL_LINK_GRIPPER_LINK;
            KDL::Frame frameReachingWrtBase = frameEndWrtBase * frameReachingWrtEnd;

            tf::poseKDLToMsg(frameReachingWrtBase, reachingPose_);

            bool found_ik = kinematic_state->setFromIK(joint_model_group, reachingPose_, 0.1);

            //     geometry_msgs::PoseStamped goal_pose;
            // goal_pose.header.frame_id = "base_footprint";
            // goal_pose.pose = graspingPoses.poses[idx];
            if (found_ik)
            {
                groupRightArmTorsoPtr_->setPoseTarget(reachingPose_);
                ROS_INFO("SET POSE TARGET");

                moveit::planning_interface::MoveItErrorCode code = groupRightArmTorsoPtr_->plan(plan_);
                successPlanning = (code == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            }
            else
            {
                successPlanning = false;
            }

            // if(groupRightArmTorsoPtr_->plan(plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            //     successPlanning = true;
            // else{
            //     successPlanning = false;
            // }
            // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", successPlanning ? "" : "FAILED");

            if (successPlanning)
            {
                indexFeasible = idx;
                break;
            }
            // ROS_INFO("AQUI");
        }
        if (successPlanning)
        {
            moveit::planning_interface::MoveItErrorCode e = groupRightArmTorsoPtr_->asyncMove();
            if (e == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            {
                ROS_INFO("[DemoSharon] Success in moving the robot to the reaching pose.");
                return true;
            }
            else
            {
                ROS_INFO("[DemoSharon] Error in moving the robot to the reaching pose.");
                return false;
            }
        }
        else
        {
            ROS_INFO("[DemoSharon] No feasible reaching pose found!");
            return false;
        }
    }

    void DemoSharon::removeCollisionObjectsPlanningScene()
    {
        ROS_INFO("[DemoSharon] Removing objects in the planningScene");
        std::vector<std::string> objectIds = planningSceneInterface_.getKnownObjectNames();
        planningSceneInterface_.removeCollisionObjects(objectIds);
        ros::Duration(1.0).sleep(); // sleep for 2 seconds
    }

    void DemoSharon::addTablePlanningScene(const std::vector<float> &dimensions, const geometry_msgs::Pose &tablePose, const std::string &id)
    {
        ROS_INFO("[DemoSharon] Add table collision objects to the planning scene");
        // Collision object
        moveit_msgs::CollisionObject collisionObject;
        collisionObject.id = id;
        collisionObject.header.frame_id = groupRightArmTorsoPtr_->getPlanningFrame();
        ROS_INFO("[DemoSharon] Planning_frame: %s", groupRightArmTorsoPtr_->getPlanningFrame().c_str());
        shape_msgs::SolidPrimitive table;
        table.type = table.BOX;
        table.dimensions.resize(3);
        table.dimensions[0] = dimensions[0];
        table.dimensions[1] = dimensions[1];
        table.dimensions[2] = dimensions[2];

        collisionObject.primitives.push_back(table);
        collisionObject.primitive_poses.push_back(tablePose);
        collisionObject.operation = collisionObject.ADD;

        planningSceneInterface_.applyCollisionObject(collisionObject);
    }

    void DemoSharon::addSupequadricsPlanningScene()
    {
        std::vector<moveit_msgs::CollisionObject> collisionObjects;

        for (int i = 0; i < superquadricsMsg_.superquadrics.size(); i++)
        {
            sharon_msgs::Superquadric superquadric = superquadricsMsg_.superquadrics[i];
            geometry_msgs::Pose superquadricPose;
            superquadricPose.position.x = superquadric.x;
            superquadricPose.position.y = superquadric.y;
            superquadricPose.position.z = superquadric.z;

            Eigen::AngleAxisf rollAngle(superquadric.roll, Eigen::Vector3f::UnitZ());
            Eigen::AngleAxisf yawAngle(superquadric.pitch, Eigen::Vector3f::UnitY());
            Eigen::AngleAxisf pitchAngle(superquadric.yaw, Eigen::Vector3f::UnitZ());

            Eigen::Quaternionf q = rollAngle * yawAngle * pitchAngle;
            superquadricPose.orientation.x = q.x();
            superquadricPose.orientation.y = q.y();
            superquadricPose.orientation.z = q.z();
            superquadricPose.orientation.w = q.w();
            moveit_msgs::CollisionObject collisionObject;
            collisionObject.header.frame_id = groupRightArmTorsoPtr_->getPlanningFrame();

            collisionObject.id = "object_" + std::to_string(superquadric.id);

            shape_msgs::SolidPrimitive primitive;

            if (superquadric.e1 < elimit1_)
            {
                if ((superquadric.e2 < elimit1_) || (superquadric.e2 >= elimit2_))
                {
                    // BOX
                    primitive.type = primitive.BOX;
                    primitive.dimensions.resize(3);
                    primitive.dimensions[0] = 2 * superquadric.a1 + inflateSize_;
                    primitive.dimensions[1] = 2 * superquadric.a2 + inflateSize_;
                    primitive.dimensions[2] = 2 * superquadric.a3 + inflateSize_;
                    collisionObject.primitives.push_back(primitive);
                    collisionObject.primitive_poses.push_back(superquadricPose);
                    collisionObject.operation = collisionObject.ADD;
                    collisionObjects.push_back(collisionObject);
                }
                if ((superquadric.e2 >= elimit1_) && (superquadric.e2 < elimit2_))
                {
                    // CYLINDER

                    int index_height = 0;
                    float height = superquadric.a1;
                    if (superquadric.a2 > height)
                    {
                        height = superquadric.a2;
                    }
                    if (superquadric.a3 > height)
                    {
                        height = superquadric.a3;
                    }

                    float radius = 0;
                    if (superquadric.a1 > radius && superquadric.a1 < height)
                    {
                        radius = superquadric.a1;
                    }
                    if (superquadric.a2 > radius && superquadric.a2 < height)
                    {
                        radius = superquadric.a2;
                    }
                    if (superquadric.a3 > radius && superquadric.a3 < height)
                    {
                        radius = superquadric.a3;
                    }
                    radius += inflateSize_;
                    height += inflateSize_;

                    primitive.type = primitive.CYLINDER;
                    primitive.dimensions.resize(2);
                    primitive.dimensions[0] = radius;
                    primitive.dimensions[1] = height;

                    collisionObject.primitives.push_back(primitive);
                    collisionObject.primitive_poses.push_back(superquadricPose);
                    collisionObject.operation = collisionObject.ADD;
                    collisionObjects.push_back(collisionObject);
                }
            }
            else if (superquadric.e1 >= elimit1_)
            {
                // BOX
                primitive.type = primitive.BOX;
                primitive.dimensions.resize(3);
                primitive.dimensions[0] = 2 * superquadric.a1 + inflateSize_;
                primitive.dimensions[1] = 2 * superquadric.a2 + inflateSize_;
                primitive.dimensions[2] = 2 * superquadric.a3 + inflateSize_;
                collisionObject.primitives.push_back(primitive);
                collisionObject.primitive_poses.push_back(superquadricPose);
                collisionObject.operation = collisionObject.ADD;
                collisionObjects.push_back(collisionObject);
            }
        }
        planningSceneInterface_.applyCollisionObjects(collisionObjects);
    }

    void DemoSharon::initializeHeadPosition(const std::vector<float> &initHeadPositions)
    {

        ROS_INFO("Setting head to init position: (%f ,%f)", initHeadPositions[0], initHeadPositions[1]);

        sensor_msgs::JointStateConstPtr jointStatesMsgPtr = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");

        float head1Position = jointStatesMsgPtr->position[find(jointStatesMsgPtr->name.begin(), jointStatesMsgPtr->name.end(), std::string("head_1_joint")) - jointStatesMsgPtr->name.begin()];
        float head2Position = jointStatesMsgPtr->position[find(jointStatesMsgPtr->name.begin(), jointStatesMsgPtr->name.end(), std::string("head_2_joint")) - jointStatesMsgPtr->name.begin()];

        if ((abs(initHeadPositions[0] - head1Position) < maxErrorJoints_) && (abs(initHeadPositions[1] - head2Position) < maxErrorJoints_))
        {
            ROS_INFO("Head joints already in the init position: (%f, %f)", initHeadPositions[0], initHeadPositions[1]);
            return;
        }

        control_msgs::FollowJointTrajectoryGoal headGoal;

        waypointHeadGoal(headGoal, initHeadPositions, 1.0);

        ROS_INFO("Head goal");
        // Sends the command to start the given trajectory now
        headClient_->sendGoal(headGoal);

        // Wait for trajectory execution
        while (!(headClient_->getState().isDone()) && ros::ok())
        {

            ros::Duration(0.1).sleep(); // sleep for 1 seconds
        }
        ROS_INFO("Head set to position: (%f, %f)", initHeadPositions[0], initHeadPositions[1]);
    }

    void DemoSharon::initializeTorsoPosition(float initTorsoPosition)
    {
        control_msgs::FollowJointTrajectoryGoal torsoGoal;

        ROS_INFO("Setting torso to init position: (%f)", initTorsoPosition);

        sensor_msgs::JointStateConstPtr jointStatesMsgPtr = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");

        float torsoPosition = jointStatesMsgPtr->position[find(jointStatesMsgPtr->name.begin(), jointStatesMsgPtr->name.end(), std::string("torso_lift_joint")) - jointStatesMsgPtr->name.begin()];

        if ((abs(initTorsoPosition - torsoPosition) < maxErrorJoints_) && (abs(initTorsoPosition - torsoPosition) < maxErrorJoints_))
        {
            ROS_INFO("Torso joint already in the init position: (%f)", initTorsoPosition);
            return;
        }

        waypointTorsoGoal(torsoGoal, initTorsoPosition, 3.0);

        // Sends the command to start the given trajectory now
        // torsoGoal.trajectory.header.stamp = ros::Time::now();
        torsoClient_->sendGoal(torsoGoal);

        // Wait for trajectory execution
        while (!(torsoClient_->getState().isDone()) && ros::ok())
        {
            ros::Duration(0.1).sleep(); // sleep for 1 seconds
        }

        ROS_INFO("Torso set to position: (%f)", initTorsoPosition);
    }

    bool DemoSharon::initializeRightArmPosition(const std::vector<double> &initRightArmPositions)
    {
        ROS_INFO("Setting RightArm to init position: (%f %f %f %f %f %f %f)", initRightArmPositions[0], initRightArmPositions[1], initRightArmPositions[2],
                 initRightArmPositions[3], initRightArmPositions[4], initRightArmPositions[5], initRightArmPositions[6]);

        groupRightArmPtr_->setStartStateToCurrentState();
        groupRightArmPtr_->setJointValueTarget(initRightArmPositions);

        moveit::planning_interface::MoveItErrorCode code = groupRightArmPtr_->plan(plan_);
        bool successPlanning = (code == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (successPlanning)
        {
            moveit::planning_interface::MoveItErrorCode e = groupRightArmPtr_->move();
            if (e == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            {
                ROS_INFO("[DemoSharon] Right Arm success in moving to the initial joints position.");
                return true;
            }
            else
            {
                ROS_INFO("[DemoSharon] Right Arm Error in moving  to the initial joints position.");
                return false;
            }
        }
        else
        {
            return false;
        }
    }

    bool DemoSharon::initializeLeftArmPosition(const std::vector<double> &initLeftArmPositions)
    {
        ROS_INFO("Setting RightArm to init position: (%f %f %f %f %f %f %f)", initLeftArmPositions[0], initLeftArmPositions[1], initLeftArmPositions[2],
                 initLeftArmPositions[3], initLeftArmPositions[4], initLeftArmPositions[5], initLeftArmPositions[6]);

        groupLeftArmPtr_->setStartStateToCurrentState();
        groupLeftArmPtr_->setJointValueTarget(initLeftArmPositions);

        moveit::planning_interface::MoveItErrorCode code = groupLeftArmPtr_->plan(plan_);
        bool successPlanning = (code == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (successPlanning)
        {
            moveit::planning_interface::MoveItErrorCode e = groupLeftArmPtr_->move();
            if (e == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            {
                ROS_INFO("[DemoSharon] Left Arm success in moving to the initial joints position.");
                return true;
            }
            else
            {
                ROS_INFO("[DemoSharon] Left Arm Error in moving  to the initial joints position.");
                return false;
            }
        }
        else
        {
            return false;
        }
    }

    void DemoSharon::init()
    {
        ROS_INFO("[DemoSharon] init().");

        ROS_INFO("[DemoSharon] creating clients...");
        clientActivateSuperquadricsComputation_ = nodeHandle_.serviceClient<sharon_msgs::ActivateSupercuadricsComputation>("/grasp_objects/activate_superquadrics_computation");
        clientGetSuperquadrics_ = nodeHandle_.serviceClient<sharon_msgs::GetSuperquadrics>("/grasp_objects/get_superquadrics");
        clientComputeGraspPoses_ = nodeHandle_.serviceClient<sharon_msgs::ComputeGraspPoses>("/grasp_objects/compute_grasp_poses");
        clientGetBboxesSuperquadrics_ = nodeHandle_.serviceClient<sharon_msgs::GetBboxes>("/grasp_objects/get_bboxes_superquadrics");
        asrSubscriber_ = nodeHandle_.subscribe("/asr_node/data", 10, &DemoSharon::asrCallback, this);
        glassesDataSubscriber_ = nodeHandle_.subscribe("/comms_glasses_server/data", 10, &DemoSharon::glassesDataCallback, this);
        serviceReleaseGripper_ = nodeHandle_.advertiseService("/demo_sharon/release_gripper", &DemoSharon::releaseGripper, this);

        ros::param::get("demo_sharon/use_asr", useAsr_);
        ros::param::get("demo_sharon/use_glasses", useGlasses_);
        ros::param::get("demo_sharon/reaching_distance", reachingDistance_);
        ros::param::get("demo_sharon/elimit1", elimit1_);
        ros::param::get("demo_sharon/elimit2", elimit2_);
        ros::param::get("demo_sharon/inflate_size", inflateSize_);
        ros::param::get("demo_sharon/max_error_joints", maxErrorJoints_);
        ros::param::get("demo_sharon/head_joints_position_init", initHeadPositions_);
        ros::param::get("demo_sharon/torso_joint_position_init", initTorsoPosition_);
        ros::param::get("demo_sharon/table_dimensions", tableDimensions_);
        ros::param::get("demo_sharon/table_position", tablePosition_);
        ros::param::get("demo_sharon/table_position2", tablePosition2_);
        ros::param::get("demo_sharon/table_dimensions2", tableDimensions2_);

        ros::param::get("demo_sharon/right_arm_joints_position_init", initRightArmPositions_);
        ros::param::get("demo_sharon/left_arm_joints_position_init", initLeftArmPositions_);

        ROS_INFO("[DemoSharon] demo_sharon/reaching_distance set to %f", reachingDistance_);

        groupRightArmTorsoPtr_ = new moveit::planning_interface::MoveGroupInterface(nameTorsoRightArmGroup_);
        ROS_INFO("[DemoSharon] Move group interface %s", nameTorsoRightArmGroup_.c_str());

        groupRightArmPtr_ = new moveit::planning_interface::MoveGroupInterface(nameRightArmGroup_);
        ROS_INFO("[DemoSharon] Move group interface %s", nameRightArmGroup_.c_str());

        groupLeftArmTorsoPtr_ = new moveit::planning_interface::MoveGroupInterface(nameTorsoLeftArmGroup_);
        ROS_INFO("[DemoSharon] Move group interface %s", nameTorsoLeftArmGroup_.c_str());

        groupLeftArmPtr_ = new moveit::planning_interface::MoveGroupInterface(nameLeftArmGroup_);
        ROS_INFO("[DemoSharon] Move group interface %s", nameLeftArmGroup_.c_str());

        groupRightArmTorsoPtr_->setPlanningTime(1.5);
        groupRightArmTorsoPtr_->setPlannerId("SBLkConfigDefault");
        groupRightArmTorsoPtr_->setPoseReferenceFrame("base_footprint");
        groupRightArmTorsoPtr_->setMaxVelocityScalingFactor(0.2);

        groupRightArmPtr_->setPlanningTime(1.5);
        groupRightArmPtr_->setPlannerId("SBLkConfigDefault");
        groupRightArmPtr_->setPoseReferenceFrame("base_footprint");
        groupRightArmPtr_->setMaxVelocityScalingFactor(0.2);

        groupLeftArmTorsoPtr_->setPlanningTime(1.5);
        groupLeftArmTorsoPtr_->setPlannerId("SBLkConfigDefault");
        groupLeftArmTorsoPtr_->setPoseReferenceFrame("base_footprint");
        groupLeftArmTorsoPtr_->setMaxVelocityScalingFactor(0.2);

        groupLeftArmPtr_->setPlanningTime(1.5);
        groupLeftArmPtr_->setPlannerId("SBLkConfigDefault");
        groupLeftArmPtr_->setPoseReferenceFrame("base_footprint");
        groupLeftArmPtr_->setMaxVelocityScalingFactor(0.2);

        createClient(headClient_, std::string("head"));
        createClient(torsoClient_, std::string("torso"));
        createClient(rightGripperClient_, std::string("gripper_right"));

        robot_model_loader::RobotModelLoader robotModelLoader_("robot_description");
        kinematicModel_ = robotModelLoader_.getModel();
        joint_model_group = kinematicModel_->getJointModelGroup(nameTorsoRightArmGroup_);

        return;
    }

    bool DemoSharon::releaseGripper(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {
        releaseGripper_ = true;
        return true;
    }

    bool DemoSharon::getSuperquadrics()
    {
        sharon_msgs::GetSuperquadrics srvSq;

        if (clientGetSuperquadrics_.call(srvSq))
        {
            superquadricsMsg_ = srvSq.response.superquadrics;
            if (superquadricsMsg_.superquadrics.size() != 0)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
    }

    bool DemoSharon::getBoundingBoxesFromSupercuadrics()
    {

        sharon_msgs::GetBboxes srvBBox;
        ROS_INFO("[DemoSharon] Get bounding boxes from superquadrics...");
        if (clientGetBboxesSuperquadrics_.call(srvBBox))
        {
            bboxesMsg_ = srvBBox.response.bounding_boxes;
            if (bboxesMsg_.bounding_boxes.size() != 0)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
    }

    void DemoSharon::activateSuperquadricsComputation(bool activate)
    {
        sharon_msgs::ActivateSupercuadricsComputation srvActivate;
        srvActivate.request.activate = activate;

        if (clientActivateSuperquadricsComputation_.call(srvActivate))
        {
            ROS_INFO("[DemoSharon] ActivateSuperquadricsComputation: %d", (bool)srvActivate.request.activate);
        }
    }

    // Create a ROS action client to move TIAGo's head
    void DemoSharon::createClient(follow_joint_control_client_Ptr &actionClient, std::string name)
    {
        ROS_INFO("Creating action client to %s controller ...", name.c_str());

        std::string controller_name = name + "_controller/follow_joint_trajectory";

        actionClient.reset(new follow_joint_control_client(controller_name));

        int iterations = 0, max_iterations = 3;
        // Wait for arm controller action server to come up
        while (!actionClient->waitForServer(ros::Duration(0.5)) && ros::ok() && iterations < max_iterations)
        {
            ROS_DEBUG("Waiting for the %s_controller_action server to come up", name.c_str());
            ++iterations;
        }

        if (iterations == max_iterations)
            ROS_ERROR("createClient: %s controller action server not available", name.c_str());
    }

    // Generates a waypoint to move TIAGo's head
    void DemoSharon::waypointHeadGoal(control_msgs::FollowJointTrajectoryGoal &goal, const std::vector<float> &positions, const float &timeToReach)
    {
        // The joint names, which apply to all waypoints
        goal.trajectory.joint_names.push_back("head_1_joint");
        goal.trajectory.joint_names.push_back("head_2_joint");

        // Two waypoints in this goal trajectory
        goal.trajectory.points.resize(1);

        // First trajectory point
        // Positions
        int index = 0;
        goal.trajectory.points[index].positions.resize(2);
        goal.trajectory.points[index].positions[0] = positions[0];
        goal.trajectory.points[index].positions[1] = positions[1];

        // Velocities
        goal.trajectory.points[index].velocities.resize(2);
        for (int j = 0; j < 2; ++j)
        {
            goal.trajectory.points[index].velocities[j] = 0.0;
        }
        goal.trajectory.header.stamp = ros::Time(0);

        // To be reached 2 second after starting along the trajectory
        goal.trajectory.points[index].time_from_start = ros::Duration(timeToReach);
    }

    // Generates a waypoint to move TIAGo's torso
    void DemoSharon::waypointTorsoGoal(control_msgs::FollowJointTrajectoryGoal &goal, const float &position, const float &timeToReach)
    {
        // The joint names, which apply to all waypoints
        goal.trajectory.joint_names.push_back("torso_lift_joint");

        // Two waypoints in this goal trajectory
        goal.trajectory.points.resize(1);

        // First trajectory point
        // Positions
        int index = 0;
        goal.trajectory.points[index].positions.resize(1);
        goal.trajectory.points[index].positions[0] = position;

        // Velocities
        goal.trajectory.points[index].velocities.resize(1);
        goal.trajectory.points[index].velocities[0] = 0.0;

        // To be reached 2 second after starting along the trajectory
        goal.trajectory.header.stamp = ros::Time(0);

        goal.trajectory.points[index].time_from_start = ros::Duration(timeToReach);
    }

    void DemoSharon::waypointGripperGoal(std::string name, control_msgs::FollowJointTrajectoryGoal &goal, const float positions[2], const float &timeToReach)
    {
        // The joint names, which apply to all waypoints
        std::string right_finger = "gripper_" + name + "_right_finger_joint";
        std::string left_finger = "gripper_" + name + "_left_finger_joint";

        goal.trajectory.joint_names.push_back(right_finger);
        goal.trajectory.joint_names.push_back(left_finger);

        int index = 0;
        goal.trajectory.points.resize(1);
        goal.trajectory.points[index].positions.resize(2);
        goal.trajectory.points[index].positions[0] = positions[0];
        goal.trajectory.points[index].positions[1] = positions[1];

        // Velocities
        goal.trajectory.points[index].velocities.resize(2);
        for (int j = 0; j < 2; ++j)
        {
            goal.trajectory.points[index].velocities[j] = 0.0;
        }
        // To be reached 2 second after starting along the trajectory
        goal.trajectory.points[index].time_from_start = ros::Duration(timeToReach);
    }

    void DemoSharon::asrCallback(const std_msgs::StringConstPtr &asrMsg)
    {
        if (waitingForAsrCommand_)
        {
            asr_ = asrMsg->data;
            ROS_INFO("%s", asr_.c_str());
            // Además que el objeto sea uno de los detectados
            mtxASR_.lock();
            foundAsr_ = false;
            if (useGlasses_)
            {
                // Aqui se supone que ya tenemos el objeto que se quiere con la mirada
                if (sqCategories_[indexGlassesSqCategory_].category.find(asr_, 0) != std::string::npos)
                {
                    ROS_INFO("[DemoSharon] ASR command received is the same: %s", asr_.c_str());
                    ROS_INFO("[DemoSharon] We continue to grasp the same object");
                }
                else
                {
                    ROS_INFO("[DemoSharon] ASR command is different. Lets check if it's one of the detected objects by the robot.");
                    indexSqCategoryAsr_ = -1;
                    for (int i = 0; i < sqCategories_.size(); i++)
                    {
                        ROS_INFO("%s %d", sqCategories_[i].category.c_str(), sqCategories_.size());
                        if (sqCategories_[i].category.find(asr_, 0) != std::string::npos)
                        {
                            foundAsr_ = true;

                            indexSqCategoryAsr_ = i;
                            asrCommandReceived_ = true;
                            if (state_ = COMPUTE_GRASP_POSES )
                            {
                                ROS_INFO("KILLING THEARD FOR COMPUTE GRASP POSES");
                                threadComputeGraspPoses_->detach();
                                indexSqCategory_ = indexSqCategoryAsr_;
                                state_ = COMPUTE_GRASP_POSES;
                            }
                            break;
                        }
                    }

                    if(indexSqCategoryAsr_<0)
                    {
                        ROS_WARN("[DemoSharon] ASR request: %s NOT found.", asr_.c_str());
                        // robot should say I don't recognize this object.
                        if (state_ = COMPUTE_GRASP_POSES )
                        {
                            ROS_INFO("KILLING THEARD FOR COMPUTE GRASP POSES");
                            threadComputeGraspPoses_->detach();
                            indexSqCategory_ = indexSqCategoryAsr_;
                            state_ = COMPUTE_GRASP_POSES;
                        }
                    }

                }
            }
            else{

                indexSqCategoryAsr_ = -1;
                for (int i = 0; i < sqCategories_.size(); i++)
                {
                    ROS_INFO("%s %d", sqCategories_[i].category.c_str(), sqCategories_.size());
                    if (sqCategories_[i].category.find(asr_, 0) != std::string::npos)
                    {
                        foundAsr_ = true;

                        indexSqCategoryAsr_ = i;
                        asrCommandReceived_ = true;
                        if (state_ = COMPUTE_GRASP_POSES)
                        {
                            ROS_INFO("KILLING THEARD FOR COMPUTE GRASP POSES");
                            state_ = -1;
                            threadComputeGraspPoses_->detach();
                        }
                        break;
                    }
                    else
                    {
                        ROS_INFO("NOT FOUND");
                    }
                }
            }
            mtxASR_.unlock();

            }
        }

        void DemoSharon::computeGraspPosesThread()
        {
            ROS_INFO("This is the thread for computing the grasping poses.");
            ROS_INFO("Planning to move %s to a target pose expressed in %s", groupRightArmTorsoPtr_->getEndEffectorLink().c_str(), groupRightArmTorsoPtr_->getPlanningFrame().c_str());

            sharon_msgs::ComputeGraspPoses srvGraspingPoses;
            srvGraspingPoses.request.id = sqCategories_[indexSqCategory_].idSq;
            geometry_msgs::PoseArray graspingPoses, orderedGraspingPoses;
            groupRightArmTorsoPtr_->setStartStateToCurrentState();

            if (clientComputeGraspPoses_.call(srvGraspingPoses))
            {
                ROS_INFO("[DemoSharon] ComputeGraspPoses: %d", (bool)srvGraspingPoses.response.success);
                graspingPoses = srvGraspingPoses.response.poses;
            }
            ROS_INFO("[DemoSharon] NumberPoses: %d", (int)graspingPoses.poses.size());
            ros::Duration(10.0).sleep(); // Sleep for one second
            return;
        }

        void DemoSharon::glassesDataCallback(const sharon_msgs::GlassesData::ConstPtr &glassesData)
        {

            if (waitingForGlassesCommand_)
            {
                std::vector<double> decisionVector = glassesData->decision_vector;
                double key = 1.0;
                std::vector<double>::iterator itr = std::find(decisionVector.begin(), decisionVector.end(), key);
                if (itr != decisionVector.cend())
                {
                    if (std::distance(decisionVector.begin(), itr) != 0)
                    {
                        glassesCategory_ = glassesData->category;
                        glassesCommandReceived_ = true;

                        ROS_INFO("[DemoSharon] Glasses command to grasp %s", glassesData->category.c_str());
                    }
                }
            }
        }

        bool DemoSharon::computeIntersectionOverUnion(const std::array<int, 4> &bboxYolo, const std::array<int, 4> &bboxSq, float &IoU)
        {
            if (bboxYolo[0] < bboxSq[2] and bboxYolo[2] > bboxSq[0] and
                bboxYolo[1] < bboxSq[3] and bboxYolo[3] > bboxSq[1])
            {
                int xA = bboxYolo[0];
                if (bboxSq[0] > xA)
                {
                    xA = bboxSq[0];
                }

                int yA = bboxYolo[1];
                if (bboxSq[1] > yA)
                {
                    yA = bboxSq[1];
                }

                int xB = bboxYolo[2];
                if (bboxSq[2] < xB)
                {
                    xB = bboxSq[2];
                }

                int yB = bboxYolo[3];
                if (bboxSq[3] < yB)
                {
                    yB = bboxSq[3];
                }

                float interArea = (xB - xA) * (yB - yA);
                float yoloBboxArea = (bboxYolo[2] - bboxYolo[0]) * (bboxYolo[3] - bboxYolo[1]);
                float sqBboxArea = (bboxSq[2] - bboxSq[0]) * (bboxSq[3] - bboxSq[1]);

                ROS_INFO("InterArea: %f", interArea);
                ROS_INFO("Yolo bbox area: %f", yoloBboxArea);
                ROS_INFO("Superquadric bbox area: %f", sqBboxArea);

                IoU = interArea / (yoloBboxArea + sqBboxArea - interArea);

                ROS_INFO("IoU: %f", IoU);
                return true;
            }
            else
            {
                ROS_INFO("No IoU");
                IoU = 0;
                return false;
            }
        }
    }
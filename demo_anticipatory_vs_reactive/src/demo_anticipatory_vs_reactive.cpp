#include "demo_anticipatory_vs_reactive/demo_anticipatory_vs_reactive.hpp"

namespace demo_anticipatory_vs_reactive
{
    DemoAnticipatoryVsReactive::DemoAnticipatoryVsReactive(ros::NodeHandle nh) : nodeHandle_(nh)
    {
        ROS_INFO("[DemoAnticipatoryVsReactive] Node started.");

        // Need it asynspinner for moveit
        ros::AsyncSpinner spinner(2);
        spinner.start();
        init(); // Tiagos head and torso are at the initial positions
        startDemoTime_ = ros::Time::now();

        timesFile_ << "Use Glasses," << useGlasses_ << "\n";
        timesFile_ << "Use ASR," << useAsr_ << "\n";

        demoGlassesASR();
    }

    DemoAnticipatoryVsReactive::~DemoAnticipatoryVsReactive()
    {
    }

    // void DemoAnticipatoryVsReactive::orderGraspingPoses(bool rightArm, const geometry_msgs::PoseArray &graspingPoses, geometry_msgs::PoseArray &orderedGraspingPoses){
    //     if(rightArm){

    //     }
    // }

    void DemoAnticipatoryVsReactive::demoGlassesASR()
    {
        state_ = INITIALIZE;
        firstInState = true;
        while (ros::ok())
        {

            switch (state_)
            {

            case INITIALIZE:
            {
                std_msgs::String msg;
                msg.data = "INITIALIZING";
                statePublisher_.publish(msg);

                companion_msgs::ActivateASR srvActivateAsr;
                srvActivateAsr.request.activate = false;

                if (clientActivateAsr_.call(srvActivateAsr))
                {
                    ROS_INFO("[DemoAnticipatoryVsReactive] ActivateASR: %d", (bool)srvActivateAsr.request.activate);
                }

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

                tablePose.position.x = tablePosition3_[0];
                tablePose.position.y = tablePosition3_[1];
                tablePose.position.z = tablePosition3_[2];
                addTablePlanningScene(tableDimensions3_, tablePose, "table3");

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

                ROS_INFO("[DemoAnticipatoryVsReactive] Start the computation of the superquadrics.");

                // Start computation of the superquadrics from the pointcloud
                activateSuperquadricsComputation(true);
                ros::Duration(3.5).sleep(); // sleep for 2 seconds

                // Stop computation of the superquadrics from the pointcloud. Our objects don't move, so there is no need to
                // continuously recompute the superquadrics
                ROS_INFO("[DemoAnticipatoryVsReactive] Stop the computation of the superquadrics.");
                activateSuperquadricsComputation(false);
                ros::Duration(0.5).sleep(); // sleep for 2 seconds

                ROS_INFO("[DemoAnticipatoryVsReactive] Get the computed superquadrics.");
                // Get the previously computed superquadrics
                if (!getSuperquadrics())
                { // If it's empty, there is no objects to grasp
                    return;
                }

                msg.data = "INITIALIZING Superquadrics Available";
                statePublisher_.publish(msg);

                ROS_INFO("[DemoAnticipatoryVsReactive] We have %d supequadrics.", (int)superquadricsMsg_.superquadrics.size());
                ros::Duration(0.5).sleep(); // sleep for 2 seconds

                if (!getBoundingBoxesFromSupercuadrics())
                {
                    return;
                }
                msg.data = "INITIALIZING Superquadrics BBoxes";
                statePublisher_.publish(msg);

                for (int i = 0; i < bboxesMsg_.bounding_boxes.size(); i++)
                    ROS_INFO("id: %d tlx: %d tly: %d brx: %d bry:%d", bboxesMsg_.bounding_boxes[i].id,
                             bboxesMsg_.bounding_boxes[i].tlx, bboxesMsg_.bounding_boxes[i].tly,
                             bboxesMsg_.bounding_boxes[i].brx, bboxesMsg_.bounding_boxes[i].bry);

                superquadricsBBoxesPublisher_.publish(bboxesMsg_);

                sqCategories_.clear();
                addSupequadricsPlanningScene();

                msg.data = "INITIALIZING Superquadrics Added to the planning scene";
                statePublisher_.publish(msg);

                darknet_ros_msgs::BoundingBoxesConstPtr darknetBBoxesMsg = ros::topic::waitForMessage<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes");
                yoloBBoxesMsg_ = *darknetBBoxesMsg;

                msg.data = "INITIALIZING Darknet detection done";
                statePublisher_.publish(msg);

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

                msg.data = "INITIALIZING Categories associated with superquadrics";
                statePublisher_.publish(msg);

                for (int idx = 0; idx < sqCategories_.size(); idx++)
                {
                    ROS_INFO("id: %d category: %s", sqCategories_[idx].idSq, sqCategories_[idx].category.c_str());
                    std::stringstream ss;
                    ss << "id: " << sqCategories_[idx].idSq << " category: " << sqCategories_[idx].category;
                    msg.data = ss.str();
                    statePublisher_.publish(msg);
                }

                pal_interaction_msgs::TtsGoal goal;
                goal.rawtext.text = initVerbalMessage_;
                goal.rawtext.lang_id = "en_GB";

                acPtr_->sendGoal(goal);
                ros::Duration(1.5).sleep();

                srvActivateAsr.request.activate = true;

                if (clientActivateAsr_.call(srvActivateAsr))
                {
                    ROS_INFO("[DemoAnticipatoryVsReactive] ActivateASR: %d", (bool)srvActivateAsr.request.activate);
                }

                foundGlasses_ = false;
                indexGlassesSqCategory_ = -1;
                waitingForGlassesCommand_ = true;
                waitingForAsrCommand_ = true;
                firstInState = true;
                state_ = WAIT_FOR_COMMAND;
            }
            break;
            case WAIT_FOR_COMMAND:
            {
                if (firstInState)
                {
                    ROS_INFO("WAIT_FOR_COMMAND");
                    firstInState = false;
                }
                std_msgs::String msg;
                // msg.data = "WAIT_FOR_COMMAND";
                // statePublisher_.publish(msg);

                // ROS_INFO("[DemoAnticipatoryVsReactive] Wait for user's command.");
                if (!glassesCommandReceived_ && asrCommandReceived_)
                {
                    ROS_WARN("[DemoAnticipatoryVsReactive] The gaze should be faster than the asr.");
                    state_ = COMPUTE_GRASP_POSES;
                    firstInState = true;
                }
            }
            break;
            case COMPUTE_GRASP_POSES:
            {
                ROS_INFO("I'M IN COMPUTE_GRASP_POSES");
                // HERE WE SHOULD CREATE A THREAD THAT CAN BE KILLED IF NEEDED
                if (firstInState)
                {
                    pthread_create(&threadComputeGraspPoses_, NULL, &this->sendcomputeGraspPosesThreadWrapper, this);
                    void *result;
                    pthread_join(threadComputeGraspPoses_, &result);
                    if (result == PTHREAD_CANCELED)
                    {
                        if (alreadyAvailable_)
                        {
                            if (currentDecisionProb_ < thresholdExecuteTrajectory_)
                            {
                                state_ = -3;
                            }
                            else
                            {
                                state_ = WAIT_TO_EXECUTE;
                            }
                        }
                        else
                        {
                            state_ = -1;
                        }
                        firstInState = true;
                    }
                    else
                    {
                        firstInState = true;
                        state_ = FIND_REACHING_GRASP_IK;
                        indexGraspingPose_ = -1;
                    }
                    foundReachIk_ = false;
                }
            }
            break;
            case FIND_REACHING_GRASP_IK:
            {
                ROS_INFO("I'M IN FIND_REACHING_GRASP_IK");
                if (firstInState)
                {
                    std_msgs::String msg;
                    std::stringstream ss;
                    ss << "FIND_REACHING_GRASP_IK";
                    msg.data = ss.str();
                    statePublisher_.publish(msg);

                    foundReachIk_ = false;
                    pthread_create(&threadFindReachGraspIK_, NULL, &this->sendFindReachGraspIKThreadWrapper, this);
                    void *result;
                    ROS_INFO("WAIT IN FIND_REACHING_GRASP_IK");

                    pthread_join(threadFindReachGraspIK_, &result);
                    if (result == PTHREAD_CANCELED)
                    {
                        if (alreadyAvailable_)
                        {
                            if (currentDecisionProb_ < thresholdExecuteTrajectory_)
                            {
                                state_ = -3;
                            }
                            else
                            {
                                state_ = WAIT_TO_EXECUTE;
                            }
                        }
                        else
                        {
                            state_ = -1;
                        }
                        firstInState = true;
                    }
                    else
                    {
                        if (foundReachIk_)
                        {
                            firstInState = true;
                            mtxWriteFile_.lock();
                            feasibleReachingPoseTime_ = ros::Time::now();
                            double t_from_start = (feasibleReachingPoseTime_ - startDemoTime_).toSec();
                            timesFile_ << "Feasible reaching pose for object," << sqCategories_[indexSqCategory_].category << ","
                                       << ","
                                       << ","
                                       << ","
                                       << ",Seconds from demo start time," << t_from_start << "\n";
                            mtxWriteFile_.unlock();
                            state_ = PLAN_TO_REACHING_JOINTS;
                        }
                        else
                        {
                            firstInState = true;
                            state_ = UNABLE_TO_REACHING_GRASP_IK;
                        }
                    }
                }
                // while (ros::ok())
                // {
                //     ros::Duration(0.01).sleep();
                //     ROS_INFO("[DemoAnticipatoryVsReactive] FIND IK STATE");
                // }
            }
            break;
            case PLAN_TO_REACHING_JOINTS:
            {
                ROS_INFO("I'M IN PLAN_TO_REACHING_JOINTS");

                if (firstInState)
                {
                    ROS_INFO("Plan to reaching joints");
                    std_msgs::String msg;
                    std::stringstream ss;
                    ss << "PLAN_TO_REACHING_JOINTS";
                    msg.data = ss.str();
                    statePublisher_.publish(msg);

                    successPlanning_ = false;
                    pthread_create(&threadPlanToReachJoints_, NULL, &this->sendPlanToReachJointsThreadWrapper, this);
                    void *result;
                    ROS_INFO("WAIT IN PLAN_TO_REACHING_JOINTS");
                    pthread_join(threadPlanToReachJoints_, &result);
                    if (result == PTHREAD_CANCELED)
                    {
                        firstInState = true;
                        if (alreadyAvailable_)
                        {
                            if (currentDecisionProb_ < thresholdExecuteTrajectory_)
                            {
                                state_ = -3;
                            }
                            else
                            {
                                state_ = WAIT_TO_EXECUTE;
                            }
                        }
                        else
                        {
                            state_ = -1;
                        }
                    }
                    else
                    {
                        if (successPlanning_)
                        {
                            ROS_INFO("Success planning");

                            ss.clear();
                            ss << "success planning";
                            msg.data = ss.str();
                            statePublisher_.publish(msg);
                            TrajectoryToGraspObject trajectoryToGraspObject;
                            trajectoryToGraspObject.category = sqCategories_[indexSqCategory_].category;
                            trajectoryToGraspObject.idSq = sqCategories_[indexSqCategory_].idSq;
                            trajectoryToGraspObject.arm = arm_;
                            trajectoryToGraspObject.goalReachPose = reachingPose_;
                            trajectoryToGraspObject.goalReachJointValues = reachJointValues_;
                            trajectoryToGraspObject.goalGraspPose = graspingPoses_.poses[indexGraspingPose_];
                            trajectoryToGraspObject.plan = plan_;
                            if (arm_ == "right")
                            {
                                for (int i = 0; i <= 1; i++)
                                {
                                    trajectoryToGraspObject.closeGripperPositions[i] = width_[indexGraspingPose_] / 2.0 - closeRightGripperDeviation_[i] - closeMorePosition;
                                }
                            }
                            else
                            {
                                for (int i = 0; i <= 1; i++)
                                {
                                    trajectoryToGraspObject.closeGripperPositions[i] = width_[indexGraspingPose_] / 2.0 - closeLeftGripperDeviation_[i] - closeMorePosition;
                                }
                            }

                            listTrajectoriesToGraspObjects.push_back(trajectoryToGraspObject);
                            mtxWriteFile_.lock();
                            planTrajectoryReachingPoseTime_ = ros::Time::now();
                            timesFile_ << "Plan trajectory reaching pose for object," << sqCategories_[indexSqCategory_].category << ","
                                       << "arm,"
                                       << arm_ << ","
                                       << ","
                                       << ",Seconds from demo start time," << (planTrajectoryReachingPoseTime_ - startDemoTime_).toSec() << "\n";
                            mtxWriteFile_.unlock();
                            firstInState = true;
                            state_ = -3;
                        }
                        else
                        {
                            ss.clear();
                            ss << "Unable to plan a trajectory";
                            msg.data = ss.str();
                            statePublisher_.publish(msg);
                            firstInState = true;
                            state_ = FIND_REACHING_GRASP_IK;
                        }
                    }
                }
            }
            break;
            case WAIT_TO_EXECUTE:
            {
                ROS_INFO("I'M IN WAIT_TO_EXECUTE");
                if (((currentDecisionProb_ >= thresholdExecuteTrajectory_) && !waitingForGlassesCommand_) || (greaterThanExecutionThreshold_ && !waitingForGlassesCommand_))
                {

                    ROS_INFO("Gaze commands to grasp %s with decision prob %f greater than threshold %f", glassesCategory_.c_str(), currentDecisionProb_, thresholdExecuteTrajectory_);
                    ROS_INFO("Lets check if we have a trajectory stored to grasp the %s", glassesCategory_.c_str());

                    bool available = false;
                    for (int i = 0; i < listTrajectoriesToGraspObjects.size(); i++)
                    {
                        ROS_INFO("listTrajectoriesToGraspObjects %d category %s", i, listTrajectoriesToGraspObjects[i].category.c_str());
                        if (listTrajectoriesToGraspObjects[i].category.find(glassesCategory_, 0) != std::string::npos)
                        {
                            ROS_INFO("We already have a trajectory % s, so we should move to the execute trajectory", glassesCategory_.c_str());
                            indexListTrajectories_ = i;
                            available = true;
                            break;
                        }
                    }

                    ROS_INFO("alreadyAvailable: %d indexListTrajectories: %d", available, indexListTrajectories_);
                    if (available && indexListTrajectories_ >= 0)
                    {
                        firstInState = true;
                        state_ = EXECUTE_PLAN_TO_REACHING_JOINTS;
                    }
                    else
                    {
                        firstInState = true;
                        state_ = -1;
                    }
                }
            }
            break;
            case EXECUTE_PLAN_TO_REACHING_JOINTS:
            {

                if (firstInState)
                {
                    ROS_INFO("I'M IN EXECUTE_PLAN_TO_REACHING_JOINTS");
                    waitingForGlassesCommand_ = false;
                    stopMotion_ = false;
                    mtxWriteFile_.lock();
                    startExecutionTrajectoryTime_ = ros::Time::now();
                    timesFile_ << "Start Execution trajectory to reach," << listTrajectoriesToGraspObjects[indexListTrajectories_].category << ","
                               << "arm,"
                               << listTrajectoriesToGraspObjects[indexListTrajectories_].arm << ","
                               << ","
                               << ",Seconds from demo start time," << (startExecutionTrajectoryTime_ - startDemoTime_).toSec() << "\n";
                    mtxWriteFile_.unlock();

                    ROS_INFO("Executing planned trajectory to reaching joints");
                    std_msgs::String msg;
                    std::stringstream ss;
                    ss << "EXECUTE_PLAN_TO_REACHING_JOINTS";
                    msg.data = ss.str();
                    statePublisher_.publish(msg);

                    firstInState = false;

                    if (listTrajectoriesToGraspObjects[indexListTrajectories_].arm == "right")
                        moveit::planning_interface::MoveItErrorCode e = groupRightArmTorsoPtr_->asyncExecute(listTrajectoriesToGraspObjects[indexListTrajectories_].plan);
                    else
                    {
                        moveit::planning_interface::MoveItErrorCode e = groupLeftArmTorsoPtr_->asyncExecute(listTrajectoriesToGraspObjects[indexListTrajectories_].plan);
                    }
                    // const robot_state::RobotState &goalRobotState = groupRightArmTorsoPtr_->getJointValueTarget();
                    goalJoints_.clear();
                    goalJoints_ = listTrajectoriesToGraspObjects[indexListTrajectories_].goalReachJointValues;
                    // goalRobotState.copyJointGroupPositions(jointModelGroupTorsoRightArm_, goalJoints_);
                }
                else
                {
                    bool inGoal = false;
                    if (listTrajectoriesToGraspObjects[indexListTrajectories_].arm == "right")
                        inGoal = goalReached(groupRightArmTorsoPtr_);
                    else
                    {
                        inGoal = goalReached(groupLeftArmTorsoPtr_);
                    }
                    if (inGoal)
                    // if (groupRightArmTorsoPtr_->getMoveGroupClient().getState().isDone())
                    {
                        state_ = OPEN_GRIPPER;
                        firstInState = true;
                        std_msgs::String msg;
                        std::stringstream ss;
                        ss << "Trajectory done";
                        msg.data = ss.str();
                        statePublisher_.publish(msg);

                        ROS_INFO("Done execution");
                        mtxWriteFile_.lock();
                        reachingPoseTime_ = ros::Time::now();
                        timesFile_ << "Robot in reaching Pose trajectory," << listTrajectoriesToGraspObjects[indexListTrajectories_].category << ","
                                   << "arm,"
                                   << listTrajectoriesToGraspObjects[indexListTrajectories_].arm << ","
                                   << ","
                                   << ",Seconds from demo start time," << (reachingPoseTime_ - startDemoTime_).toSec() << "\n";
                        mtxWriteFile_.unlock();
                        stopMotion_ = false;
                    }
                    if (stopMotion_)
                    {
                        ROS_INFO("Stop motion");
                        std::stringstream ss;
                        ss.clear();
                        std_msgs::String msg;
                        ss << "Stop motion";
                        msg.data = ss.str();
                        statePublisher_.publish(msg);

                        // if (listTrajectoriesToGraspObjects[indexListTrajectories_].arm == "right")
                        groupRightArmTorsoPtr_->stop();
                        // else
                        // {
                        groupLeftArmTorsoPtr_->stop();
                        // }
                        ROS_INFO("Stop motion done");
                        // I need to clear the whole list since the planners include the torso
                        listTrajectoriesToGraspObjects.clear();

                        state_ = -1;
                        firstInState = true;
                        stopMotion_ = false;
                    }
                }
            }
            break;
            case OPEN_GRIPPER:
            {
                ROS_INFO("I'M IN OPEN_GRIPPER");

                if (waitingForAsrCommand_)
                {
                    if (firstInState)
                    {
                        ROS_INFO("HEY! I'M WAITING FOR CONFIRMATION...");
                        std_msgs::String msg;
                        std::stringstream ss;
                        ss << "Waiting for asr command";
                        msg.data = ss.str();
                        statePublisher_.publish(msg);

                        firstInState = false;
                    }
                }
                else
                {
                    // if (foundAsr_)
                    // {
                    //     state_ = -1;
                    //     firstInState = true;
                    //     foundAsr_ = false;
                    // }
                    // else
                    // {
                    ROS_INFO("OPEN GRIPPER!");

                    std_msgs::String msg;
                    std::stringstream ss;
                    ss << "Open Gripper";
                    msg.data = ss.str();
                    statePublisher_.publish(msg);
                    // Open gripper
                    moveGripper(openGripperPositions_, listTrajectoriesToGraspObjects[indexListTrajectories_].arm);
                    std::vector<std::string> objectIds;
                    objectIds.push_back("object_" + std::to_string(listTrajectoriesToGraspObjects[indexListTrajectories_].idSq));
                    planningSceneInterface_.removeCollisionObjects(objectIds);
                    ros::Duration(1.0).sleep(); // sleep for 1 seconds

                    firstInState = true;
                    state_ = APROACH_TO_GRASP;
                    // }
                }
            }
            break;
            case APROACH_TO_GRASP:
            {
                ROS_INFO("I'M IN APROACH_TO_GRASP");
                if (firstInState)
                {
                    ROS_INFO("Aproaching to object .....");
                    std_msgs::String msg;
                    std::stringstream ss;
                    ss << "Aproach to grasp";
                    msg.data = ss.str();
                    statePublisher_.publish(msg);
                    goToGraspingPose(listTrajectoriesToGraspObjects[indexListTrajectories_].goalGraspPose);
                    firstInState = false;
                }
                else
                {
                    moveit::planning_interface::MoveGroupInterface *groupAuxArmTorsoPtr_;
                    if (listTrajectoriesToGraspObjects[indexListTrajectories_].arm == "right")
                    {
                        groupAuxArmTorsoPtr_ = groupRightArmTorsoPtr_;
                    }
                    else
                    {
                        groupAuxArmTorsoPtr_ = groupLeftArmTorsoPtr_;
                    }

                    ROS_INFO("HERE!!!!!!!!!!......");
                    if (groupAuxArmTorsoPtr_->getMoveGroupClient().getState().isDone())
                    {
                        state_ = CLOSE_GRIPPER;
                        firstInState = true;
                    }
                }
            }
            break;
            case CLOSE_GRIPPER:
            {
                ROS_INFO("I'M IN CLOSE_GRIPPER");
                if (firstInState)
                {
                    std_msgs::String msg;
                    std::stringstream ss;
                    ss << "Object grasped";
                    msg.data = ss.str();
                    statePublisher_.publish(msg);

                    ROS_INFO("Closing gripper...");
                    firstInState = true;
                    moveGripper(listTrajectoriesToGraspObjects[indexListTrajectories_].closeGripperPositions, listTrajectoriesToGraspObjects[indexListTrajectories_].arm);
                    ros::Duration(0.2).sleep(); // sleep for 1 seconds

                    mtxWriteFile_.lock();
                    objectGrasppedTime_ = ros::Time::now();
                    timesFile_ << "Object grasped," << listTrajectoriesToGraspObjects[indexListTrajectories_].category << ","
                               << ","
                               << ","
                               << ","
                               << ",Seconds from demo start time," << (objectGrasppedTime_ - startDemoTime_).toSec() << "\n";
                    mtxWriteFile_.unlock();
                    state_ = GO_UP;
                }
            }
            break;
            case GO_UP:
            {
                ROS_INFO("I'M IN GO_UP");
                std_msgs::String msg;
                std::stringstream ss;
                ss << "Go Up";
                msg.data = ss.str();
                statePublisher_.publish(msg);

                moveit::planning_interface::MoveGroupInterface *groupAuxArmTorsoPtr_;
                if (listTrajectoriesToGraspObjects[indexListTrajectories_].arm == "right")
                {
                    groupAuxArmTorsoPtr_ = groupRightArmTorsoPtr_;
                }
                else
                {
                    groupAuxArmTorsoPtr_ = groupLeftArmTorsoPtr_;
                }
                goUp(groupAuxArmTorsoPtr_, 0.2);

                firstInState = true;
                state_ = RELEASE_OBJECT;

                mtxWriteFile_.lock();
                objectUpTime_ = ros::Time::now();
                timesFile_ << "Object up," << listTrajectoriesToGraspObjects[indexListTrajectories_].category << ","
                           << ","
                           << ","
                           << ","
                           << ",Seconds from demo start time," << (objectUpTime_ - startDemoTime_).toSec() << "\n";
                mtxWriteFile_.unlock();

                pal_interaction_msgs::TtsGoal goal;
                goal.rawtext.text = passObjectVerbalMessage_;
                goal.rawtext.lang_id = "en_GB";

                acPtr_->sendGoal(goal);
                ros::Duration(1.5).sleep();
            }
            break;
            case RELEASE_OBJECT:
            {
                ROS_INFO("I'M IN RELEASE_OBJECT");
                if (firstInState)
                {
                    firstInState = false;
                    releaseGripper_ = false;
                }
                else
                {
                    if (!releaseGripper_)
                    {
                        ROS_INFO("Waiting for command to release the gripper...");
                        ros::Duration(0.5).sleep(); // sleep for 1 seconds
                    }
                    else
                    {
                        std_msgs::String msg;
                        std::stringstream ss;
                        ss << "Release";
                        msg.data = ss.str();
                        statePublisher_.publish(msg);

                        moveGripper(openGripperPositions_, listTrajectoriesToGraspObjects[indexListTrajectories_].arm);
                        ros::Duration(1.0).sleep(); // sleep for 1 seconds
                        firstInState = true;
                        state_ = OBJECT_DELIVERED;
                    }
                }
            }
            break;
            case OBJECT_DELIVERED:
            {

                if (firstInState)
                {
                    ROS_INFO("[DemoAnticipatoryVsReactive] Object delivered!");
                    ROS_INFO("[DemoAnticipatoryVsReactive] Waiting for command to move to home position...");
                    firstInState = false;
                }
                else
                {
                    if (!moveToHomePosition_)
                    {
                        ros::Duration(0.5).sleep(); // sleep for 1 seconds
                    }
                    else
                    {
                        std_msgs::String msg;
                        std::stringstream ss;
                        ss << "Move to home position";
                        msg.data = ss.str();
                        statePublisher_.publish(msg);

                        float closeGripperPositions[2] = {0.0, 0.0};
                        moveGripper(closeGripperPositions, listTrajectoriesToGraspObjects[indexListTrajectories_].arm);
                        ros::Duration(1.0).sleep(); // sleep for 1 seconds

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

                        state_ = ROBOT_IN_HOME_POSITION;
                        firstInState = true;
                    }
                }
            }
            break;
            case ROBOT_IN_HOME_POSITION:
            {
                if (firstInState)
                {
                    ROS_INFO("[DemoAnticipatoryVsReactive] Robot in home position!");
                    firstInState = false;
                }
            }
            break;
            case -1:
            {
                ROS_INFO("[DemoAnticipatoryVsReactive] I'M IN -1");

                // indexSqCategory_ = indexSqCategoryAsr_;
                firstInState = true;
                state_ = COMPUTE_GRASP_POSES;
            }
            break;
            case UNABLE_TO_REACHING_GRASP_IK:
            {
                // ROS_INFO("I'M IN UNABLE_TO_REACHING_GRASP_IK");
                if (firstInState)
                {
                    ROS_WARN("[DemoAnticipatoryVsReactive] Unable to found ik to any of the possible reaching poses. What now?");
                    firstInState = false;
                }
            }
            break;
            case -3:
            {

                if (firstInState)
                {
                    ROS_INFO("[DemoAnticipatoryVsReactive] I'M IN -3");
                    firstInState = false;
                }

                if (foundAsr_)
                {
                    bool available = false;
                    for (int i = 0; i < listTrajectoriesToGraspObjects.size(); i++)
                    {
                        ROS_INFO("[DemoAnticipatoryVsReactive] listTrajectoriesToGraspObjects %d category %s", i, listTrajectoriesToGraspObjects[i].category.c_str());
                        if (listTrajectoriesToGraspObjects[i].category.find(asr_, 0) != std::string::npos)
                        {
                            ROS_INFO("[DemoAnticipatoryVsReactive] We already have a trajectory % s, so we should move to the execute trajectory", glassesCategory_.c_str());
                            indexListTrajectories_ = i;
                            available = true;
                            break;
                        }
                    }
                    if(available)
                        state_ = -4;
                    else{
                        firstInState = true;
                        state_ = COMPUTE_GRASP_POSES;

                    }
                }
                if(greaterThanExecutionThreshold_ && !waitingForGlassesCommand_ && !foundAsr_){
                    state_ = WAIT_TO_EXECUTE;
                    firstInState = true;
                }
            }
            break;
            case -4:
            {
                state_ = EXECUTE_PLAN_TO_REACHING_JOINTS;
                firstInState = true;
                stopMotion_ = false;
            }
            break;
            }
        }
    }

    bool DemoAnticipatoryVsReactive::goalReached(moveit::planning_interface::MoveGroupInterface *&groupArmTorsoPtr_)
    {
        std::vector<double> currentJoints = groupArmTorsoPtr_->getCurrentJointValues();

        for (int i = 0; i < currentJoints.size(); i++)
        {
            if ((currentJoints[i] < (goalJoints_[i] - goalJointTolerance_)) || (currentJoints[i] > (goalJoints_[i] + goalJointTolerance_)))
            {
                // ROS_INFO("CURRENT JOINT %d: %f is NOT CLOSE ENOUCH TO THE GOAL JOINT %f",i, currentJoints[i],goalJoints_[i]);
                return false;
            }
        }
        // ROS_INFO("GOAL REACHED");
        return true;
    }

    void DemoAnticipatoryVsReactive::moveGripper(const float positions[2], std::string name)
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
        ROS_INFO("[DemoAnticipatoryVsReactive] Setting gripper %s position: (%f ,%f)", name.c_str(), positions[0], positions[1]);
        waypointGripperGoal(name, gripperGoal, positions, 0.5);

        // Sends the command to start the given trajectory now
        gripperGoal.trajectory.header.stamp = ros::Time(0);
        auxGripperClient->sendGoal(gripperGoal);

        // Wait for trajectory execution
        while (!(auxGripperClient->getState().isDone()) && ros::ok())
        {
            ros::Duration(0.1).sleep(); // sleep for 1 seconds
        }
        ROS_INFO("[DemoAnticipatoryVsReactive] Gripper set to position: (%f, %f)", positions[0], positions[1]);
    }

    bool DemoAnticipatoryVsReactive::goUp(moveit::planning_interface::MoveGroupInterface *groupArmTorsoPtr, float upDistance)
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
            moveit::planning_interface::MoveItErrorCode e = groupArmTorsoPtr->move();
            if (e == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            {
                ROS_INFO("[DemoAnticipatoryVsReactive] Success in moving the grasped object up.");
                return true;
            }
            else
            {
                ROS_INFO("[DemoAnticipatoryVsReactive] Error in moving the grasped object up.");
                return false;
            }
        }
        else
        {
            ROS_INFO("[DemoAnticipatoryVsReactive] No feasible up pose!");
            return false;
        }
    }

    bool DemoAnticipatoryVsReactive::goToGraspingPose(const geometry_msgs::Pose &graspingPose)
    {
        moveit::planning_interface::MoveGroupInterface *groupAuxArmTorsoPtr_;
        if (listTrajectoriesToGraspObjects[indexListTrajectories_].arm == "right")
        {
            groupAuxArmTorsoPtr_ = groupRightArmTorsoPtr_;
        }
        else
        {
            groupAuxArmTorsoPtr_ = groupLeftArmTorsoPtr_;
        }
        groupAuxArmTorsoPtr_->setMaxVelocityScalingFactor(0.1);
        groupAuxArmTorsoPtr_->setMaxAccelerationScalingFactor(0.1);

        geometry_msgs::PoseStamped currentPose = groupAuxArmTorsoPtr_->getCurrentPose();
        KDL::Frame frameEndWrtBase;
        tf::poseMsgToKDL(currentPose.pose, frameEndWrtBase);
        KDL::Frame frameToolWrtEnd;
        frameToolWrtEnd.p[0] = +reachingDistance_;
        KDL::Frame frameToolWrtBase = frameEndWrtBase * frameToolWrtEnd;

        geometry_msgs::Pose toolPose;
        tf::poseKDLToMsg(frameToolWrtBase, toolPose);

        std::vector<geometry_msgs::Pose> waypoints;
        waypoints.push_back(toolPose);

        groupAuxArmTorsoPtr_->setStartStateToCurrentState();

        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.001;
        double fraction = groupAuxArmTorsoPtr_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        ROS_INFO("DemoAnticipatoryVsReactive] plan (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

        // for(int i=0; i<trajectory.joint_trajectory.points.size(); i++){
        //     ROS_INFO_STREAM(trajectory.joint_trajectory.points[i]);
        // }

        moveit::planning_interface::MoveGroupInterface::Plan planAproach;

        planAproach.trajectory_ = trajectory;

        sleep(1.0);

        moveit::planning_interface::MoveItErrorCode e = groupAuxArmTorsoPtr_->execute(planAproach);

        return true;
        // tf::poseKDLToMsg(frameToolWrtBase, toolPose);
        // groupRightArmTorsoPtr_->setPoseTarget(toolPose);

        // moveit::planning_interface::MoveItErrorCode code = groupRightArmTorsoPtr_->plan(plan_);
        // bool successPlanning = (code == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        // if (successPlanning)
        // {
        //     moveit::planning_interface::MoveItErrorCode e = groupRightArmTorsoPtr_->asyncMove();
        //     if (e == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        //     {
        //         ROS_INFO("[DemoAnticipatoryVsReactive] Success in moving the robot to the grasping pose.");
        //         return true;
        //     }
        //     else
        //     {
        //         ROS_INFO("[DemoAnticipatoryVsReactive] Error in moving the robot to the grasping pose.");
        //         return false;
        //     }
        // }
        // else
        // {
        //     ROS_INFO("[DemoAnticipatoryVsReactive] No feasible grasping pose!");
        //     return false;
        // }
    }

    bool DemoAnticipatoryVsReactive::goToAFeasibleReachingPose(const geometry_msgs::PoseArray &graspingPoses, int &indexFeasible)
    {

        bool successPlanning = false;
        robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematicModel_));
        kinematic_state->setToDefaultValues();
        for (int idx = 0; idx < graspingPoses.poses.size(); idx++)
        {
            ROS_INFO("[DemoAnticipatoryVsReactive] idx: %d", idx);
            ROS_INFO("DemoAnticipatoryVsReactive] Grasping Pose[%d]: %f %f %f", idx, graspingPoses.poses[idx].position.x, graspingPoses.poses[idx].position.y, graspingPoses.poses[idx].position.z);

            KDL::Frame frameEndWrtBase;
            tf::poseMsgToKDL(graspingPoses.poses[idx], frameEndWrtBase);
            KDL::Frame frameReachingWrtEnd;
            frameReachingWrtEnd.p[0] = -reachingDistance_ - DISTANCE_TOOL_LINK_GRIPPER_LINK;
            KDL::Frame frameReachingWrtBase = frameEndWrtBase * frameReachingWrtEnd;

            tf::poseKDLToMsg(frameReachingWrtBase, reachingPose_);

            bool found_ik = kinematic_state->setFromIK(jointModelGroupTorsoRightArm_, reachingPose_, 0.1);

            //     geometry_msgs::PoseStamped goal_pose;
            // goal_pose.header.frame_id = "base_footprint";
            // goal_pose.pose = graspingPoses.poses[idx];
            if (found_ik)
            {
                groupRightArmTorsoPtr_->setPoseTarget(reachingPose_);
                ROS_INFO("DemoAnticipatoryVsReactive] Set pose target");

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
                ROS_INFO("[DemoAnticipatoryVsReactive] Success in moving the robot to the reaching pose.");
                return true;
            }
            else
            {
                ROS_INFO("[DemoAnticipatoryVsReactive] Error in moving the robot to the reaching pose.");
                return false;
            }
        }
        else
        {
            ROS_INFO("[DemoAnticipatoryVsReactive] No feasible reaching pose found!");
            return false;
        }
    }

    void DemoAnticipatoryVsReactive::removeCollisionObjectsPlanningScene()
    {
        ROS_INFO("[DemoAnticipatoryVsReactive] Removing objects in the planningScene");
        std::vector<std::string> objectIds = planningSceneInterface_.getKnownObjectNames();
        planningSceneInterface_.removeCollisionObjects(objectIds);
        ros::Duration(1.0).sleep(); // sleep for 2 seconds
    }

    void DemoAnticipatoryVsReactive::addTablePlanningScene(const std::vector<float> &dimensions, const geometry_msgs::Pose &tablePose, const std::string &id)
    {
        ROS_INFO("[DemoAnticipatoryVsReactive] Add table collision objects to the planning scene");
        // Collision object
        moveit_msgs::CollisionObject collisionObject;
        collisionObject.id = id;
        collisionObject.header.frame_id = groupRightArmTorsoPtr_->getPlanningFrame();
        ROS_INFO("[DemoAnticipatoryVsReactive] Planning_frame: %s", groupRightArmTorsoPtr_->getPlanningFrame().c_str());
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

    void DemoAnticipatoryVsReactive::addSupequadricsPlanningScene()
    {
        std::vector<moveit_msgs::CollisionObject> collisionObjects;

        for (int i = 0; i < superquadricsMsg_.superquadrics.size(); i++)
        {
            companion_msgs::Superquadric superquadric = superquadricsMsg_.superquadrics[i];
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

            if (abs(superquadric.e1 - superquadric.e2) < 0.15)
            {
                if (superquadric.e1 <= elimit1_)
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
                else
                {
                    // CYLINDER
                    int ax = 0;
                    int index_height = 0;
                    float height = superquadric.a1;
                    if (superquadric.a2 > height)
                    {
                        height = superquadric.a2;
                        ax = 1;
                    }
                    if (superquadric.a3 > height)
                    {
                        height = superquadric.a3;
                        ax = 2;
                    }

                    float radius = 0;
                    if (superquadric.a1 > radius && ax != 0)
                    {
                        radius = superquadric.a1;
                    }
                    if (superquadric.a2 > radius && ax != 1)
                    {
                        radius = superquadric.a2;
                    }
                    if (superquadric.a3 > radius && ax != 2)
                    {
                        radius = superquadric.a3;
                    }

                    ROS_INFO("DemoAnticipatoryVsReactive] CYLINDER height: %f radius: %f", height, radius);
                    radius += inflateSize_;
                    height += inflateSize_;

                    primitive.type = primitive.CYLINDER;
                    primitive.dimensions.resize(2);
                    primitive.dimensions[0] = 2 * height;
                    primitive.dimensions[1] = radius;

                    collisionObject.primitives.push_back(primitive);
                    Eigen::AngleAxisf rot(M_PI / 2.0, Eigen::Vector3f::UnitY());
                    q = q * rot;
                    superquadricPose.orientation.x = q.x();
                    superquadricPose.orientation.y = q.y();
                    superquadricPose.orientation.z = q.z();
                    superquadricPose.orientation.w = q.w();
                    collisionObject.primitive_poses.push_back(superquadricPose);
                    collisionObject.operation = collisionObject.ADD;
                    collisionObjects.push_back(collisionObject);
                }
            }
            else
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

    void DemoAnticipatoryVsReactive::initializeHeadPosition(const std::vector<float> &initHeadPositions)
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

    void DemoAnticipatoryVsReactive::initializeTorsoPosition(float initTorsoPosition)
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

    bool DemoAnticipatoryVsReactive::initializeRightArmPosition(const std::vector<double> &initRightArmPositions)
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
                ROS_INFO("[DemoAnticipatoryVsReactive] Right Arm success in moving to the initial joints position.");
                return true;
            }
            else
            {
                ROS_INFO("[DemoAnticipatoryVsReactive] Right Arm Error in moving  to the initial joints position.");
                return false;
            }
        }
        else
        {
            return false;
        }
    }

    bool DemoAnticipatoryVsReactive::initializeLeftArmPosition(const std::vector<double> &initLeftArmPositions)
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
                ROS_INFO("[DemoAnticipatoryVsReactive] Left Arm success in moving to the initial joints position.");
                return true;
            }
            else
            {
                ROS_INFO("[DemoAnticipatoryVsReactive] Left Arm Error in moving  to the initial joints position.");
                return false;
            }
        }
        else
        {
            return false;
        }
    }

    void DemoAnticipatoryVsReactive::init()
    {
        ROS_INFO("[DemoAnticipatoryVsReactive] init().");

        ROS_INFO("[DemoAnticipatoryVsReactive] creating clients...");
        clientActivateSuperquadricsComputation_ = nodeHandle_.serviceClient<companion_msgs::ActivateSupercuadricsComputation>("/grasp_objects/activate_superquadrics_computation");
        clientGetSuperquadrics_ = nodeHandle_.serviceClient<companion_msgs::GetSuperquadrics>("/grasp_objects/get_superquadrics");
        clientComputeGraspPoses_ = nodeHandle_.serviceClient<companion_msgs::ComputeGraspPoses>("/grasp_objects/compute_grasp_poses");
        clientGetBboxesSuperquadrics_ = nodeHandle_.serviceClient<companion_msgs::GetBboxes>("/grasp_objects/get_bboxes_superquadrics");
        asrSubscriber_ = nodeHandle_.subscribe("/asr_node/data", 10, &DemoAnticipatoryVsReactive::asrCallback, this);
        glassesDataSubscriber_ = nodeHandle_.subscribe("/comms_glasses_server/data", 10, &DemoAnticipatoryVsReactive::glassesDataCallback, this);
        serviceReleaseGripper_ = nodeHandle_.advertiseService("/demo/release_gripper", &DemoAnticipatoryVsReactive::releaseGripper, this);
        serviceMoveToHomePosition_ = nodeHandle_.advertiseService("/demo/move_to_home_position", &DemoAnticipatoryVsReactive::moveToHomePosition, this);
        statePublisher_ = nodeHandle_.advertise<std_msgs::String>("/demo/state", 10);
        superquadricsBBoxesPublisher_ = nodeHandle_.advertise<companion_msgs::BoundingBoxes>("/demo/superquadrics_bboxes", 10);
        reachingPosePublisher_ = nodeHandle_.advertise<geometry_msgs::PoseStamped>("/demo/reaching_pose", 10);
        planPublisher_ = nodeHandle_.advertise<moveit_msgs::RobotTrajectory>("/demo/trajectory", 10);

        clientActivateAsr_ = nodeHandle_.serviceClient<companion_msgs::ActivateASR>("/asr_node/activate_asr");
        acPtr_ = new actionlib::SimpleActionClient<pal_interaction_msgs::TtsAction>("tts", true);

        ROS_INFO("Waiting for /tts action server to start.");
        acPtr_->waitForServer();
        ROS_INFO("/tts action server started.");

        ros::param::get("demo/use_asr", useAsr_);
        ros::param::get("demo/use_glasses", useGlasses_);
        ros::param::get("demo/reaching_distance", reachingDistance_);
        ros::param::get("demo/elimit1", elimit1_);
        ros::param::get("demo/elimit2", elimit2_);
        ros::param::get("demo/inflate_size", inflateSize_);
        ros::param::get("demo/max_error_joints", maxErrorJoints_);
        ros::param::get("demo/head_joints_position_init", initHeadPositions_);
        ros::param::get("demo/torso_joint_position_init", initTorsoPosition_);
        ros::param::get("demo/table_dimensions", tableDimensions_);
        ros::param::get("demo/table_position", tablePosition_);
        ros::param::get("demo/table_position2", tablePosition2_);
        ros::param::get("demo/table_dimensions2", tableDimensions2_);
        ros::param::get("demo/table_position3", tablePosition3_);
        ros::param::get("demo/table_dimensions3", tableDimensions3_);
        ros::param::get("demo/right_arm_joints_position_init", initRightArmPositions_);
        ros::param::get("demo/left_arm_joints_position_init", initLeftArmPositions_);
        ros::param::get("demo/threshold_plan_trajectory", thresholdPlanTrajectory_);
        ros::param::get("demo/threshold_execute_trajectory", thresholdExecuteTrajectory_);
        ros::param::get("demo/goal_joint_tolerance", goalJointTolerance_);
        ros::param::get("demo/init_demo_verbal_message", initVerbalMessage_);
        ros::param::get("demo/pass_object_verbal_message", passObjectVerbalMessage_);

        std::vector<float> tmp;
        ros::param::get("demo/left_gripper_position_difference_wrt_default", tmp);
        closeLeftGripperDeviation_[0] = tmp[0];
        closeLeftGripperDeviation_[1] = tmp[1];

        ros::param::get("demo/right_gripper_position_difference_wrt_default", tmp);
        closeRightGripperDeviation_[0] = tmp[0];
        closeRightGripperDeviation_[1] = tmp[1];

        ROS_INFO("[DemoAnticipatoryVsReactive] demo/reaching_distance set to %f", reachingDistance_);

        groupRightArmTorsoPtr_ = new moveit::planning_interface::MoveGroupInterface(nameTorsoRightArmGroup_);
        ROS_INFO("[DemoAnticipatoryVsReactive] Move group interface %s", nameTorsoRightArmGroup_.c_str());

        groupRightArmPtr_ = new moveit::planning_interface::MoveGroupInterface(nameRightArmGroup_);
        ROS_INFO("[DemoAnticipatoryVsReactive] Move group interface %s", nameRightArmGroup_.c_str());

        groupLeftArmTorsoPtr_ = new moveit::planning_interface::MoveGroupInterface(nameTorsoLeftArmGroup_);
        ROS_INFO("[DemoAnticipatoryVsReactive] Move group interface %s", nameTorsoLeftArmGroup_.c_str());

        groupLeftArmPtr_ = new moveit::planning_interface::MoveGroupInterface(nameLeftArmGroup_);
        ROS_INFO("[DemoAnticipatoryVsReactive] Move group interface %s", nameLeftArmGroup_.c_str());

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
        createClient(leftGripperClient_, std::string("gripper_left"));

        // createClient(torsoClient_, std::string("torso"));

        robot_model_loader::RobotModelLoader robotModelLoader_("robot_description");
        kinematicModel_ = robotModelLoader_.getModel();

        jointModelGroupTorsoRightArm_ = kinematicModel_->getJointModelGroup(nameTorsoRightArmGroup_);
        jointModelGroupTorsoLeftArm_ = kinematicModel_->getJointModelGroup(nameTorsoLeftArmGroup_);

        // goalJointTolerance_ = groupRightArmTorsoPtr_->getGoalJointTolerance();
        // ROS_INFO("GOAL JOINT TOLERANCE: %f", goalJointTolerance_);

        greaterThanExecutionThreshold_ = false;
        prevGlassesCategory_ = "background";
        glassesCategory_ = "background";

        auto start = std::chrono::system_clock::now();

        std::time_t start_time = std::chrono::system_clock::to_time_t(start);

        ROS_INFO("current_time: %s", std::ctime(&start_time));
        struct stat sb;

        std::string date = std::ctime(&start_time);

        std::replace(date.begin(), date.end(), ' ', '_');
        date.pop_back();

        if (boost::filesystem::exists(date + ".csv"))
            ROS_INFO("The path is valid!");
        else
        {
            std::string path = ros::package::getPath("demo_anticipatory_vs_reactive");
            timesFile_.open(path + "/csv/" + date + ".csv");
        }
        return;
    }

    bool DemoAnticipatoryVsReactive::releaseGripper(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {
        releaseGripper_ = true;
        return true;
    }

    bool DemoAnticipatoryVsReactive::moveToHomePosition(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {
        moveToHomePosition_ = true;
        return true;
    }

    bool DemoAnticipatoryVsReactive::getSuperquadrics()
    {
        companion_msgs::GetSuperquadrics srvSq;

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

    bool DemoAnticipatoryVsReactive::getBoundingBoxesFromSupercuadrics()
    {

        companion_msgs::GetBboxes srvBBox;
        ROS_INFO("[DemoAnticipatoryVsReactive] Get bounding boxes from superquadrics...");
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

    void DemoAnticipatoryVsReactive::activateSuperquadricsComputation(bool activate)
    {
        companion_msgs::ActivateSupercuadricsComputation srvActivate;
        srvActivate.request.activate = activate;

        if (clientActivateSuperquadricsComputation_.call(srvActivate))
        {
            ROS_INFO("[DemoAnticipatoryVsReactive] ActivateSuperquadricsComputation: %d", (bool)srvActivate.request.activate);
        }
    }

    // Create a ROS action client to move TIAGo's head
    void DemoAnticipatoryVsReactive::createClient(follow_joint_control_client_Ptr &actionClient, std::string name)
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
    void DemoAnticipatoryVsReactive::waypointHeadGoal(control_msgs::FollowJointTrajectoryGoal &goal, const std::vector<float> &positions, const float &timeToReach)
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
    void DemoAnticipatoryVsReactive::waypointTorsoGoal(control_msgs::FollowJointTrajectoryGoal &goal, const float &position, const float &timeToReach)
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

    void DemoAnticipatoryVsReactive::waypointGripperGoal(std::string name, control_msgs::FollowJointTrajectoryGoal &goal, const float positions[2], const float &timeToReach)
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

    void DemoAnticipatoryVsReactive::asrCallback(const std_msgs::StringConstPtr &asrMsg)
    {
        if (waitingForAsrCommand_ && useAsr_)
        {
            asr_ = asrMsg->data;
            ROS_INFO("%s", asr_.c_str());
            // Además que el objeto sea uno de los detectados
            mtxASR_.lock();
            foundAsr_ = false;
            if (useGlasses_)
            {
                if (indexGlassesSqCategory_ >= 0)
                {
                    // Aqui se supone que ya tenemos el objeto que se quiere con la mirada
                    if (glassesCategory_.find(asr_, 0) != std::string::npos)
                    {
                        ROS_INFO("[DemoAnticipatoryVsReactive] ASR command received is the same: %s", asr_.c_str());
                        ROS_INFO("[DemoAnticipatoryVsReactive] We continue to grasp the same object");
                        std_msgs::String msg;
                        std::stringstream ss;
                        ss << "ASR command received is the same: " << asr_.c_str();

                        mtxWriteFile_.lock();
                        asrTime_ = ros::Time::now();
                        timesFile_ << "Asr command to grasp object," << asr_ << ",same"
                                   << ","
                                   << ","
                                   << ","
                                   << ",Seconds from demo start time," << (asrTime_ - startDemoTime_).toSec() << "\n";
                        mtxWriteFile_.unlock();
                        msg.data = ss.str();
                        statePublisher_.publish(msg);
                        waitingForAsrCommand_ = false;
                    }
                    else
                    {
                        ROS_INFO("[DemoAnticipatoryVsReactive] ASR command is different. Lets check if it's one of the detected objects by the robot.");
                        std_msgs::String msg;
                        std::stringstream ss;
                        ss << "ASR command is different. Lets check if it's one of the detected objects by the robot";
                        msg.data = ss.str();
                        statePublisher_.publish(msg);

                        indexSqCategoryAsr_ = -1;

                        // it's different, so we need to check if we already have a trajectory available

                        for (int i = 0; i < sqCategories_.size(); i++)
                        {
                            ROS_INFO("%s %d", sqCategories_[i].category.c_str(), sqCategories_.size());
                            if (sqCategories_[i].category.find(asr_, 0) != std::string::npos)
                            {
                                foundAsr_ = true;

                                indexSqCategoryAsr_ = i;
                                asrCommandReceived_ = true;
                                waitingForAsrCommand_ = false;

                                mtxWriteFile_.lock();
                                asrTime_ = ros::Time::now();
                                timesFile_ << "Asr command to grasp object," << asr_ << ",different"
                                           << ","
                                           << ","
                                           << ","
                                           << ",Seconds from demo start time," << (asrTime_ - startDemoTime_).toSec() << "\n";
                                mtxWriteFile_.unlock();

                                if (state_ == COMPUTE_GRASP_POSES)
                                {
                                    ss.clear();
                                    ss << "CANCELL THREAD FOR COMPUTE GRASP POSES";
                                    msg.data = ss.str();
                                    statePublisher_.publish(msg);

                                    ROS_INFO("CANCELL THREAD FOR COMPUTE GRASP POSES");
                                    pthread_cancel(threadComputeGraspPoses_);
                                    ROS_INFO("CANCELLING THREAD!");
                                    indexSqCategory_ = indexSqCategoryAsr_;
                                }
                                else if (state_ == FIND_REACHING_GRASP_IK)
                                {
                                    ROS_INFO("CANCELL THREAD FOR FINDING IK");
                                    pthread_cancel(threadFindReachGraspIK_);
                                    ROS_INFO("CANCELLING THREAD!");

                                    ss.clear();
                                    ss << "CANCELL THREAD FOR FINDING IK";
                                    msg.data = ss.str();
                                    statePublisher_.publish(msg);

                                    indexSqCategory_ = indexSqCategoryAsr_;
                                }
                                else if (state_ == PLAN_TO_REACHING_JOINTS)
                                {
                                    ROS_INFO("CANCELL THREAD FOR PLAN_TO_REACHING_JOINTS");
                                    pthread_cancel(threadFindReachGraspIK_);
                                    ROS_INFO("CANCELLING THREAD!");
                                    indexSqCategory_ = indexSqCategoryAsr_;

                                    ss.clear();
                                    ss << "CANCELL THREAD FOR PLAN_TO_REACHING_JOINTS";
                                    msg.data = ss.str();
                                    statePublisher_.publish(msg);
                                }
                                else if (state_ == EXECUTE_PLAN_TO_REACHING_JOINTS)
                                {
                                    indexSqCategory_ = indexSqCategoryAsr_;
                                    stopMotion_ = true;
                                }
                                else if (state_ == OPEN_GRIPPER)
                                {
                                    indexSqCategory_ = indexSqCategoryAsr_;
                                }
                                else if (state_ == UNABLE_TO_REACHING_GRASP_IK)
                                {
                                    indexSqCategory_ = indexSqCategoryAsr_;
                                }
                                else if (state_ == -3)
                                {
                                    indexSqCategory_ = indexSqCategoryAsr_;
                                }
                                break;
                            }
                        }

                        if (indexSqCategoryAsr_ < 0)
                        {
                            ROS_WARN("[DemoAnticipatoryVsReactive] ASR request: %s NOT found.", asr_.c_str());
                            ss.clear();
                            ss << "ASR request: " << asr_ << "NOT FOUND";
                            msg.data = ss.str();
                            statePublisher_.publish(msg);

                            mtxWriteFile_.lock();
                            asrTime_ = ros::Time::now();
                            timesFile_ << "Asr command to grasp object," << glassesCategory_ << ",NOT found"
                                       << ","
                                       << ","
                                       << ","
                                       << ",Seconds from demo start time," << (asrTime_ - startDemoTime_).toSec() << "\n";
                            mtxWriteFile_.unlock();

                            // robot should say I don't recognize this object.
                            if (state_ = COMPUTE_GRASP_POSES)
                            {
                                ROS_INFO("KILLING THEARD FOR COMPUTE GRASP POSES");
                                pthread_cancel(threadComputeGraspPoses_);
                                state_ = -1;
                            }
                        }
                        else
                        {
                            // We have found the object in the list of available objects, Lets check if we have the object in the list of trajecotories

                            alreadyAvailable_ = false;
                            indexListTrajectories_ = -1;
                            for (int i = 0; i < listTrajectoriesToGraspObjects.size(); i++)
                            {
                                if (listTrajectoriesToGraspObjects[i].category.find(asr_, 0) != std::string::npos)
                                {
                                    ROS_INFO("We already have a trajectory so we should move to the execute trajectory");
                                    indexListTrajectories_ = i;
                                    alreadyAvailable_ = true;
                                    break;
                                }
                            }
                            if (state_ == EXECUTE_PLAN_TO_REACHING_JOINTS)
                            {
                                state_ = -1;
                            }
                            else
                            {
                                state_ = -4;
                            }
                        }
                    }
                }
                else
                {
                    ROS_WARN("[DemoAnticipatoryVsReactive] Glasses command should be faster than asr command.");
                    indexSqCategoryAsr_ = -1;
                    for (int i = 0; i < sqCategories_.size(); i++)
                    {
                        ROS_INFO("%s %d", sqCategories_[i].category.c_str(), sqCategories_.size());
                        if (sqCategories_[i].category.find(asr_, 0) != std::string::npos)
                        {
                            foundAsr_ = true;
                            asrTime_ = ros::Time::now();
                            timesFile_ << "Asr command to grasp object," << asr_ << ",same"
                                       << ","
                                       << ","
                                       << ","
                                       << ",Seconds from demo start time," << (asrTime_ - startDemoTime_).toSec() << "\n";
                            indexSqCategoryAsr_ = i;
                            indexSqCategory_ = indexSqCategoryAsr_;
                            asrCommandReceived_ = true;
                            waitingForAsrCommand_ = false;
                        }
                    }
                }
            }
            else
            {

                indexSqCategoryAsr_ = -1;
                for (int i = 0; i < sqCategories_.size(); i++)
                {
                    ROS_INFO("%s %d", sqCategories_[i].category.c_str(), sqCategories_.size());
                    if (sqCategories_[i].category.find(asr_, 0) != std::string::npos)
                    {
                        foundAsr_ = true;

                        indexSqCategoryAsr_ = i;
                        asrCommandReceived_ = true;
                        indexSqCategory_ = indexSqCategoryAsr_;
                        waitingForAsrCommand_ = false;

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

    void *DemoAnticipatoryVsReactive::sendcomputeGraspPosesThreadWrapper(void *object)
    {
        reinterpret_cast<DemoAnticipatoryVsReactive *>(object)->computeGraspPosesThread(NULL);
        return 0;
    }

    void *DemoAnticipatoryVsReactive::computeGraspPosesThread(void *ptr)
    {
        pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
        graspingPoses_.poses.clear();
        ROS_INFO("This is the thread for computing the grasping poses.");
        ROS_INFO("Planning to move %s to a target pose expressed in %s", groupRightArmTorsoPtr_->getEndEffectorLink().c_str(), groupRightArmTorsoPtr_->getPlanningFrame().c_str());
        ROS_INFO("Compute grasping poses of %s %d ", sqCategories_[indexSqCategory_].category.c_str(), sqCategories_[indexSqCategory_].idSq);

        std_msgs::String msg;
        std::stringstream ss;
        ss << "Compute grasping poses of " << sqCategories_[indexSqCategory_].category << " " << sqCategories_[indexSqCategory_].idSq;
        msg.data = ss.str();
        statePublisher_.publish(msg);

        companion_msgs::ComputeGraspPoses srvGraspingPoses;
        srvGraspingPoses.request.id = sqCategories_[indexSqCategory_].idSq;
        groupRightArmTorsoPtr_->setStartStateToCurrentState();

        if (clientComputeGraspPoses_.call(srvGraspingPoses))
        {
            ROS_INFO("[DemoAnticipatoryVsReactive] ComputeGraspPoses: %d", (bool)srvGraspingPoses.response.success);
            graspingPoses_ = srvGraspingPoses.response.poses;
            width_ = srvGraspingPoses.response.width;
        }
        ROS_INFO("[DemoAnticipatoryVsReactive] NumberPoses: %d", (int)graspingPoses_.poses.size());

        mtxWriteFile_.lock();
        computeGraspPosesTime_ = ros::Time::now();
        timesFile_ << "Compute grasp poses object," << sqCategories_[indexSqCategory_].category << ","
                   << ","
                   << ","
                   << ","
                   << ",Seconds from demo start time," << (computeGraspPosesTime_ - startDemoTime_).toSec() << "\n";
        mtxWriteFile_.unlock();

        // ros::Duration(4.0).sleep();
    }

    void *DemoAnticipatoryVsReactive::sendFindReachGraspIKThreadWrapper(void *object)
    {
        reinterpret_cast<DemoAnticipatoryVsReactive *>(object)->findReachGraspIKThread(NULL);
        return 0;
    }

    void *DemoAnticipatoryVsReactive::findReachGraspIKThread(void *ptr)
    {
        pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
        foundReachIk_ = false;
        ROS_INFO("This is the thread for computing the ik feasible poses.");
        robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematicModel_));
        kinematic_state->setToDefaultValues();

        std_msgs::String msg;
        std::stringstream ss;
        ss << "Find a feasible reaching pose";
        msg.data = ss.str();
        statePublisher_.publish(msg);

        for (int idx = indexGraspingPose_ + 1; idx < graspingPoses_.poses.size(); idx++)
        {
            // ROS_INFO("[DemoAnticipatoryVsReactive] idx: %d", idx);
            // ROS_INFO("Grasping Pose[%d]: %f %f %f", idx, graspingPoses_.poses[idx].position.x, graspingPoses_.poses[idx].position.y, graspingPoses_.poses[idx].position.z);

            KDL::Frame frameEndWrtBase;
            tf::poseMsgToKDL(graspingPoses_.poses[idx], frameEndWrtBase);
            KDL::Frame frameReachingWrtEnd;
            frameReachingWrtEnd.p[0] = -reachingDistance_ - DISTANCE_TOOL_LINK_GRIPPER_LINK;
            KDL::Frame frameReachingWrtBase = frameEndWrtBase * frameReachingWrtEnd;

            tf::poseKDLToMsg(frameReachingWrtBase, reachingPose_);

            if (graspingPoses_.poses[idx].position.y < 0.05)
            {
                arm_ = "right";
            }
            else
            {
                arm_ = "left";
            }

            if (arm_ == "right")
                foundReachIk_ = kinematic_state->setFromIK(jointModelGroupTorsoRightArm_, reachingPose_, 0.01);
            else
            {
                foundReachIk_ = kinematic_state->setFromIK(jointModelGroupTorsoLeftArm_, reachingPose_, 0.01);
            }
            //     geometry_msgs::PoseStamped goal_pose;
            // goal_pose.header.frame_id = "base_footprint";
            // goal_pose.pose = graspingPoses.poses[idx];
            if (foundReachIk_)
            {
                ROS_INFO("[DemoAnticipatoryVsReactive] IK Found!");
                ss.clear();
                ss << "IK Found";
                msg.data = ss.str();
                statePublisher_.publish(msg);

                geometry_msgs::PoseStamped reachingPoseStamped;
                reachingPoseStamped.header.frame_id = "base_footprint";
                reachingPoseStamped.header.stamp = ros::Time::now();
                reachingPoseStamped.pose = reachingPose_;
                reachingPosePublisher_.publish(reachingPoseStamped);

                reachJointValues_.clear();

                if (arm_ == "right")
                {
                    kinematic_state->copyJointGroupPositions(jointModelGroupTorsoRightArm_, reachJointValues_);
                }
                else
                {
                    kinematic_state->copyJointGroupPositions(jointModelGroupTorsoLeftArm_, reachJointValues_);
                }
                indexGraspingPose_ = idx;
                break;
            }
        }
        // ros::Duration(4.0).sleep();
    }

    void *DemoAnticipatoryVsReactive::sendPlanToReachJointsThreadWrapper(void *object)
    {
        reinterpret_cast<DemoAnticipatoryVsReactive *>(object)->planToReachJointsThread(NULL);
        return 0;
    }

    void *DemoAnticipatoryVsReactive::planToReachJointsThread(void *ptr)
    {
        pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
        successPlanning_ = false;

        if (arm_ == "right")
        {
            groupRightArmTorsoPtr_->setJointValueTarget(reachJointValues_);

            ROS_INFO("SET POSE TARGET");
            robot_state::RobotState start_state(*groupRightArmTorsoPtr_->getCurrentState());
            groupRightArmTorsoPtr_->setStartState(start_state);
            moveit::planning_interface::MoveItErrorCode code = groupRightArmTorsoPtr_->plan(plan_);
            // ros::Duration(4.0).sleep();
            successPlanning_ = (code == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if (successPlanning_)
                planPublisher_.publish(plan_.trajectory_);
        }
        else
        {
            groupLeftArmTorsoPtr_->setJointValueTarget(reachJointValues_);

            ROS_INFO("SET POSE TARGET");
            robot_state::RobotState start_state(*groupLeftArmTorsoPtr_->getCurrentState());
            groupLeftArmTorsoPtr_->setStartState(start_state);
            moveit::planning_interface::MoveItErrorCode code = groupLeftArmTorsoPtr_->plan(plan_);
            // ros::Duration(4.0).sleep();
            successPlanning_ = (code == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if (successPlanning_)
                planPublisher_.publish(plan_.trajectory_);
        }
    }

    void DemoAnticipatoryVsReactive::glassesDataCallback(const companion_msgs::GlassesData::ConstPtr &glassesData)
    {

        std_msgs::String msg;

        if (waitingForGlassesCommand_ && useGlasses_)
        {
            // ROS_INFO("waitingForGlassesCommand_");
            std::vector<double> decisionVector = glassesData->decision_vector;

            double key = 1.0;
            std::vector<double>::iterator itr = max_element(std::begin(decisionVector), std::end(decisionVector)); // C++11
            if (itr != decisionVector.cend())
            {
                if (std::distance(decisionVector.begin(), itr) != 0)
                {

                    // ---------------- In case they detect something that is not in the table, we ignore it ----------------------------------------//
                    bool object_in_the_table = false;
                    for (int i = 0; i < sqCategories_.size(); i++)
                    {
                        if (sqCategories_[i].category.find(glassesData->category, 0) != std::string::npos)
                        {
                            object_in_the_table = true;
                            break;
                        }
                    }
                    if (!object_in_the_table)
                    {
                        ROS_WARN("[DemoAnticipatoryVsReactive] Gaze command category %s is NOT available in the table.", glassesData->category.c_str());
                        return;
                    }
                    // -------------------------------------------------------------------------------------------------------------------------------//

                    currentDecisionProb_ = *itr;
                    if (currentDecisionProb_ > thresholdPlanTrajectory_) // The probability of the decision is greater than the planning threshold
                    {

                        ROS_INFO("[DemoAnticipatoryVsReactive] Glasses command to grasp %s prob: %f", glassesData->category.c_str(), currentDecisionProb_);
                        prevGlassesCategory_ = glassesCategory_;
                        glassesCategory_ = glassesData->category;

                        ROS_INFO("Prev: %s Current: %s", prevGlassesCategory_.c_str(), glassesCategory_.c_str());

                        // Check if we already have a planned trajectory to grasp the object
                        alreadyAvailable_ = false;
                        indexListTrajectories_ = -1;
                        for (int i = 0; i < listTrajectoriesToGraspObjects.size(); i++)
                        {
                            if (listTrajectoriesToGraspObjects[i].category.find(glassesCategory_, 0) != std::string::npos)
                            {
                                ROS_INFO("We already have a trajectory so we should move to the execute trajectory");
                                indexListTrajectories_ = i;
                                alreadyAvailable_ = true;
                                break;
                            }
                        }

                        // We dont have a planned trajectory to grasp the object, so we need to compute it
                        if (!alreadyAvailable_)
                        {
                            if (glassesCategory_ != prevGlassesCategory_) // In case we are planning the trajectory that is different, we stop all the processes that are running,
                                                                          // and we start the planning
                            {
                                mtxWriteFile_.lock();
                                gazeCommandTime_ = ros::Time::now();
                                if (currentDecisionProb_ < thresholdExecuteTrajectory_)
                                    timesFile_ << "Gaze object," << glassesCategory_ << ",Decision Prob.," << currentDecisionProb_ << ",>"
                                               << ",Threshold plan"
                                               << ",Seconds from demo start time," << (gazeCommandTime_ - startDemoTime_).toSec() << "\n";
                                else
                                    timesFile_ << "Gaze object," << glassesCategory_ << ",Decision Prob.," << currentDecisionProb_ << ",>"
                                               << ",Threshold execute"
                                               << ",Seconds from demo start time," << (gazeCommandTime_ - startDemoTime_).toSec() << "\n";

                                mtxWriteFile_.unlock();

                                // We gatther the identificiation number of the supercuadric associated to the object that we want to grasp
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

                                // If it has a id associated we continue with the planning

                                if (indexGlassesSqCategory_ >= 0)
                                {
                                    ROS_INFO("[DemoAnticipatoryVsReactive] Gaze command category %s is  available in the table.", sqCategories_[indexGlassesSqCategory_].category.c_str());
                                    std::stringstream ss;
                                    ss << "Gaze command category: " << sqCategories_[indexGlassesSqCategory_].category << " is  available in the table.";
                                    msg.data = ss.str();
                                    statePublisher_.publish(msg);
                                    waitingForGlassesCommand_ = true;
                                    waitingForAsrCommand_ = true;

                                    if (state_ == COMPUTE_GRASP_POSES) // If we are computing the grasp poses, we cancel the thread for computing the grasp poses
                                    {

                                        ROS_INFO("CANCELL THREAD FOR COMPUTE GRASP POSES");
                                        pthread_cancel(threadComputeGraspPoses_);
                                        ROS_INFO("CANCELLING THREAD!");
                                        indexSqCategory_ = indexGlassesSqCategory_;
                                        state_ = -1;
                                    }

                                    else if (state_ == FIND_REACHING_GRASP_IK) // If we are finding the IK for reaching pose, we cancel the thread for finding the IK for reaching pose
                                    {
                                        ROS_INFO("CANCELL THREAD FOR FINDING IK");
                                        pthread_cancel(threadFindReachGraspIK_);
                                        ROS_INFO("CANCELLING THREAD!");

                                        ss.clear();
                                        ss << "CANCELL THREAD FOR FINDING IK";
                                        msg.data = ss.str();
                                        statePublisher_.publish(msg);

                                        indexSqCategory_ = indexGlassesSqCategory_;
                                        state_ = -1;
                                    }
                                    else if (state_ == PLAN_TO_REACHING_JOINTS) // If we are planning the trajectory to reach a different object, we cancel the thread for planning the reaching trajectory to the different object
                                    {
                                        ROS_INFO("CANCELL THREAD FOR PLAN_TO_REACHING_JOINTS");
                                        pthread_cancel(threadFindReachGraspIK_);
                                        ROS_INFO("CANCELLING THREAD!");
                                        indexSqCategory_ = indexGlassesSqCategory_;

                                        ss.clear();
                                        ss << "CANCELL THREAD FOR PLAN_TO_REACHING_JOINTS";
                                        msg.data = ss.str();
                                        statePublisher_.publish(msg);
                                        state_ = -1;
                                    }
                                    else if (state_ == WAIT_TO_EXECUTE)
                                    {
                                        firstInState = true;
                                        state_ = -3;
                                    }
                                    else if (state_ == -3)
                                    {
                                        firstInState = true;
                                        state_ = -1;
                                    }
                                    else if (state_ == UNABLE_TO_REACHING_GRASP_IK)
                                    {
                                        indexSqCategory_ = indexGlassesSqCategory_;
                                        state_ = -1;
                                    }
                                    else
                                    {
                                        state_ = -1;
                                    }
                                    if (currentDecisionProb_ > thresholdExecuteTrajectory_)
                                    {
                                        greaterThanExecutionThreshold_ = true;
                                        waitingForGlassesCommand_ = false;
                                        decisisionProbGreaterExecuteThreshold_ = ros::Time::now();
                                        timesFile_ << "Gaze object," << glassesCategory_ << ",Decision Prob.," << currentDecisionProb_ << ",>"
                                                   << ",Threshold execute"
                                                   << ",Seconds from demo start time," << (decisisionProbGreaterExecuteThreshold_ - startDemoTime_).toSec() << "\n";
                                    }
                                }
                                else
                                {

                                    ROS_WARN("[DemoAnticipatoryVsReactive] Gaze command category %s is NOT available in the table.");
                                    std::stringstream ss;
                                    ss << "Gaze command category: " << sqCategories_[indexGlassesSqCategory_].category << " is NOT available in the table.";
                                    // msg.data = ss.str();
                                    // statePublisher_.publish(msg);
                                    // waitingForGlassesCommand_ = true;
                                    // waitingForAsrCommand_ = true;
                                    // glassesCommandReceived_ = false;
                                }
                            }
                        }
                        else
                        {
                            if (currentDecisionProb_ < thresholdExecuteTrajectory_)
                            {
                                if (state_ == COMPUTE_GRASP_POSES)
                                {

                                    ROS_INFO("CANCELL THREAD FOR COMPUTE GRASP POSES");
                                    pthread_cancel(threadComputeGraspPoses_);
                                    ROS_INFO("CANCELLING THREAD!");
                                    state_ = -3;
                                }
                                else if (state_ == FIND_REACHING_GRASP_IK)
                                {
                                    ROS_INFO("CANCELL THREAD FOR FINDING IK");
                                    pthread_cancel(threadFindReachGraspIK_);
                                    ROS_INFO("CANCELLING THREAD!");
                                    state_ = -3;

                                    // ss.clear();
                                    // ss << "CANCELL THREAD FOR FINDING IK";
                                    // msg.data = ss.str();
                                    // statePublisher_.publish(msg);

                                    // indexSqCategory_ = indexGlassesSqCategory_;
                                    // state_ = -1;
                                }
                                else if (state_ == PLAN_TO_REACHING_JOINTS)
                                {
                                    ROS_INFO("CANCELL THREAD FOR PLAN_TO_REACHING_JOINTS");
                                    pthread_cancel(threadFindReachGraspIK_);
                                    ROS_INFO("CANCELLING THREAD!");
                                    state_ = -3;
                                    // indexSqCategory_ = indexGlassesSqCategory_;

                                    // ss.clear();
                                    // ss << "CANCELL THREAD FOR PLAN_TO_REACHING_JOINTS";
                                    // msg.data = ss.str();
                                    // statePublisher_.publish(msg);
                                    // state_ = -1;
                                }
                            }
                            else
                            {
                                waitingForGlassesCommand_ = false;
                                decisisionProbGreaterExecuteThreshold_ = ros::Time::now();
                                timesFile_ << "Gaze object," << glassesCategory_ << ",Decision Prob.," << currentDecisionProb_ << ",>"
                                           << ",Threshold execute"
                                           << ",Seconds from demo start time," << (decisisionProbGreaterExecuteThreshold_ - startDemoTime_).toSec() << "\n";
                                if (state_ == COMPUTE_GRASP_POSES)
                                {

                                    ROS_INFO("CANCELL THREAD FOR COMPUTE GRASP POSES");
                                    pthread_cancel(threadComputeGraspPoses_);
                                    ROS_INFO("CANCELLING THREAD!");
                                }
                                else if (state_ == FIND_REACHING_GRASP_IK)
                                {
                                    ROS_INFO("CANCELL THREAD FOR FINDING IK");
                                    pthread_cancel(threadFindReachGraspIK_);
                                    ROS_INFO("CANCELLING THREAD!");

                                    // ss.clear();
                                    // ss << "CANCELL THREAD FOR FINDING IK";
                                    // msg.data = ss.str();
                                    // statePublisher_.publish(msg);

                                    // indexSqCategory_ = indexGlassesSqCategory_;
                                    // state_ = -1;
                                }
                                else if (state_ == PLAN_TO_REACHING_JOINTS)
                                {
                                    ROS_INFO("CANCELL THREAD FOR PLAN_TO_REACHING_JOINTS");
                                    pthread_cancel(threadFindReachGraspIK_);
                                    ROS_INFO("CANCELLING THREAD!");
                                    // indexSqCategory_ = indexGlassesSqCategory_;

                                    // ss.clear();
                                    // ss << "CANCELL THREAD FOR PLAN_TO_REACHING_JOINTS";
                                    // msg.data = ss.str();
                                    // statePublisher_.publish(msg);
                                    // state_ = -1;
                                }
                                state_ = WAIT_TO_EXECUTE;
                            }
                        }
                    }
                }
            }
        }
    }

    bool DemoAnticipatoryVsReactive::computeIntersectionOverUnion(const std::array<int, 4> &bboxYolo, const std::array<int, 4> &bboxSq, float &IoU)
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
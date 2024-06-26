#include "demo_anticipatory_vs_reactive/basic_demo_asr.hpp"

namespace demo_anticipatory_vs_reactive
{
  BasicDemoAsr::BasicDemoAsr(ros::NodeHandle nh) : nodeHandle_(nh)
  {
    ROS_INFO("[BasicDemoAsr] Node started.");

    // Need it asynspinner for moveit
    ros::AsyncSpinner spinner(2);
    spinner.start();
    init(); // Tiagos head and torso are at the initial positions
    startDemoTime_ = ros::Time::now();

    demoASR();
  }

  BasicDemoAsr::~BasicDemoAsr()
  {
  }

  void BasicDemoAsr::demoASR()
  {
    state_ = INITIALIZE;
    firstInState = true;
    stopDemo_ = false;
    okDemo_ = false;

    maxTorsoPosition_ = 0.35;
    while (ros::ok() && !stopDemo_)
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
          ROS_INFO("[BasicDemoAsr] ActivateASR: %d", (bool)srvActivateAsr.request.activate);
        }

        waitingForAsrCommand_ = false;
        asrCommandReceived_ = false;
        waitingForGlassesCommand_ = false;

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

        initializeTorsoPosition(initTorsoPosition_, 3.0);

        if (!initializeRightArmPosition(initRightArmPositions_))
        {
          return;
        }

        if (!initializeLeftArmPosition(initLeftArmPositions_))
        {
          return;
        }

        initializeHeadPosition(initHeadPositions_);

        ros::Duration(1.5).sleep(); // sleep for 2 seconds

        activateDetectHitFt(true);

        ROS_INFO("[BasicDemoAsr] Start the computation of the superquadrics.");

        // Start computation of the superquadrics from the pointcloud
        activateSuperquadricsComputation(true);
        ros::Duration(3.5).sleep(); // sleep for 2 seconds

        // Stop computation of the superquadrics from the pointcloud. Our objects don't move, so there is no need to
        // continuously recompute the superquadrics
        ROS_INFO("[BasicDemoAsr] Stop the computation of the superquadrics.");
        activateSuperquadricsComputation(false);
        ros::Duration(0.5).sleep(); // sleep for 2 seconds

        ROS_INFO("[BasicDemoAsr] Get the computed superquadrics.");
        // Get the previously computed superquadrics
        if (!getSuperquadrics())
        { // If it's empty, there is no objects to grasp
          return;
        }

        msg.data = "INITIALIZING Superquadrics Available";
        statePublisher_.publish(msg);

        ROS_INFO("[BasicDemoAsr] We have %d supequadrics.", (int)superquadricsMsg_.superquadrics.size());
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

        while (!initDemo_)
        {
          ;
        }

        pal_interaction_msgs::TtsGoal goal;
        goal.rawtext.text = initVerbalMessage_;
        goal.rawtext.lang_id = "en_GB";

        acPtr_->sendGoal(goal);
        ros::Duration(1.5).sleep();

        srvActivateAsr.request.activate = true;

        if (clientActivateAsr_.call(srvActivateAsr))
        {
          ROS_INFO("[BasicDemoAsr] ActivateASR: %d", (bool)srvActivateAsr.request.activate);
        }

        indexGlassesSqCategory_ = -1;
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

        if (asrCommandReceived_)
        {
          ROS_WARN("[BasicDemoAsr] The gaze should be faster than the asr.");
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
                if (trajectoryToGraspObject.category == "sliced_bread")
                  closeMorePosition += 0.008;
                else if (trajectoryToGraspObject.category == "coffee")
                  closeMorePosition += 0.005;

                for (int i = 0; i <= 1; i++)
                {
                  trajectoryToGraspObject.closeGripperPositions[i] = width_[indexGraspingPose_] / 2.0 - closeRightGripperDeviation_[i] - closeMorePosition;
                }
              }
              else
              {
                if (trajectoryToGraspObject.category == "sliced_bread")
                  closeMorePosition += 0.008;
                else if (trajectoryToGraspObject.category == "coffee")
                  closeMorePosition += 0.005;

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
          if (foundAsrDifferent_)
          {
            state_ = -3;
            foundAsrDifferent_ = false;
          }
          else
          {
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
          }
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
            // state_ = CLOSE_GRIPPER;
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
          activateDetectHitFt(false);
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
        float go_up_distance = 0.1;
        std_msgs::String msg;
        std::stringstream ss;
        ss << "Go Up";
        msg.data = ss.str();
        statePublisher_.publish(msg);

        control_msgs::FollowJointTrajectoryGoal torsoGoal;

        sensor_msgs::JointStateConstPtr jointStatesMsgPtr = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");

        float torsoPosition = jointStatesMsgPtr->position[find(jointStatesMsgPtr->name.begin(), jointStatesMsgPtr->name.end(), std::string("torso_lift_joint")) - jointStatesMsgPtr->name.begin()];

        if (maxTorsoPosition_ >= torsoPosition + go_up_distance)
        {
          initializeTorsoPosition(initTorsoPosition_, 0.5);
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
          goUp(groupAuxArmTorsoPtr_, go_up_distance);
        }

        firstInState = true;
        state_ = DEBUG_STATE;

        mtxWriteFile_.lock();
        objectUpTime_ = ros::Time::now();
        timesFile_ << "Object up," << listTrajectoriesToGraspObjects[indexListTrajectories_].category << ","
                   << ","
                   << ","
                   << ","
                   << ",Seconds from demo start time," << (objectUpTime_ - startDemoTime_).toSec() << "\n";
        mtxWriteFile_.unlock();

        // pal_interaction_msgs::TtsGoal goal;
        // goal.rawtext.text = passObjectVerbalMessage_;
        // goal.rawtext.lang_id = "en_GB";

        // acPtr_->sendGoal(goal);
        // ros::Duration(1.5).sleep();
      }
      break;
      case BRING_CLOSER:
      {
        ROS_INFO("I'M IN BRING CLOSER");
        std_msgs::String msg;
        std::stringstream ss;
        ss << "Bring closer";
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

        bringCloser(groupAuxArmTorsoPtr_, 0.1);

        firstInState = true;
        state_ = GO_BACKWARDS;

        mtxWriteFile_.lock();
        bringCloserTime_ = ros::Time::now();
        timesFile_ << "Bring closer," << listTrajectoriesToGraspObjects[indexListTrajectories_].category << ","
                   << ","
                   << ","
                   << ","
                   << ",Seconds from demo start time," << (bringCloserTime_ - startDemoTime_).toSec() << "\n";
        mtxWriteFile_.unlock();
      }
      break;
      case GO_BACKWARDS:
      {
        if (firstInState)
        {
          ROS_INFO("I'M IN GO_BACKWARDS");
          goBackwardsStartTime_ = ros::Time::now();
          firstInState = false;
        }
        double speed = 0.085;
        if ((ros::Time::now() - goBackwardsStartTime_).toSec() > (abs(0.4 / speed)))
        {
          geometry_msgs::Twist vel;
          vel.linear.x = 0.0;
          cmdVelPublisher_.publish(vel);
          firstInState = true;
          state_ = TURN_ANTICLOCKWISE;
        }
        else
        {
          geometry_msgs::Twist vel;
          vel.linear.x = -speed;
          cmdVelPublisher_.publish(vel);
        }
      }
      break;
      case TURN_ANTICLOCKWISE:
      {
        ROS_INFO("I'M IN TURN_ANTICLOCKWISE");
        if (firstInState)
        {
          firstInState = false;
          turnAnticlockwiseStartTime_ = ros::Time::now();
          geometry_msgs::Twist vel;
          vel.linear.x = 0.0;
          cmdVelPublisher_.publish(vel);
        }
        else
        {
          double speed = 0.24;
          double angle = 0;
          if (listTrajectoriesToGraspObjects[indexListTrajectories_].arm == "right")
          {
            angle = 90 * M_PI / 180.0;
          }
          else
          {
            angle = 70 * M_PI / 180.0;
          }
          if ((ros::Time::now() - turnAnticlockwiseStartTime_).toSec() > (abs(angle / speed)))
          {
            geometry_msgs::Twist vel;
            vel.angular.z = 0.0;
            cmdVelPublisher_.publish(vel);
            firstInState = true;
            state_ = RELEASE_OBJECT;
          }
          else
          {
            geometry_msgs::Twist vel;
            vel.angular.z = speed;
            cmdVelPublisher_.publish(vel);
          }
        }
      }

      break;
      case RELEASE_OBJECT:
      {
        ROS_INFO("I'M IN RELEASE_OBJECT");
        if (firstInState)
        {
          mtxWriteFile_.lock();
          objectDeliveredTime_ = ros::Time::now();
          timesFile_ << "Object delivered," << listTrajectoriesToGraspObjects[indexListTrajectories_].category << ","
                     << ","
                     << ","
                     << ","
                     << ",Seconds from demo start time," << (objectDeliveredTime_ - startDemoTime_).toSec() << "\n";
          mtxWriteFile_.unlock();
          pal_interaction_msgs::TtsGoal goal;
          goal.rawtext.text = passObjectVerbalMessage_;
          goal.rawtext.lang_id = "en_GB";

          acPtr_->sendGoal(goal);
          ros::Duration(1.5).sleep();
          firstInState = false;
          releaseGripper_ = false;
          geometry_msgs::Twist vel;
          vel.linear.x = 0.0;
          cmdVelPublisher_.publish(vel);
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
            activateDetectHitFt(true);
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
          ROS_INFO("[BasicDemoAsr] Object delivered!");

          ROS_INFO("[BasicDemoAsr] Waiting for command to move to home position...");
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
            timesFile_ << "Object delivered," << okDemo_;
            std_msgs::String msg;
            std::stringstream ss;
            ss << "Move to home position";
            msg.data = ss.str();
            statePublisher_.publish(msg);

            float closeGripperPositions[2] = {0.0, 0.0};
            moveGripper(closeGripperPositions, listTrajectoriesToGraspObjects[indexListTrajectories_].arm);
            ros::Duration(1.0).sleep(); // sleep for 1 seconds

            initializeTorsoPosition(initTorsoPosition_, 3.0);

            if (!initializeRightArmPosition(initRightArmPositions_))
            {
              return;
            }

            if (!initializeLeftArmPosition(initLeftArmPositions_))
            {
              return;
            }

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
          ROS_INFO("[BasicDemoAsr] Robot in home position!");
          firstInState = false;
        }
      }
      break;
      case -1:
      {
        ROS_INFO("[BasicDemoAsr] I'M IN -1");

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
          ROS_WARN("[BasicDemoAsr] Unable to found ik to any of the possible reaching poses. What now?");
          firstInState = false;
        }
      }
      break;
      case -3:
      {

        if (firstInState)
        {
          ROS_INFO("[BasicDemoAsr] I'M IN -3");
          firstInState = false;
        }

        if (foundAsr_)
        {
          bool available = false;
          for (int i = 0; i < listTrajectoriesToGraspObjects.size(); i++)
          {
            ROS_INFO("[BasicDemoAsr] listTrajectoriesToGraspObjects %d category %s", i, listTrajectoriesToGraspObjects[i].category.c_str());
            if (listTrajectoriesToGraspObjects[i].category.find(asr_, 0) != std::string::npos)
            {
              ROS_INFO("[BasicDemoAsr] We already have a trajectory % s, so we should move to the execute trajectory", glassesCategory_.c_str());
              indexListTrajectories_ = i;
              available = true;
              break;
            }
          }
          if (available)
            state_ = -4;
          else
          {
            firstInState = true;
            state_ = COMPUTE_GRASP_POSES;
          }
        }
        if (greaterThanExecutionThreshold_ && !waitingForGlassesCommand_ && !foundAsr_)
        {
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

      case DEBUG_STATE:
      {
        if (firstInState)
        {
          ROS_INFO("I'M IN DEBUG_STATE");
          std_msgs::String msg;
          std::stringstream ss;
          ss << "Debug state";
          msg.data = ss.str();
          statePublisher_.publish(msg);
          firstInState = false;
        }
      }
      break;
      }
    }
  }

  bool BasicDemoAsr::goalReached(moveit::planning_interface::MoveGroupInterface *&groupArmTorsoPtr_)
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

  void BasicDemoAsr::moveGripper(const float positions[2], std::string name)
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
    ROS_INFO("[BasicDemoAsr] Setting gripper %s position: (%f ,%f)", name.c_str(), positions[0], positions[1]);
    waypointGripperGoal(name, gripperGoal, positions, 0.5);

    // Sends the command to start the given trajectory now
    gripperGoal.trajectory.header.stamp = ros::Time(0);
    auxGripperClient->sendGoal(gripperGoal);

    // Wait for trajectory execution
    while (!(auxGripperClient->getState().isDone()) && ros::ok())
    {
      ros::Duration(0.1).sleep(); // sleep for 1 seconds
    }
    ROS_INFO("[BasicDemoAsr] Gripper set to position: (%f, %f)", positions[0], positions[1]);
  }

  bool BasicDemoAsr::goUp(moveit::planning_interface::MoveGroupInterface *groupArmTorsoPtr, float upDistance)
  {
    groupArmTorsoPtr->setMaxVelocityScalingFactor(0.1);
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
        ROS_INFO("[BasicDemoAsr] Success in moving the grasped object up.");
        return true;
      }
      else
      {
        ROS_INFO("[BasicDemoAsr] Error in moving the grasped object up.");
        return false;
      }
    }
    else
    {
      ROS_INFO("[BasicDemoAsr] No feasible up pose!");
      return false;
    }
  }

  bool BasicDemoAsr::bringCloser(moveit::planning_interface::MoveGroupInterface *groupArmTorsoPtr, float bringCloserDistance)
  {

    /////////////////////////
    groupArmTorsoPtr->setMaxVelocityScalingFactor(0.1);

    geometry_msgs::PoseStamped currentPose = groupArmTorsoPtr->getCurrentPose();
    KDL::Frame frameEndWrtBase;
    tf::poseMsgToKDL(currentPose.pose, frameEndWrtBase);
    KDL::Frame frameToolWrtEnd;
    frameToolWrtEnd.p[0] = -bringCloserDistance;
    KDL::Frame frameToolWrtBase = frameEndWrtBase * frameToolWrtEnd;

    geometry_msgs::Pose toolPose;
    tf::poseKDLToMsg(frameToolWrtBase, toolPose);

    groupArmTorsoPtr->setPoseTarget(toolPose);

    moveit::planning_interface::MoveItErrorCode code = groupArmTorsoPtr->plan(plan_);
    bool successPlanning = (code == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (successPlanning)
    {
      moveit::planning_interface::MoveItErrorCode e = groupArmTorsoPtr->move();
      if (e == moveit::planning_interface::MoveItErrorCode::SUCCESS)
      {
        ROS_INFO("[BasicDemoAsr] Success in moving the grasped object up.");
        return true;
      }
      else
      {
        ROS_INFO("[BasicDemoAsr] Error in moving the grasped object up.");
        return false;
      }
    }
    else
    {
      ROS_INFO("[BasicDemoAsr] No feasible up pose!");
      return false;
    }
  }

  bool BasicDemoAsr::goToGraspingPose(const geometry_msgs::Pose &graspingPose)
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
    const double jump_threshold = 0.00;
    const double eef_step = 0.001;
    double fraction = groupAuxArmTorsoPtr_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO("BasicDemoAsr] plan (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

    moveit::planning_interface::MoveGroupInterface::Plan planAproach;

    planAproach.trajectory_ = trajectory;

    sleep(1.0);

    moveit::planning_interface::MoveItErrorCode e = groupAuxArmTorsoPtr_->execute(planAproach);

    return true;
  }

  void BasicDemoAsr::removeCollisionObjectsPlanningScene()
  {
    ROS_INFO("[BasicDemoAsr] Removing objects in the planningScene");
    std::vector<std::string> objectIds = planningSceneInterface_.getKnownObjectNames();
    planningSceneInterface_.removeCollisionObjects(objectIds);
    ros::Duration(1.0).sleep(); // sleep for 2 seconds
  }

  void BasicDemoAsr::addTablePlanningScene(const std::vector<float> &dimensions, const geometry_msgs::Pose &tablePose, const std::string &id)
  {
    ROS_INFO("[BasicDemoAsr] Add table collision objects to the planning scene");
    // Collision object
    moveit_msgs::CollisionObject collisionObject;
    collisionObject.id = id;
    collisionObject.header.frame_id = "base_footprint";
    ROS_INFO("[BasicDemoAsr] Planning_frame: %s", "base_footprint");
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

  void BasicDemoAsr::addSupequadricsPlanningScene()
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

          ROS_INFO("BasicDemoAsr] CYLINDER height: %f radius: %f", height, radius);
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

  void BasicDemoAsr::initializeHeadPosition(const std::vector<float> &initHeadPositions)
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

  void BasicDemoAsr::initializeTorsoPosition(float initTorsoPosition, float execution_time)
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

    waypointTorsoGoal(torsoGoal, initTorsoPosition, execution_time);

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

  bool BasicDemoAsr::initializeRightArmPosition(const std::vector<double> &initRightArmPositions)
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
        ROS_INFO("[BasicDemoAsr] Right Arm success in moving to the initial joints position.");
        return true;
      }
      else
      {
        ROS_INFO("[BasicDemoAsr] Right Arm Error in moving  to the initial joints position.");
        return false;
      }
    }
    else
    {
      return false;
    }
  }

  bool BasicDemoAsr::initializeLeftArmPosition(const std::vector<double> &initLeftArmPositions)
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
        ROS_INFO("[BasicDemoAsr] Left Arm success in moving to the initial joints position.");
        return true;
      }
      else
      {
        ROS_INFO("[BasicDemoAsr] Left Arm Error in moving  to the initial joints position.");
        return false;
      }
    }
    else
    {
      return false;
    }
  }

  void BasicDemoAsr::init()
  {
    ROS_INFO("[BasicDemoAsr] init().");

    ROS_INFO("[BasicDemoAsr] creating clients...");
    clientActivateSuperquadricsComputation_ = nodeHandle_.serviceClient<companion_msgs::ActivateSupercuadricsComputation>("/grasp_objects/activate_superquadrics_computation");
    clientActivateDetectHit_ = nodeHandle_.serviceClient<companion_msgs::ActivateSupercuadricsComputation>("/detect_hit_ft/activate");

    clientGetSuperquadrics_ = nodeHandle_.serviceClient<companion_msgs::GetSuperquadrics>("/grasp_objects/get_superquadrics");
    clientComputeGraspPoses_ = nodeHandle_.serviceClient<companion_msgs::ComputeGraspPoses>("/grasp_objects/compute_grasp_poses");
    clientGetBboxesSuperquadrics_ = nodeHandle_.serviceClient<companion_msgs::GetBboxes>("/grasp_objects/get_bboxes_superquadrics");
    asrSubscriber_ = nodeHandle_.subscribe("/asr_node/data", 10, &BasicDemoAsr::asrCallback, this);
    amclPoseSubscriber_ = nodeHandle_.subscribe("/amcl_pose", 10, &BasicDemoAsr::amclPoseCallback, this);
    stopDemoSubscriber_ = nodeHandle_.subscribe("/demo/stop", 2, &BasicDemoAsr::stopDemoCallback, this);
    okDemoSubscriber_ = nodeHandle_.subscribe("/demo/ok", 2, &BasicDemoAsr::okDemoCallback, this);
    serviceReleaseGripper_ = nodeHandle_.advertiseService("/demo/release_gripper", &BasicDemoAsr::releaseGripper, this);

    serviceInitDemo_ = nodeHandle_.advertiseService("/demo/init_demo", &BasicDemoAsr::initDemo, this);

    serviceMoveToHomePosition_ = nodeHandle_.advertiseService("/demo/move_to_home_position", &BasicDemoAsr::moveToHomePosition, this);
    statePublisher_ = nodeHandle_.advertise<std_msgs::String>("/demo/state", 10);
    superquadricsBBoxesPublisher_ = nodeHandle_.advertise<companion_msgs::BoundingBoxes>("/demo/superquadrics_bboxes", 10);
    reachingPosePublisher_ = nodeHandle_.advertise<geometry_msgs::PoseStamped>("/demo/reaching_pose", 10);
    planPublisher_ = nodeHandle_.advertise<moveit_msgs::RobotTrajectory>("/demo/trajectory", 10);
    cmdVelPublisher_ = nodeHandle_.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 10);

    clientActivateAsr_ = nodeHandle_.serviceClient<companion_msgs::ActivateASR>("/asr_node/activate_asr");
    acPtr_ = new actionlib::SimpleActionClient<pal_interaction_msgs::TtsAction>("tts", true);

    ROS_INFO("Waiting for /tts action server to start.");
    // acPtr_->waitForServer();
    ROS_INFO("/tts action server started.");

    ros::param::get("demo/use_asr", useAsr_);
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
    ros::param::get("demo/user_id", userId_);
    ROS_INFO("USER_ID: %d", userId_);
    std::vector<float> tmp;
    ros::param::get("demo/left_gripper_position_difference_wrt_default", tmp);
    closeLeftGripperDeviation_[0] = tmp[0];
    closeLeftGripperDeviation_[1] = tmp[1];

    ros::param::get("demo/right_gripper_position_difference_wrt_default", tmp);
    closeRightGripperDeviation_[0] = tmp[0];
    closeRightGripperDeviation_[1] = tmp[1];

    ROS_INFO("[BasicDemoAsr] demo/reaching_distance set to %f", reachingDistance_);

    groupRightArmTorsoPtr_ = new moveit::planning_interface::MoveGroupInterface(nameTorsoRightArmGroup_);
    ROS_INFO("[BasicDemoAsr] Move group interface %s", nameTorsoRightArmGroup_.c_str());

    groupRightArmPtr_ = new moveit::planning_interface::MoveGroupInterface(nameRightArmGroup_);
    ROS_INFO("[BasicDemoAsr] Move group interface %s", nameRightArmGroup_.c_str());

    groupLeftArmTorsoPtr_ = new moveit::planning_interface::MoveGroupInterface(nameTorsoLeftArmGroup_);
    ROS_INFO("[BasicDemoAsr] Move group interface %s", nameTorsoLeftArmGroup_.c_str());

    groupLeftArmPtr_ = new moveit::planning_interface::MoveGroupInterface(nameLeftArmGroup_);
    ROS_INFO("[BasicDemoAsr] Move group interface %s", nameLeftArmGroup_.c_str());

    groupRightArmTorsoPtr_->setPlanningTime(1.5);
    groupRightArmTorsoPtr_->setPlannerId("SBLkConfigDefault");
    groupRightArmTorsoPtr_->setPoseReferenceFrame("base_footprint");
    groupRightArmTorsoPtr_->setMaxVelocityScalingFactor(0.4);

    groupRightArmPtr_->setPlanningTime(1.5);
    groupRightArmPtr_->setPlannerId("SBLkConfigDefault");
    groupRightArmPtr_->setPoseReferenceFrame("base_footprint");
    groupRightArmPtr_->setMaxVelocityScalingFactor(0.4);

    groupLeftArmTorsoPtr_->setPlanningTime(1.5);
    groupLeftArmTorsoPtr_->setPlannerId("SBLkConfigDefault");
    groupLeftArmTorsoPtr_->setPoseReferenceFrame("base_footprint");
    groupLeftArmTorsoPtr_->setMaxVelocityScalingFactor(0.4);

    groupLeftArmPtr_->setPlanningTime(1.5);
    groupLeftArmPtr_->setPlannerId("SBLkConfigDefault");
    groupLeftArmPtr_->setPoseReferenceFrame("base_footprint");
    groupLeftArmPtr_->setMaxVelocityScalingFactor(0.4);

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

    std::string path = ros::package::getPath("demo_anticipatory_vs_reactive");

    std::string user_id_str = "user_" + std::to_string(userId_);
    ROS_INFO_STREAM(user_id_str);
    const char *path_dir = (path + "/csv/" + user_id_str).c_str();
    boost::filesystem::path dir(path_dir);

    if (std::experimental::filesystem::create_directory(path + "/csv/" + user_id_str))
      ;
    ROS_INFO("DIR_CREATED %s", (path + "/csv/" + user_id_str).c_str());
    // if(!fs::is_directory(path+"/csv/"+user_id_str) || !fs::exists(path+"/csv/"+user_id_str))
    //     fs::create_directory(path+"/csv/"+user_id_str); // create user folder

    if (boost::filesystem::exists(date + ".csv"))
      ROS_INFO("The path is valid!");
    else
    {
      timesFile_.open(path + "/csv/" + user_id_str + "/" + date + ".csv");
    }
    return;
  }

  bool BasicDemoAsr::releaseGripper(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
  {
    releaseGripper_ = true;
    return true;
  }

  bool BasicDemoAsr::initDemo(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
  {
    initDemo_ = true;
    return true;
  }

  bool BasicDemoAsr::moveToHomePosition(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
  {
    moveToHomePosition_ = true;
    return true;
  }

  bool BasicDemoAsr::getSuperquadrics()
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

  bool BasicDemoAsr::getBoundingBoxesFromSupercuadrics()
  {

    companion_msgs::GetBboxes srvBBox;
    ROS_INFO("[BasicDemoAsr] Get bounding boxes from superquadrics...");
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

  void BasicDemoAsr::activateSuperquadricsComputation(bool activate)
  {
    companion_msgs::ActivateSupercuadricsComputation srvActivate;
    srvActivate.request.activate = activate;

    if (clientActivateSuperquadricsComputation_.call(srvActivate))
    {
      ROS_INFO("[BasicDemoAsr] ActivateSuperquadricsComputation: %d", (bool)srvActivate.request.activate);
    }
  }

  void BasicDemoAsr::activateDetectHitFt(bool activate)
  {
    companion_msgs::ActivateSupercuadricsComputation srvActivate;
    srvActivate.request.activate = activate;

    if (clientActivateDetectHit_.call(srvActivate))
    {
      ROS_INFO("[BasicDemoAsr] ActivateDetectHit: %d", (bool)srvActivate.request.activate);
    }
  }

  // Create a ROS action client to move TIAGo's head
  void BasicDemoAsr::createClient(follow_joint_control_client_Ptr &actionClient, std::string name)
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
  void BasicDemoAsr::waypointHeadGoal(control_msgs::FollowJointTrajectoryGoal &goal, const std::vector<float> &positions, const float &timeToReach)
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
  void BasicDemoAsr::waypointTorsoGoal(control_msgs::FollowJointTrajectoryGoal &goal, const float &position, const float &timeToReach)
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

  void BasicDemoAsr::waypointGripperGoal(std::string name, control_msgs::FollowJointTrajectoryGoal &goal, const float positions[2], const float &timeToReach)
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

  void BasicDemoAsr::asrCallback(const std_msgs::StringConstPtr &asrMsg)
  {
    if (waitingForAsrCommand_ && useAsr_)
    {
      asr_ = asrMsg->data;
      ROS_INFO("%s", asr_.c_str());
      // Además que el objeto sea uno de los detectados
      mtxASR_.lock();
      foundAsr_ = false;

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

      mtxASR_.unlock();
    }
  }
  void BasicDemoAsr::amclPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &amclPoseMsg)
  {
    basePose_ = amclPoseMsg->pose.pose;
  }

  void *BasicDemoAsr::sendcomputeGraspPosesThreadWrapper(void *object)
  {
    reinterpret_cast<BasicDemoAsr *>(object)->computeGraspPosesThread(NULL);
    return 0;
  }

  void *BasicDemoAsr::computeGraspPosesThread(void *ptr)
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
      ROS_INFO("[BasicDemoAsr] ComputeGraspPoses: %d", (bool)srvGraspingPoses.response.success);
      graspingPoses_ = srvGraspingPoses.response.poses;
      width_ = srvGraspingPoses.response.width;
    }
    ROS_INFO("[BasicDemoAsr] NumberPoses: %d", (int)graspingPoses_.poses.size());

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

  void *BasicDemoAsr::sendFindReachGraspIKThreadWrapper(void *object)
  {
    reinterpret_cast<BasicDemoAsr *>(object)->findReachGraspIKThread(NULL);
    return 0;
  }

  void *BasicDemoAsr::findReachGraspIKThread(void *ptr)
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
      {
        foundReachIk_ = kinematic_state->setFromIK(jointModelGroupTorsoRightArm_, reachingPose_, 0.01);
        if (reachingPose_.position.z <= 0.8) // TODO: USE A PARAMETER
        {

          const Eigen::Affine3d &elbow_state = kinematic_state->getGlobalLinkTransform("arm_right_4_link");
          if (elbow_state.translation().z() < 0.72)
          {
            foundReachIk_ = false;
          }
          else
          {
            foundReachIk_ = true;
          }
        }
      }
      else
      {
        foundReachIk_ = kinematic_state->setFromIK(jointModelGroupTorsoLeftArm_, reachingPose_, 0.01);
        ROS_INFO("Reaching Pose: %f", reachingPose_.position.z);

        if (reachingPose_.position.z <= 0.8) // TODO: USE A PARAMETER
        {

          const Eigen::Affine3d &elbow_state = kinematic_state->getGlobalLinkTransform("arm_left_4_link");
          if (elbow_state.translation().z() < 0.72)
          {
            foundReachIk_ = false;
          }
          else
          {
            foundReachIk_ = true;
          }
        }
      }
      //     geometry_msgs::PoseStamped goal_pose;
      // goal_pose.header.frame_id = "base_footprint";
      // goal_pose.pose = graspingPoses.poses[idx];
      if (foundReachIk_)
      {
        ROS_INFO("[BasicDemoAsr] IK Found!");
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

  void *BasicDemoAsr::sendPlanToReachJointsThreadWrapper(void *object)
  {
    reinterpret_cast<BasicDemoAsr *>(object)->planToReachJointsThread(NULL);
    return 0;
  }

  void *BasicDemoAsr::planToReachJointsThread(void *ptr)
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

  void BasicDemoAsr::stopDemoCallback(const std_msgs::EmptyConstPtr &stop)
  {
    ROS_INFO("Demo stopped!");
    stopDemo_ = true;
  }

  void BasicDemoAsr::okDemoCallback(const std_msgs::EmptyConstPtr &msg)
  {
    ROS_INFO("Demo stopped!");
    okDemo_ = true;
  }

  bool BasicDemoAsr::computeIntersectionOverUnion(const std::array<int, 4> &bboxYolo, const std::array<int, 4> &bboxSq, float &IoU)
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

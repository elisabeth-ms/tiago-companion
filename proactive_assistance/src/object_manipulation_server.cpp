#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <proactive_assistance/ObjectManipulationAction.h> // Include your action file here
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf_conversions/tf_kdl.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <moveit_msgs/OrientationConstraint.h>
#include <moveit_msgs/Constraints.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> follow_joint_control_client;
typedef boost::shared_ptr<follow_joint_control_client> follow_joint_control_client_Ptr;

class ObjectManipulationAction
{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<proactive_assistance::ObjectManipulationAction> as_;
  std::string action_name_;

  // create messages that are used to published feedback/result
  proactive_assistance::ObjectManipulationFeedback feedback_;
  proactive_assistance::ObjectManipulationResult result_;

  std::string nameTorsoRightArmGroup_ = "arm_right_torso";
  std::string nameRightArmGroup_ = "arm_right";
  std::string nameTorsoLeftArmGroup_ = "arm_left_torso";
  std::string nameLeftArmGroup_ = "arm_left";
  moveit::planning_interface::MoveGroupInterface *groupRightArmTorsoPtr_;
  moveit::planning_interface::MoveGroupInterface *groupRightArmPtr_;
  moveit::planning_interface::MoveGroupInterface *groupLeftArmTorsoPtr_;
  moveit::planning_interface::MoveGroupInterface *groupLeftArmPtr_;

  moveit::planning_interface::PlanningSceneInterface planningSceneInterface_;

  robot_model::RobotModelPtr kinematicModel_;
  robot_state::RobotStatePtr kinematicState_;
  const robot_state::JointModelGroup *jointModelGroupTorsoRightArm_;
  const robot_state::JointModelGroup *jointModelGroupTorsoLeftArm_;
  float reachingDistance_ = 0.12;    // TODO: Define the reaching distance as a rosparam
  float gripperLinkDistance_ = 0.18; // TODO: Define the distance between the gripper link and the tool link as a rosparam

  follow_joint_control_client_Ptr rightGripperClient_;
  follow_joint_control_client_Ptr leftGripperClient_;

  float openGripperPositions_[2] = {0.07, 0.07};
  float closeGripperPositions_[2] = {0.0, 0.0};
  float closeGripperDeviation_ = 0.033;
  geometry_msgs::Pose comfortablePoseRight_;
  geometry_msgs::Pose comfortablePoseLeft_;

  ros::Publisher target_pose_pub_; 
  



public:
  ObjectManipulationAction(std::string name) : as_(nh_, name, boost::bind(&ObjectManipulationAction::executeCB, this, _1), false),
                                               action_name_(name)
  {

    robot_model_loader::RobotModelLoader robotModelLoader("robot_description");
    kinematicModel_ = robotModelLoader.getModel();

    jointModelGroupTorsoRightArm_ = kinematicModel_->getJointModelGroup(nameTorsoRightArmGroup_);
    jointModelGroupTorsoLeftArm_ = kinematicModel_->getJointModelGroup(nameTorsoLeftArmGroup_);

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

    createClient(rightGripperClient_, std::string("gripper_right"));
    createClient(leftGripperClient_, std::string("gripper_left"));

    comfortablePoseRight_.position.x = 0.527;
    comfortablePoseRight_.position.y = -0.325;
    comfortablePoseRight_.position.z = 0.960;
    comfortablePoseRight_.orientation.x = 0.018;
    comfortablePoseRight_.orientation.y = 0.706;
    comfortablePoseRight_.orientation.z = -0.696;
    comfortablePoseRight_.orientation.w = -0.131;

    target_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("target_poses", 10); // Add this line
    as_.start();
  }

  ~ObjectManipulationAction(void)
  {
  }

  void executeCB(const proactive_assistance::ObjectManipulationGoalConstPtr &goal)
  {
    // Start processing the goal
    // Check for objects, obstacles, and calculate grasping poses
    // Publish feedback and result
    ROS_INFO("%s: Processing the manipulation task", action_name_.c_str());
    // Remove all objects in the planning scene
    removeCollisionObjectsPlanningScene();
    ROS_INFO("Removed all objects in the planning scene");
    addObstaclesToPlanningScene(goal->obstacles, goal->obstacle_poses, goal->reference_frames_of_obstacles);
    bool success = true; // example processing result

    if (goal->task == "pick")
    {
      ROS_INFO("Processing the manipulation task");
      ROS_INFO("Number of obstacles: %d", goal->obstacles.size());
      ROS_INFO("Number of obstacle poses: %d", goal->obstacle_poses.size());
      ROS_INFO("Number of reference frames of obstacles: %d", goal->reference_frames_of_obstacles.size());
      ROS_INFO("Number of grasping poses: %d", goal->grasping_poses.poses.size());

      float closeGripperPositions[2] = {0.0, 0.0};
      moveGripper(closeGripperPositions, "right");
      // Lets plan the motion of the robot to the grasping poses
      std::vector<geometry_msgs::Pose> pregrasp_poses;
      getPregrasp(goal->grasping_poses, pregrasp_poses, reachingDistance_);

      int index_grasp = 0;
      for (index_grasp = 0; index_grasp < pregrasp_poses.size(); index_grasp++)
      {
        ROS_INFO("Moving to pregrasp pose %d", index_grasp);
        geometry_msgs::Pose pose = pregrasp_poses[index_grasp];
        groupRightArmTorsoPtr_->setPoseTarget(pose);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        moveit::planning_interface::MoveItErrorCode code = groupRightArmTorsoPtr_->plan(my_plan);
        bool successPlanning = (code == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        bool successExecution = false;
        if (successPlanning)
        {
          moveit::planning_interface::MoveItErrorCode codeExecute = groupRightArmTorsoPtr_->execute(my_plan);
          successExecution = (codeExecute == moveit::planning_interface::MoveItErrorCode::SUCCESS);
          if (successExecution)
          {
            ROS_INFO("Successfully moved to the grasping pose %d", index_grasp);
          }
          break;
        }
        else
        {
          ROS_INFO("Failed to plan to the grasping pose %d", index_grasp);
        }
      }

      if (index_grasp == pregrasp_poses.size())
      {
        ROS_INFO("Failed to move to any of the grasping poses");
        success = false;
      }

      if (index_grasp < pregrasp_poses.size())
      {
        moveGripper(openGripperPositions_, "right");
        success = goToGraspingPose(goal->grasping_poses.poses[index_grasp]);
        float close[2] = {goal->width[index_grasp] / 2.0-closeGripperDeviation_, goal->width[index_grasp] / 2.0-closeGripperDeviation_};
        moveGripper(close, "right");
      }
    }
    else if (goal->task == "comfortable")
    {
      ROS_INFO("Moving to comfortable pose");
      moveToConfortablePose(comfortablePoseRight_);
    }
    else if (goal->task == "place")
    {
      ROS_INFO("Placing the object");
      placeObject(goal->zone_place);
      moveGripper(openGripperPositions_, "right");
      goToDistancedPose(0.2);
    }
    else
    {
      ROS_INFO("Unknown task");
      success = false;
    }

    // Check that preempt has not been requested by the client
    if (as_.isPreemptRequested() || !ros::ok())
    {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      // Set the action state to preempted
      as_.setPreempted();
      success = false;
    }

    if (success)
    {
      result_.success = true;
      result_.message = "Successfully processed the manipulation task";
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
    else
    {
      result_.success = false;
      result_.message = "Failed to process the manipulation task";
      ROS_INFO("%s: Failed", action_name_.c_str());
      // set the action state to succeeded
      as_.setAborted(result_);
    }
  }
  // Create a ROS action client to move TIAGo's head
  void
  createClient(follow_joint_control_client_Ptr &actionClient, std::string name)
  {
    ROS_INFO("[ObjectManipulation] Creating action client to %s controller ...", name.c_str());

    std::string controller_name = name + "_controller/follow_joint_trajectory";

    actionClient.reset(new follow_joint_control_client(controller_name));

    int iterations = 0, max_iterations = 3;
    // Wait for arm controller action server to come up
    while (!actionClient->waitForServer(ros::Duration(0.5)) && ros::ok() && iterations < max_iterations)
    {
      ROS_DEBUG("[ObjectManipulation] Waiting for the %s_controller_action server to come up", name.c_str());
      ++iterations;
    }

    if (iterations == max_iterations)
      ROS_ERROR("[ObjectManipulation] createClient: %s controller action server not available", name.c_str());
  }

  void removeCollisionObjectsPlanningScene()
  {
    ROS_INFO("[ObjectManipulation] Removing objects in the planningScene");
    std::vector<std::string> objectIds = planningSceneInterface_.getKnownObjectNames();
    planningSceneInterface_.removeCollisionObjects(objectIds);
  }

  void addObstaclesToPlanningScene(const std::vector<shape_msgs::SolidPrimitive> &obstacles, const std::vector<geometry_msgs::Pose> &obstacle_poses,
                                   const std::vector<std::string> &reference_frames_of_obstacles)
  {
    for (int i = 0; i < obstacles.size(); i++)
    {
      ROS_INFO("Adding Collision object %d", i);
      moveit_msgs::CollisionObject collisionObject;
      collisionObject.id = i;
      collisionObject.header.frame_id = reference_frames_of_obstacles[i];
      collisionObject.operation = moveit_msgs::CollisionObject::ADD;
      collisionObject.primitives.push_back(obstacles[i]);
      collisionObject.primitive_poses.push_back(obstacle_poses[i]);
      collisionObject.header.stamp = ros::Time::now();

      planningSceneInterface_.applyCollisionObject(collisionObject);
    }
  }

  void getPregrasp(const geometry_msgs::PoseArray &grasping_poses, std::vector<geometry_msgs::Pose> &pregrasp_poses, double pregrasp_distance)
  {
    pregrasp_poses.clear();

    for (const auto &pose : grasping_poses.poses)
    {
      // Convert geometry_msgs::Pose to tf2::Transform
      tf2::Transform transform;
      tf2::fromMsg(pose, transform);

      // Define the translation along the local negative x-axis
      tf2::Vector3 pregrasp_translation(-pregrasp_distance - gripperLinkDistance_, 0.0, 0.0);

      // Apply the translation to the transform
      transform.setOrigin(transform.getOrigin() + transform.getBasis() * pregrasp_translation);

      // Convert tf2::Transform back to geometry_msgs::Pose
      geometry_msgs::Pose pregrasp_pose;
      tf2::toMsg(transform, pregrasp_pose);

      pregrasp_poses.push_back(pregrasp_pose);
    }
  }

  bool goToGraspingPose(const geometry_msgs::Pose &graspingPose)
  {

    moveit::planning_interface::MoveGroupInterface *groupAuxArmTorsoPtr_;

    groupAuxArmTorsoPtr_ = groupRightArmTorsoPtr_;

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
    ROS_INFO("ObjectManipulationServer] plan (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

    moveit::planning_interface::MoveGroupInterface::Plan planAproach;

    planAproach.trajectory_ = trajectory;

    sleep(1.0);

    moveit::planning_interface::MoveItErrorCode e = groupAuxArmTorsoPtr_->execute(planAproach);

    return true;
  }

  bool goToDistancedPose(double distanced)
  {

    moveit::planning_interface::MoveGroupInterface *groupAuxArmTorsoPtr_;

    groupAuxArmTorsoPtr_ = groupRightArmTorsoPtr_;

    groupAuxArmTorsoPtr_->setMaxVelocityScalingFactor(0.1);
    groupAuxArmTorsoPtr_->setMaxAccelerationScalingFactor(0.1);

    geometry_msgs::PoseStamped currentPose = groupAuxArmTorsoPtr_->getCurrentPose();
    KDL::Frame frameEndWrtBase;
    tf::poseMsgToKDL(currentPose.pose, frameEndWrtBase);
    KDL::Frame frameToolWrtEnd;
    frameToolWrtEnd.p[0] = - distanced;
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
    ROS_INFO("ObjectManipulationServer] plan (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

    moveit::planning_interface::MoveGroupInterface::Plan planAproach;

    planAproach.trajectory_ = trajectory;

    sleep(1.0);

    moveit::planning_interface::MoveItErrorCode e = groupAuxArmTorsoPtr_->execute(planAproach);

    return true;
  }

  void waypointGripperGoal(std::string name, control_msgs::FollowJointTrajectoryGoal &goal, const float positions[2], const float &timeToReach)
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

  void moveGripper(const float positions[2], std::string name)
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
    ROS_INFO("[ObjectManipulationServer] Setting gripper %s position: (%f ,%f)", name.c_str(), positions[0], positions[1]);
    waypointGripperGoal(name, gripperGoal, positions, 0.5);

    // Sends the command to start the given trajectory now
    gripperGoal.trajectory.header.stamp = ros::Time(0);
    auxGripperClient->sendGoal(gripperGoal);

    // Wait for trajectory execution
    while (!(auxGripperClient->getState().isDone()) && ros::ok())
    {
      ros::Duration(0.1).sleep(); // sleep for 1 seconds
    }
    ROS_INFO("[ObjectManipulationServer] Gripper set to position: (%f, %f)", positions[0], positions[1]);
  }

  bool moveToConfortablePose(const geometry_msgs::Pose &pose)
  {


    moveit::planning_interface::MoveGroupInterface *groupAuxArmTorsoPtr_;

    groupAuxArmTorsoPtr_ = groupRightArmTorsoPtr_;

    groupAuxArmTorsoPtr_->setMaxVelocityScalingFactor(0.3);
    groupAuxArmTorsoPtr_->setMaxAccelerationScalingFactor(0.3);

    geometry_msgs::PoseStamped grasp_pose_stamped = groupAuxArmTorsoPtr_->getCurrentPose();
    geometry_msgs::Pose grasp_pose = grasp_pose_stamped.pose;

    // Define comfortable pose with fixed position
    geometry_msgs::Pose comfortable_pose;
    comfortable_pose.position.x = 0.527;
    comfortable_pose.position.y = -0.325;
    comfortable_pose.position.z = 0.960;

    // Set orientation to be the same as the grasp pose to maintain level
    comfortable_pose.orientation = grasp_pose.orientation;

    // Apply orientation constraints
    // moveit_msgs::OrientationConstraint ocm;
    // ocm.link_name = "gripper_right_tool_link";  // Change to the appropriate end effector link
    // ocm.header.frame_id = "base_footprint";  // Change to the appropriate reference frame
    // ocm.orientation = grasp_pose.orientation;
    // ocm.absolute_x_axis_tolerance = 0.01;
    // ocm.absolute_y_axis_tolerance = 0.01;
    // ocm.absolute_z_axis_tolerance = 0.8;
    // ocm.weight = 1.0;

    // moveit_msgs::Constraints constraints;
    // constraints.orientation_constraints.push_back(ocm);
    // groupAuxArmTorsoPtr_->setPathConstraints(constraints);

        // Set the target pose
    groupAuxArmTorsoPtr_->setPoseTarget(comfortable_pose);

    // Plan and execute the motion
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (groupAuxArmTorsoPtr_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success) {
        groupAuxArmTorsoPtr_->move();
        return true;
    } else {
        ROS_WARN("Planning to the comfortable pose failed.");
        return false;
    }

  }
bool placeObject(const std::vector<geometry_msgs::Point>& bounding_zone)
{

    //lETS PRINT THE BOUNDING ZONE
    for (int i = 0; i < bounding_zone.size(); i++)
    {
      ROS_INFO("Bounding zone %d: x=%f, y=%f, z=%f", i, bounding_zone[i].x, bounding_zone[i].y, bounding_zone[i].z);
    }
    moveit::planning_interface::MoveGroupInterface* groupAuxArmTorsoPtr_;
    groupAuxArmTorsoPtr_ = groupRightArmPtr_;

    groupAuxArmTorsoPtr_->setMaxVelocityScalingFactor(0.3);
    groupAuxArmTorsoPtr_->setMaxAccelerationScalingFactor(0.3);

    geometry_msgs::PoseStamped grasp_pose_stamped = groupAuxArmTorsoPtr_->getCurrentPose();
    geometry_msgs::Pose grasp_pose = grasp_pose_stamped.pose;

    // Calculate the center of the bounding zone
    geometry_msgs::Point center;
    center.x = (bounding_zone[0].x + bounding_zone[1].x + bounding_zone[2].x + bounding_zone[3].x) / 4.0;
    center.y = (bounding_zone[0].y + bounding_zone[1].y + bounding_zone[2].y + bounding_zone[3].y) / 4.0;
    center.z = (bounding_zone[0].z + bounding_zone[1].z + bounding_zone[2].z + bounding_zone[3].z) / 4.0;

    // Define a function to interpolate points within the bounding zone
    auto interpolate = [&](double alpha, double beta) {
        geometry_msgs::Pose target_pose;
        target_pose.position.x = (1 - alpha) * ((1 - beta) * bounding_zone[0].x + beta * bounding_zone[1].x) + alpha * ((1 - beta) * bounding_zone[2].x + beta * bounding_zone[3].x);
        target_pose.position.y = (1 - alpha) * ((1 - beta) * bounding_zone[0].y + beta * bounding_zone[1].y) + alpha * ((1 - beta) * bounding_zone[2].y + beta * bounding_zone[3].y);
        target_pose.position.z = (1 - alpha) * ((1 - beta) * bounding_zone[0].z + beta * bounding_zone[1].z) + alpha * ((1 - beta) * bounding_zone[2].z + beta * bounding_zone[3].z);
        target_pose.orientation = grasp_pose.orientation; // Maintain the same orientation
        return target_pose;
    };

    bool success = false;
    // Test points within the bounding zone starting from the center
    for (double step = 0.0; step <= 1.0; step += 0.1) {
        for (double alpha = 0.5 - step; alpha <= 0.5 + step; alpha += step) {
            for (double beta = 0.5 - step; beta <= 0.5 + step; beta += step) {
                if (alpha < 0 || alpha > 1 || beta < 0 || beta > 1) continue;
                geometry_msgs::Pose target_pose = interpolate(alpha, beta);

                groupAuxArmTorsoPtr_->setPoseTarget(target_pose);

                geometry_msgs::PoseStamped target_pose_stamped;
                target_pose_stamped.header.frame_id = "base_footprint"; // Change to your reference frame
                target_pose_stamped.header.stamp = ros::Time::now();
                target_pose_stamped.pose = target_pose;
                target_pose_pub_.publish(target_pose_stamped);

                moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                success = (groupAuxArmTorsoPtr_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

                if (success) {
                    groupAuxArmTorsoPtr_->move();
                    return true; // Exit the function after the first successful motion
                }
            }
        }
    }


    ROS_WARN("Planning to the bounding zone failed.");
    return false;




}

  
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_manipulation_server");
  ObjectManipulationAction object_manipulation("object_manipulation");
  ros::spin();
  return 0;
}
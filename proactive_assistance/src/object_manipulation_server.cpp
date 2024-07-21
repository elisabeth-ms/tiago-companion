#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <proactive_assistance/ObjectManipulationAction.h> // Include your action file here
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/MoveGroupAction.h>
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
    addObstaclesToPlanningScene(goal->obstacles, goal->obstacle_poses, goal->reference_frames_of_obstacles);
    bool success = true; // example processing result

    ROS_INFO("Processing the manipulation task");
    ROS_INFO("Number of obstacles: %d", goal->obstacles.size());
    ROS_INFO("Number of obstacle poses: %d", goal->obstacle_poses.size());
    ROS_INFO("Number of reference frames of obstacles: %d", goal->reference_frames_of_obstacles.size());
    ROS_INFO("Number of grasping poses: %d", goal->grasping_poses.poses.size());

    // Lets plan the motion of the robot to the grasping poses
    for (int i = 0; i < goal->grasping_poses.poses.size(); i++)
    {
      ROS_INFO("Moving to grasping pose %d", i);
      geometry_msgs::Pose pose = goal->grasping_poses.poses[i];
      groupRightArmTorsoPtr_->setPoseTarget(pose);
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;

      moveit::planning_interface::MoveItErrorCode code = groupRightArmTorsoPtr_->plan(my_plan);
      bool successPlanning = (code == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      if (successPlanning)
      {
        groupRightArmTorsoPtr_->execute(my_plan);
        break;
      }
      else
      {
        ROS_INFO("Failed to plan to the grasping pose %d", i);
      }
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
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_manipulation_server");
  ObjectManipulationAction object_manipulation("object_manipulation");
  ros::spin();
  return 0;
}

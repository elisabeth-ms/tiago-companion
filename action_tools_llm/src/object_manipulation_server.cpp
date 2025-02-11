#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <action_tools_llm/ObjectManipulationAction.h> // Include your action file here
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
#include <moveit_msgs/AllowedCollisionMatrix.h>
#include <moveit_msgs/AllowedCollisionEntry.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <geometry_msgs/PoseArray.h>

#define GRIPPER_MAX_WIDTH 0.08
#include <math.h>
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> follow_joint_control_client;
typedef boost::shared_ptr<follow_joint_control_client> follow_joint_control_client_Ptr;

class ObjectManipulationAction
{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<action_tools_llm::ObjectManipulationAction> as_;
  std::string action_name_;

  // create messages that are used to published feedback/result
  action_tools_llm::ObjectManipulationFeedback feedback_;
  action_tools_llm::ObjectManipulationResult result_;

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
  float reachingDistance_ = 0.1;    // TODO: Define the reaching distance as a rosparam
  float gripperLinkDistance_ = 0.19; // TODO: Define the distance between the gripper link and the tool link as a rosparam

  follow_joint_control_client_Ptr rightGripperClient_;
  follow_joint_control_client_Ptr leftGripperClient_;

  float openGripperPositions_[2] = {0.07, 0.07};
  float closeGripperPositions_[2] = {0.0, 0.0};
  float closeGripperDeviation_ = 0.033;
  geometry_msgs::Pose comfortablePoseRight_;
  geometry_msgs::Pose comfortablePoseLeft_;

  ros::Publisher target_pose_pub_; 
  float current_gripper_positions_[2 ] = {0.0, 0.0};
  
  ros::Publisher grasp_pub_;
  ros::Publisher pregrasp_pub_;
  ros::Publisher top_object_pub_;
  ros::Publisher pouring_pose_top_pub_;
  ros::Publisher new_grasping_pose_pub_;
  ros::Publisher trajectory_pub_;
  std::vector<double> initRightArmPositions_ = {-47*M_PI/180.0, 4*M_PI/180.0, 115*M_PI/180.0, 117*M_PI/180.0, -87*M_PI/180.0, 48.0*M_PI/180.0, -48*M_PI/180.0};
  std::vector<double> initLeftArmPositions_ = {-47*M_PI/180.0, 4*M_PI/180.0, 115*M_PI/180.0, 117*M_PI/180.0, -87*M_PI/180.0, 48.0*M_PI/180.0, -48*M_PI/180.0};


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

    groupRightArmTorsoPtr_->setPlanningTime(4.5);
    groupRightArmTorsoPtr_->setPlannerId("RRTConnectConfigDefault");
    groupRightArmTorsoPtr_->setPoseReferenceFrame("aruco_base");
    groupRightArmTorsoPtr_->setMaxVelocityScalingFactor(0.6);
    groupRightArmTorsoPtr_->setGoalPositionTolerance(0.02);   // Allow 2 cm position error
    groupRightArmTorsoPtr_->setGoalOrientationTolerance(0.1); 
    
    groupRightArmPtr_->setPlanningTime(2.5);
    groupRightArmPtr_->setPlannerId("RRTConnectConfigDefault");
    groupRightArmPtr_->setPoseReferenceFrame("aruco_base");
    groupRightArmPtr_->setMaxVelocityScalingFactor(0.4);
    groupRightArmPtr_->setGoalPositionTolerance(0.02);   // Allow 2 cm position error
    groupRightArmPtr_->setGoalOrientationTolerance(0.1); 
    
    groupLeftArmTorsoPtr_->setPlanningTime(1.5);
    groupLeftArmTorsoPtr_->setPlannerId("SBLkConfigDefault");
    groupLeftArmTorsoPtr_->setPoseReferenceFrame("aruco_base");
    groupLeftArmTorsoPtr_->setMaxVelocityScalingFactor(0.4);

    groupLeftArmPtr_->setPlanningTime(2.5);
    groupLeftArmPtr_->setPlannerId("SBLkConfigDefault");
    groupLeftArmPtr_->setPoseReferenceFrame("aruco_base");
    groupLeftArmPtr_->setMaxVelocityScalingFactor(0.4);

    createClient(rightGripperClient_, std::string("gripper_right"));
    createClient(leftGripperClient_, std::string("gripper_left"));

    comfortablePoseRight_.position.x = 0.527;
    comfortablePoseRight_.position.y = -0.325;
    comfortablePoseRight_.position.z = 1.20;
    comfortablePoseRight_.orientation.x = 0.018;
    comfortablePoseRight_.orientation.y = 0.706;
    comfortablePoseRight_.orientation.z = -0.696;
    comfortablePoseRight_.orientation.w = -0.131;

    target_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("target_poses", 10); // Add this line

    // SUbscriber to joint_states
    ros::Subscriber sub = nh_.subscribe("/joint_states", 1000, &ObjectManipulationAction::jointStatesCallback, this);

    grasp_pub_ = nh_.advertise<geometry_msgs::PoseArray>("grasp_poses", 10);
    pregrasp_pub_ = nh_.advertise<geometry_msgs::PoseArray>("pregrasp_poses", 10);
    top_object_pub_ = nh_.advertise<geometry_msgs::PoseArray>("top_object_poses", 10);
    pouring_pose_top_pub_ =  nh_.advertise<geometry_msgs::PoseStamped>("pouring_pose_top", 10);
    new_grasping_pose_pub_= nh_.advertise<geometry_msgs::PoseStamped>("new_grasping_pose", 10);
    trajectory_pub_ = nh_.advertise<geometry_msgs::PoseArray>("trajectory_pouring", 10);


    as_.start();
    ROS_INFO("[ObjectManipulation] Action server started");


  }

  ~ObjectManipulationAction(void)
  {
  }

// Define a function to interpolate points within the bounding zone
geometry_msgs::Pose interpolate(double alpha, double beta, geometry_msgs::Quaternion orientation, std::vector<geometry_msgs::Point> bounding_zone) {
    geometry_msgs::Pose target_pose;
    target_pose.position.x = (1 - alpha) * ((1 - beta) * bounding_zone[0].x + beta * bounding_zone[1].x) + alpha * ((1 - beta) * bounding_zone[2].x + beta * bounding_zone[3].x);
    target_pose.position.y = (1 - alpha) * ((1 - beta) * bounding_zone[0].y + beta * bounding_zone[1].y) + alpha * ((1 - beta) * bounding_zone[2].y + beta * bounding_zone[3].y);
    target_pose.position.z = (1 - alpha) * ((1 - beta) * bounding_zone[0].z + beta * bounding_zone[1].z) + alpha * ((1 - beta) * bounding_zone[2].z + beta * bounding_zone[3].z);
    target_pose.orientation = orientation; // Maintain the same orientation
    return target_pose;
}

void disableCollisionChecking(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, 
                              const std::string& object_id, 
                              const std::string& link_name)
{
 // Create a service client to get the planning scene
    ros::ServiceClient get_planning_scene_client = nh_.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");
    moveit_msgs::GetPlanningScene srv;
    srv.request.components.components = moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;

    if (!get_planning_scene_client.call(srv)) {
        ROS_ERROR("Failed to call service get_planning_scene");
        return;
    }

    // Modify the AllowedCollisionMatrix
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.is_diff = true;

    moveit_msgs::AllowedCollisionMatrix& acm = srv.response.scene.allowed_collision_matrix;

    // Add the link and object to the ACM if they are not present
    if (std::find(acm.entry_names.begin(), acm.entry_names.end(), link_name) == acm.entry_names.end()) {
        acm.entry_names.push_back(link_name);
    }

    if (std::find(acm.entry_names.begin(), acm.entry_names.end(), object_id) == acm.entry_names.end()) {
        acm.entry_names.push_back(object_id);
    }

    // Ensure the entry_values vector is resized to match entry_names
    size_t new_size = acm.entry_names.size();
    if (acm.entry_values.size() < new_size) {
        acm.entry_values.resize(new_size);
    }

    // Enable collision between the specific link and object
    size_t link_index = std::distance(acm.entry_names.begin(), std::find(acm.entry_names.begin(), acm.entry_names.end(), link_name));
    size_t object_index = std::distance(acm.entry_names.begin(), std::find(acm.entry_names.begin(), acm.entry_names.end(), object_id));

    // Resize the enabled vector for each entry to the correct size
    for (size_t i = 0; i < acm.entry_values.size(); ++i) {
        acm.entry_values[i].enabled.resize(new_size, false);
    }

    // Enable collision between the link and object
    acm.entry_values[link_index].enabled[object_index] = true;
    acm.entry_values[object_index].enabled[link_index] = true;

    // Apply the updated ACM to the planning scene using the PlanningSceneInterface
    planning_scene.allowed_collision_matrix = acm;

    // Apply the changes to the planning scene
    planning_scene_interface.applyPlanningScene(planning_scene);
}
  void executeCB(const action_tools_llm::ObjectManipulationGoalConstPtr &goal)
  {
    // Start processing the goal
    // Check for objects, obstacles, and calculate grasping poses
    // Publish feedback and result
    ROS_INFO("%s: Processing the manipulation task", action_name_.c_str());
    // Remove all objects in the planning scene
    // removeCollisionObjectsPlanningScene();
    // ROS_INFO("Removed all objects in the planning scene");
    // addObstaclesToPlanningScene(goal->obstacles, goal->obstacle_poses, goal->reference_frames_of_obstacles);
    bool success = true; // example processing result

    if (goal->task == "hand_over_to_person")
    {
      bool init_right_arm = initializeArmPosition(groupRightArmPtr_,initRightArmPositions_);
      ROS_INFO("[ObjectManipulation] Right arm initialized: %d", init_right_arm);
      bool init_left_arm = initializeArmPosition(groupLeftArmPtr_ ,initLeftArmPositions_);
      ROS_INFO("[ObjectManipulation] Left arm initialized: %d", init_right_arm);

      ROS_INFO("Processing the hand_over_to_person task");
      ROS_INFO("The object to hand over is: %s", goal->object_for_task_name1.c_str());
      ROS_INFO("The person to hand over to is: %s", goal->object_for_task_name2.c_str());
      std::vector<std::string> reference_frames;
      int index_object = -1;
      for (int i=0; i<goal->object_names.size(); i++)
      {
        ROS_INFO("Object %d: %s", i, goal->object_names[i].c_str());
        ROS_INFO("Pose of object %d: x=%f, y=%f, z=%f", i, goal->object_poses[i].position.x, goal->object_poses[i].position.y, goal->object_poses[i].position.z);
        ROS_INFO("Shape of object %d: type=%d", i, goal->object_shapes[i].type);
        reference_frames.push_back("aruco_base");
        if (goal->object_names[i] == goal->object_for_task_name1)
        {
          index_object = i;              
        }

      }
      ROS_INFO("INDEX OBJECT %d", index_object);
      if (index_object==-1){
        success = false;
      }else{


          geometry_msgs::PoseArray grasping_poses;
          geometry_msgs::PoseArray top_object_poses;
          int index_grasp = 0;
          moveit::planning_interface::MoveGroupInterface *groupPtr;
          if(goal->object_poses[index_object].position.y<0){
              groupPtr = groupRightArmPtr_;
          }
          else{
              groupPtr = groupLeftArmPtr_;
          }

          bool grasp_ok = graspObject(groupPtr, &planningSceneInterface_, goal->object_names, goal->object_poses, goal->object_shapes, 
                          reference_frames, index_object, goal->object_for_task_name1, grasping_poses, top_object_poses, index_grasp);
          if(grasp_ok){
              ROS_INFO("GRASP OK: ", grasp_ok);
              success = aproachGraspedObjectToUser(groupPtr, &planningSceneInterface_, goal->object_for_task_name1, grasping_poses, index_grasp, goal->zone_place);
          }


      }
    }
    else if (goal->task == "pour_into"){
      bool init_right_arm = initializeArmPosition(groupRightArmPtr_,initRightArmPositions_);
      ROS_INFO("[ObjectManipulation] Right arm initialized: %d", init_right_arm);
      bool init_left_arm = initializeArmPosition(groupLeftArmPtr_ ,initLeftArmPositions_);
      ROS_INFO("[ObjectManipulation] Left arm initialized: %d", init_right_arm);

      ROS_INFO("Processing the pour_into task");
      ROS_INFO("The container to pour from is: %s", goal->object_for_task_name1.c_str());
      ROS_INFO("The container to pour into is: %s", goal->object_for_task_name2.c_str());
      std::vector<std::string> reference_frames;
      int index_object_from = -1, index_object_to = -1;
      for (int i=0; i<goal->object_names.size(); i++)
      {
        ROS_INFO("Object %d: %s", i, goal->object_names[i].c_str());
        ROS_INFO("Pose of object %d: x=%f, y=%f, z=%f", i, goal->object_poses[i].position.x, goal->object_poses[i].position.y, goal->object_poses[i].position.z);
        ROS_INFO("Shape of object %d: type=%d", i, goal->object_shapes[i].type);
        reference_frames.push_back("aruco_base");
        if (goal->object_names[i] == goal->object_for_task_name1)
        {
          index_object_from = i;              
        }
        if (goal->object_names[i] == goal->object_for_task_name2){
          index_object_to = i;
        }

      }
      ROS_INFO("INDEX OBJECT FROM %d", index_object_from);
      ROS_INFO("INDEX OBJECT TO %d", index_object_to);
      if (index_object_from == -1 && index_object_to == -1){
          success = false;
      }else{
          geometry_msgs::PoseArray grasping_poses;
          geometry_msgs::PoseArray top_pouring_poses;

          // For each grasping pose we also compute the pouring pose of the top of the object

          int index_grasp = 0;
          moveit::planning_interface::MoveGroupInterface *groupPtr;
          if(goal->object_poses[index_object_from].position.y<0){
              groupPtr = groupRightArmPtr_;
          }
          else{
              groupPtr = groupLeftArmPtr_;
          }

          bool grasp_ok = graspObject(groupPtr, &planningSceneInterface_, goal->object_names, goal->object_poses, goal->object_shapes, 
                          reference_frames, index_object_from, goal->object_for_task_name1, grasping_poses, top_pouring_poses, index_grasp);
          
          // Compute pouring pose
          geometry_msgs::Pose grasping_pose = grasping_poses.poses[index_grasp];
          geometry_msgs::Pose top_pouring_pose = top_pouring_poses.poses[index_grasp];


          bool pour_ok = pourInto(groupPtr, &planningSceneInterface_, goal->object_names, goal->object_poses, goal->object_shapes,
                          reference_frames, index_object_from, goal->object_for_task_name1, index_object_to, goal->object_for_task_name2, grasping_pose, top_pouring_pose);

          if (pour_ok){
              bool init = initializeArmPosition(groupPtr,initRightArmPositions_);
              success = true;

          }
      }
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
      ROS_INFO("success: %d", success);
      result_.success = true;
      result_.message = "Successfully processed the manipulation task";
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
    else
    {
      ROS_INFO("success: %d", success);
      result_.success = false;
      result_.message = "Failed to process the manipulation task";
      ROS_INFO("%s: Failed", action_name_.c_str());
      // set the action state to succeeded
      as_.setAborted(result_);
    }
  }
  // Create a ROS action client to move TIAGo's head
  void createClient(follow_joint_control_client_Ptr &actionClient, std::string name)
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

  void removeAllAttachedObjects()
  {
      moveit::planning_interface::PlanningSceneInterface planningSceneInterface_;
      
      // Get list of currently attached objects
      std::map<std::string, moveit_msgs::AttachedCollisionObject> attached_objects = planningSceneInterface_.getAttachedObjects();

      // Loop through and detach each object
      for (const auto& pair : attached_objects)
      {
          moveit_msgs::AttachedCollisionObject detach_object;
          detach_object.object.id = pair.first;  // Object ID
          detach_object.object.operation = moveit_msgs::CollisionObject::REMOVE;
          planningSceneInterface_.applyAttachedCollisionObject(detach_object);
      }

      ROS_INFO("All attached objects removed.");
  }


  void removeCollisionObjectsPlanningScene()
  {
    ROS_INFO("[ObjectManipulation] Removing objects in the planningScene");
    std::vector<std::string> objectIds = planningSceneInterface_.getKnownObjectNames();
    planningSceneInterface_.removeCollisionObjects(objectIds);
  }

  void addObstaclesToPlanningScene(const std::vector<std::string> &names, const std::vector<shape_msgs::SolidPrimitive> &obstacles, const std::vector<geometry_msgs::Pose> &obstacle_poses,
                                   const std::vector<std::string> &reference_frames_of_obstacles)
  {
    for (int i = 0; i < obstacles.size(); i++)
    {
      ROS_INFO("Adding Collision object %d", i);
      moveit_msgs::CollisionObject collisionObject;
      collisionObject.id = names[i];
      collisionObject.header.frame_id = reference_frames_of_obstacles[i];
      collisionObject.operation = moveit_msgs::CollisionObject::ADD;
      collisionObject.primitives.push_back(obstacles[i]);
      collisionObject.primitive_poses.push_back(obstacle_poses[i]);
      collisionObject.header.stamp = ros::Time::now();
      ROS_INFO("Object pose: x=%f, y=%f, z=%f", obstacle_poses[i].position.x, obstacle_poses[i].position.y, obstacle_poses[i].position.z);
      ROS_INFO("Object orientation: x=%f, y=%f, z=%f, w=%f", obstacle_poses[i].orientation.x, obstacle_poses[i].orientation.y, obstacle_poses[i].orientation.z, obstacle_poses[i].orientation.w);
      planningSceneInterface_.applyCollisionObject(collisionObject);
    }
  }

  bool graspObject(moveit::planning_interface::MoveGroupInterface *groupPtr, moveit::planning_interface::PlanningSceneInterface *planningSceneInterfacePtr,
    const std::vector<std::string>& object_names,
    const std::vector<geometry_msgs::Pose>& object_poses,
    const std::vector<shape_msgs::SolidPrimitive>& object_shapes,
    const std::vector<std::string>& reference_frames,
    int index_object,
    std::string object_for_task_name1,
    geometry_msgs::PoseArray& grasping_poses,  
    geometry_msgs::PoseArray& top_object_poses,
    int& index_grasp) 
  {
      // Compute the grasping poses for the object
      ROS_INFO("Computing grasping poses for the object index: %d", index_object);
      std::string group_name = groupPtr->getName();
      std::string arm;
      if (group_name.find("left") != std::string::npos)
      {
          arm = "left";
      }else if(group_name.find("right") != std::string::npos){
          arm = "right";
      }
      else{
        ROS_ERROR("No valid group interface. No left or right arm.");
      }
      grasping_poses = computeGraspPoses(object_poses[index_object], object_shapes[index_object], top_object_poses, arm);

      top_object_pub_.publish(top_object_poses);

      grasp_pub_.publish(grasping_poses);

      // Close gripper
      float closeGripperPositions[2] = {0.0, 0.0};
      moveGripper(closeGripperPositions, arm);
      
      geometry_msgs::PoseArray pregrasp_poses;
      getPregrasp(grasping_poses, pregrasp_poses, reachingDistance_);

      pregrasp_pub_.publish(pregrasp_poses);


      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      moveit::planning_interface::MoveGroupInterface::Plan aprox_plan;

      

      for (index_grasp = 0; index_grasp < pregrasp_poses.poses.size(); index_grasp++)
      {

          removeAllAttachedObjects();

          removeCollisionObjectsPlanningScene();
      
          addObstaclesToPlanningScene(object_names,object_shapes, object_poses,reference_frames);
          groupPtr->setStartStateToCurrentState();

          
          ROS_INFO("Moving to pregrasp pose %d", index_grasp);
          geometry_msgs::Pose pose = pregrasp_poses.poses[index_grasp];
          bool valid = groupPtr->setPoseTarget(pose);
          ROS_INFO("Setting pregrasp pose target: %d", valid);
          ROS_INFO("Position: x=%f, y=%f, z=%f", pose.position.x, pose.position.y, pose.position.z);
          ROS_INFO("Orientation: x=%f, y=%f, z=%f, w=%f", pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);


          moveit::planning_interface::MoveItErrorCode code = groupPtr->plan(my_plan);

          bool successPlanning = (code == moveit::planning_interface::MoveItErrorCode::SUCCESS);
          ROS_INFO("Planning result: %d", successPlanning);
          if (successPlanning)
          {
              ROS_INFO("Found a valid pregrasp pose at index: %d", index_grasp);

              if (arm == "right"){
                  disableCollisionChecking(planningSceneInterface_,  object_for_task_name1, "gripper_right_left_finger_link");
                  disableCollisionChecking(planningSceneInterface_,  object_for_task_name1, "gripper_right_right_finger_link");
              }else{

                  disableCollisionChecking(planningSceneInterface_,  object_for_task_name1, "gripper_left_left_finger_link");
                  disableCollisionChecking(planningSceneInterface_,  object_for_task_name1, "gripper_left_right_finger_link");
              }

            
              // Last point of the trajectory
              std::vector<double> rightArmPositions = my_plan.trajectory_.joint_trajectory.points.back().positions;
              // Get the current robot state
              robot_state::RobotState start_state(*groupPtr->getCurrentState());

              const robot_state::JointModelGroup* joint_model_group = start_state.getJointModelGroup(groupPtr->getName());
              start_state.setJointGroupPositions(joint_model_group, rightArmPositions);

              groupPtr->setStartState(start_state);
              geometry_msgs::Pose last_computed_pose;
              bool success_aprox = computeSmoothCartesianPath(groupPtr, pregrasp_poses.poses[index_grasp],  grasping_poses.poses[index_grasp], 10, aprox_plan, last_computed_pose, 0.45);


              if (success_aprox)
              {
                  // We have the path from the pregrasp to the grasping pose. I need to attach the object to the gripper
                  // and move the arm to the pregrasp pose
                  // Now we need to execute the plan
                  moveGripper(closeGripperPositions, arm);
                  // execute the pregrasp plan
                  ROS_INFO("Updating MoveIt start state...");
                  groupPtr->setStartStateToCurrentState();

                  groupPtr->execute(my_plan);
                  // open gripper
                  moveGripper(openGripperPositions_, arm);

                  groupPtr->execute(aprox_plan);
                  // // close gripper
                  moveGripper(closeGripperPositions, arm);
                  break;
              }
          }
      }
      if (index_grasp< grasping_poses.poses.size())
      {
        return true;
      }
      else
      {
        return false;
      }


  }

  double computeSignedAngle(const tf2::Vector3& reference_axis,const tf2::Vector3& x_axis) {

    // Compute dot and cross product
    double dot = reference_axis.dot(x_axis);
    tf2::Vector3 cross = reference_axis.cross(x_axis);

    // Compute the angle (acos gives absolute value)
    double angle = std::atan2(cross.z(), dot);  // atan2 provides a signed angle in radians

    return angle;  // Signed angle
}


  geometry_msgs::PoseArray computeGraspPoses(const geometry_msgs::Pose &object_pose, const shape_msgs::SolidPrimitive &object_shape, geometry_msgs::PoseArray& top_object_poses,
                                             const std::string &arm)
  {
    geometry_msgs::PoseArray local_grasp_poses;
    geometry_msgs::PoseArray global_grasp_poses;
    geometry_msgs::PoseArray local_top_object_poses;
    if(object_shape.type == shape_msgs::SolidPrimitive::CYLINDER)
    {
      local_grasp_poses = computeGraspPosesCylinder(object_pose, object_shape, local_top_object_poses);
    }
    else if(object_shape.type == shape_msgs::SolidPrimitive::BOX)
    {
      local_grasp_poses = computeGraspPosesBox(object_pose, object_shape, local_top_object_poses);
    }
    else
    {
      ROS_ERROR("Object shape not supported");
    }

    global_grasp_poses = transformToGlobalFrame(local_grasp_poses, object_pose);
    geometry_msgs::PoseArray  all_top_object_poses = transformToGlobalFrame(local_top_object_poses, object_pose);
    geometry_msgs::PoseArray valid_global_grasp_poses;
    valid_global_grasp_poses.header = global_grasp_poses.header;
    top_object_poses.header = all_top_object_poses.header;

    for(int i=0; i<global_grasp_poses.poses.size(); i++){
        // Convert quaternion to rotation matrix
        geometry_msgs::Pose pose = global_grasp_poses.poses[i];
        tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
        tf2::Matrix3x3 rotation_matrix(q);

        // Extract the x-axis (first column of rotation matrix)
        tf2::Vector3 x_axis(rotation_matrix[0][0], rotation_matrix[1][0], rotation_matrix[2][0]);
        tf2::Vector3 reference_axis(1,0,0);
        // Compute the signed angle with respect to (1,0,0)
        double angle = computeSignedAngle(reference_axis,x_axis);
        ROS_INFO("ANGLE: %f", angle);
        if (arm == "right" && angle<=(M_PI/2.0+0.1) && angle>0)
        {
          valid_global_grasp_poses.poses.push_back(pose);
          top_object_poses.poses.push_back(all_top_object_poses.poses[i]);
        }
        else if (arm == "left" && angle>=-(M_PI/2.0) && angle<-0.1){
          valid_global_grasp_poses.poses.push_back(pose);
          top_object_poses.poses.push_back(all_top_object_poses.poses[i]);
        }

    }



    return valid_global_grasp_poses;
  }

  geometry_msgs::PoseArray computeGraspPosesCylinder(const geometry_msgs::Pose &object_pose, const shape_msgs::SolidPrimitive &object_shape, geometry_msgs::PoseArray& local_top_object_poses)
  {
    geometry_msgs::PoseArray local_grasp_poses;
    // Compute the grasping poses for a cylinder
     ROS_INFO("Computing grasping poses for a cylinder");

    // Cylinder dimensions
    double radius = object_shape.dimensions[0] / 2.0; // Diameter is stored in dimensions[0]
    double height = object_shape.dimensions[1];       // Height is stored in dimensions[1]

    ROS_INFO("Radius: %f", radius);
    ROS_INFO("Height: %f", height);

    int num_side_grasps = 16; // Define how many grasp points around the cylinder
    for (int i = 0; i < num_side_grasps; i++)
    {
        double angle = (i * 2.0 * M_PI) / num_side_grasps; // Evenly distribute around the circle

        for (float h = 0.5*height; h<=1.5*height; h += height) // Grasping along the height
        {
            for (int j = -1; j <= 1; j += 2) // Grasping from both sides
            {
                geometry_msgs::Pose local_grasp_pose;
                geometry_msgs::Pose local_top_object_pose;

                // **Positioning: Grasp from different Y positions around the cylinder**
                local_grasp_pose.position.x = (radius+gripperLinkDistance_) * cos(angle); // Circle X coordinate
                local_grasp_pose.position.y = (radius+gripperLinkDistance_) * sin(angle); // Circle Y coordinate
                local_grasp_pose.position.z = h;                   // Vary height

                // **Orientation: X always towards object, Z is the gripper opening**
                tf2::Quaternion q;
                q.setRPY(0, 0, angle+M_PI); // Rotate to align with grasp approach
                tf2::Quaternion q2;
                q2.setRPY(j*M_PI/2.0, 0, 0);   // Flip for better stability

                q = q * q2; // Combine rotations
                // q2.setRPY(0, M_PI/2.0, 0);   // Flip for better stability
                // q = q2 * q2; // Combine rotations

                local_grasp_pose.orientation.x = q.x();
                local_grasp_pose.orientation.y = q.y();
                local_grasp_pose.orientation.z = q.z();
                local_grasp_pose.orientation.w = q.w();

                local_top_object_pose.position.x = 0;
                local_top_object_pose.position.y = 0;
                local_top_object_pose.position.z = 2.5*height;

                local_top_object_pose.orientation = local_grasp_pose.orientation;
                // Validate quaternion before using it
                if (fabs(q.x()) < 1e-6 && fabs(q.y()) < 1e-6 && fabs(q.z()) < 1e-6 && fabs(q.w()) < 1e-6)
                {
                  ROS_WARN("Invalid quaternion detected in grasp pose %d! Setting to identity.", i);
                  q.setRPY(0, 0, 0);
                }
                q.normalize();
                local_grasp_poses.poses.push_back(local_grasp_pose);
                local_top_object_poses.poses.push_back(local_top_object_pose);
            }
        }
            
        }
    
    return local_grasp_poses;
  }

  geometry_msgs::PoseArray computeGraspPosesBox(const geometry_msgs::Pose &object_pose, const shape_msgs::SolidPrimitive &object_shape, geometry_msgs::PoseArray& local_top_object_poses)
  {
    ROS_INFO("Computing grasping poses for a box");
    geometry_msgs::PoseArray local_grasp_poses;
    // Compute the grasping poses for a box
    // I need to check the dimensions of the primitive
    double length = object_shape.dimensions[0];
    double width = object_shape.dimensions[1];
    double height = object_shape.dimensions[2];

    ROS_INFO("Length: %f", length);
    ROS_INFO("Width: %f", width);
    ROS_INFO("Height: %f", height);
    // Define the grasping poses
    if (length<= GRIPPER_MAX_WIDTH){
      for(int i = -1; i<=1; i+=2){
          for( float h=-height/5+0.5*height/5; h<=height/5; h+=0.25*height/5){
              geometry_msgs::Pose local_grasp_pose;

              local_grasp_pose.position.x =  0;
              local_grasp_pose.position.y = i*(width/2+gripperLinkDistance_-0.02);
              
              local_grasp_pose.position.z = h;
              for(int j=-1; j<=1; j+=2){
                tf2::Quaternion q;
                q.setRPY(0, 0, -i * M_PI / 2.0);  // Rotate to align gripper with Y-axis
                tf2::Quaternion q2;
                q2.setRPY(j * M_PI / 2.0, 0, 0);
                q = q*q2;
                local_grasp_pose.orientation.x = q.x();
                local_grasp_pose.orientation.y = q.y();
                local_grasp_pose.orientation.z = q.z();
                local_grasp_pose.orientation.w = q.w();
                local_grasp_poses.poses.push_back(local_grasp_pose);

                geometry_msgs::Pose local_top_object_pose;
                local_top_object_pose.position.x = 0;
                local_top_object_pose.position.y = -i*width/2.0;
                local_top_object_pose.position.z = 0.7*height;

                local_top_object_pose.orientation = local_grasp_pose.orientation;
                local_top_object_poses.poses.push_back(local_top_object_pose);
              }
              
          }

      }
    }
    return local_grasp_poses;
  }

  geometry_msgs::PoseArray transformToGlobalFrame(const geometry_msgs::PoseArray &local_grasp_poses, const geometry_msgs::Pose &object_pose)
  {
    geometry_msgs::PoseArray global_grasp_poses;
    global_grasp_poses.header.frame_id = "aruco_base";
    global_grasp_poses.header.stamp = ros::Time::now();
    tf2::Transform obj_transform;
    tf2::Quaternion obj_orientation(
        object_pose.orientation.x,
        object_pose.orientation.y,
        object_pose.orientation.z,
        object_pose.orientation.w
    );
    obj_transform.setOrigin(tf2::Vector3(object_pose.position.x, object_pose.position.y, object_pose.position.z));
    obj_transform.setRotation(obj_orientation);

    for (const auto &local_grasp_pose : local_grasp_poses.poses)
    {
      tf2::Transform grasp_transform;
      tf2::Quaternion grasp_orientation(
          local_grasp_pose.orientation.x,
          local_grasp_pose.orientation.y,
          local_grasp_pose.orientation.z,
          local_grasp_pose.orientation.w
      );
      grasp_transform.setOrigin(tf2::Vector3(local_grasp_pose.position.x, local_grasp_pose.position.y, local_grasp_pose.position.z));
      grasp_transform.setRotation(grasp_orientation);

      tf2::Transform global_grasp_transform = obj_transform * grasp_transform;

      geometry_msgs::Pose global_grasp_pose;
      global_grasp_pose.position.x = global_grasp_transform.getOrigin().getX();
      global_grasp_pose.position.y = global_grasp_transform.getOrigin().getY();
      global_grasp_pose.position.z = global_grasp_transform.getOrigin().getZ();
      tf2::Quaternion q = global_grasp_transform.getRotation();
      global_grasp_pose.orientation.x = q.x();
      global_grasp_pose.orientation.y = q.y();
      global_grasp_pose.orientation.z = q.z();
      global_grasp_pose.orientation.w = q.w();

      global_grasp_poses.poses.push_back(global_grasp_pose);
    }

    return global_grasp_poses;
  }

  void getPregrasp(const geometry_msgs::PoseArray &grasping_poses, geometry_msgs::PoseArray &pregrasp_poses, double pregrasp_distance)
  {
    pregrasp_poses.poses.clear();
    pregrasp_poses.header.frame_id = grasping_poses.header.frame_id;
    pregrasp_poses.header.stamp = grasping_poses.header.stamp;

    for (const auto &pose : grasping_poses.poses)
    {
      // Convert geometry_msgs::Pose to tf2::Transform
      tf2::Transform transform;
      tf2::fromMsg(pose, transform);

      // Define the translation along the local negative x-axis
      tf2::Vector3 pregrasp_translation(-pregrasp_distance, 0.0, 0.0);

      // Apply the translation to the transform
      transform.setOrigin(transform.getOrigin() + transform.getBasis() * pregrasp_translation);

      // Convert tf2::Transform back to geometry_msgs::Pose
      geometry_msgs::Pose pregrasp_pose;
      tf2::toMsg(transform, pregrasp_pose);

      pregrasp_poses.poses.push_back(pregrasp_pose);
    }
  }

  void createPouringPose(const std::vector<geometry_msgs::Pose>& object_poses, const std::vector<shape_msgs::SolidPrimitive>& object_shapes,
    const std::vector<std::string>& reference_frames, const int& index_object_from, const std::string& object_for_task_name1,
    const int& index_object_to, const std::string& object_for_task_name2, geometry_msgs::Pose grasping_pose,  
    geometry_msgs::Pose top_object_pose, const double &rotation_angle, const double& distance_from_top,
    geometry_msgs::PoseStamped& gripper_pouring_pose)
    {
    if (object_shapes[index_object_to].type == shape_msgs::SolidPrimitive::CYLINDER)
    {
      double radius = object_shapes[index_object_to].dimensions[0] / 2.0; // Diameter is stored in dimensions[0]
      double height = object_shapes[index_object_to].dimensions[1];       // Height is stored in dimensions[1]

      ROS_INFO("Radius: %f", radius);
      ROS_INFO("Height: %f", height);

      geometry_msgs::PoseStamped pouring_pose;
      pouring_pose.header.frame_id = reference_frames[index_object_to];
      pouring_pose.header.stamp = ros::Time::now();
      pouring_pose.pose.position.x = object_poses[index_object_to].position.x;
      pouring_pose.pose.position.y = object_poses[index_object_to].position.y;

      pouring_pose.pose.position.z = object_poses[index_object_to].position.z + 2*height+distance_from_top;

      // Orientation of the top pose when grasping
      pouring_pose.pose.orientation = top_object_pose.orientation;

      pouring_pose.pose = rotatePoseAroundLocalZ(pouring_pose.pose, rotation_angle); // Maybe is not always positive 
      pouring_pose_top_pub_.publish(pouring_pose);

      tf2::Transform T_top_grasp = computeRelativeTransform(grasping_pose, top_object_pose);

      gripper_pouring_pose.header = pouring_pose.header;

      gripper_pouring_pose.pose = computeNewPose(pouring_pose.pose, T_top_grasp);

      new_grasping_pose_pub_.publish(gripper_pouring_pose);

    }


  }

  bool pourInto(moveit::planning_interface::MoveGroupInterface *groupPtr, moveit::planning_interface::PlanningSceneInterface *planningSceneInterfacePtr,
    const std::vector<std::string>& object_names,
    const std::vector<geometry_msgs::Pose>& object_poses,
    const std::vector<shape_msgs::SolidPrimitive>& object_shapes,
    const std::vector<std::string>& reference_frames,
    const int& index_object_from,
    const std::string& object_for_task_name1,
    const int& index_object_to,
    const std::string& object_for_task_name2,
    geometry_msgs::Pose grasping_pose,  
    geometry_msgs::Pose top_object_pose)
  {
    bool success = false;
    moveit::planning_interface::MoveGroupInterface::Plan move_to_start_pouring_plan, pour_plan, up_plan, place_plan, backwards_plan;
    ROS_INFO("POUR %s INTO % s", object_for_task_name1.c_str(), object_for_task_name2.c_str());
    if (object_shapes[index_object_to].type == shape_msgs::SolidPrimitive::CYLINDER)
    {
      geometry_msgs::PoseStamped new_gripper_pouring_pose;
      createPouringPose(object_poses, object_shapes, reference_frames, index_object_from, object_for_task_name1, index_object_to, object_for_task_name2,
                        grasping_pose, top_object_pose, 20, 0.15,new_gripper_pouring_pose);

      std::string group_name = groupPtr->getName();
      std::string arm;
      if (group_name.find("left") != std::string::npos)
      {
          arm = "left";
      }else if(group_name.find("right") != std::string::npos){
          arm = "right";
      }
      
      moveit_msgs::AttachedCollisionObject attached_object;
      if(arm == "right"){
          attached_object.link_name = "gripper_right_tool_link";
          attached_object.object.id = object_for_task_name1;
          attached_object.object.header.frame_id = "aruco_base";
          attached_object.object.operation = attached_object.object.ADD;
          planningSceneInterface_.applyAttachedCollisionObject(attached_object);
      }else{
          attached_object.link_name = "gripper_left_tool_link";
          attached_object.object.id = object_for_task_name1;
          attached_object.object.header.frame_id = "aruco_base";
          attached_object.object.operation = attached_object.object.ADD;
          planningSceneInterface_.applyAttachedCollisionObject(attached_object);
      }
                  
      ROS_INFO("Attached object");


      groupPtr->setMaxVelocityScalingFactor(0.3);
      groupPtr->setMaxAccelerationScalingFactor(0.3);

      ros::Duration(0.1).sleep();
      groupPtr->setStartStateToCurrentState();

      geometry_msgs::Pose last_computed_pose, last_computed_pose2, last_computed_pose3, last_computed_pose4;
      double height = object_shapes[index_object_to].dimensions[1];       // Height is stored in dimensions[1]

      moveit::core::RobotState last_robot_state(*groupPtr->getCurrentState());

      // Convert joint state to Cartesian pose
      const Eigen::Isometry3d& end_effector_state = last_robot_state.getGlobalLinkTransform(groupPtr->getEndEffectorLink());

      geometry_msgs::Pose last_computed_pose0;
      last_computed_pose0.position.x = end_effector_state.translation().x()-0.8; // Make the correct transformation please
      last_computed_pose0.position.y = end_effector_state.translation().y();
      last_computed_pose0.position.z = end_effector_state.translation().z();

      Eigen::Quaterniond quat(end_effector_state.rotation());
      last_computed_pose0.orientation.x = quat.x();
      last_computed_pose0.orientation.y = quat.y();
      last_computed_pose0.orientation.z = quat.z();
      last_computed_pose0.orientation.w = quat.w();

      bool success_move_to_pour = computeSmoothCartesianPath(groupPtr, last_computed_pose0, new_gripper_pouring_pose.pose, 10, move_to_start_pouring_plan, last_computed_pose, 0.8);
      
      if (!success_move_to_pour)
      {
        return false;
      }
     
      // Last point of the trajectory
      std::vector<double> armPositions = move_to_start_pouring_plan.trajectory_.joint_trajectory.points.back().positions;
      // Get the current robot state
      robot_state::RobotState start_state(*groupPtr->getCurrentState());

      const robot_state::JointModelGroup* joint_model_group = start_state.getJointModelGroup(groupPtr->getName());
      start_state.setJointGroupPositions(joint_model_group, armPositions);

      groupPtr->setStartState(start_state);
      
      createPouringPose(object_poses, object_shapes, reference_frames, index_object_from, object_for_task_name1, index_object_to, object_for_task_name2,
                        grasping_pose, top_object_pose, 90, 0.0, new_gripper_pouring_pose);
      


      bool success_pouring_first_phase = computeSmoothCartesianPath(groupPtr, last_computed_pose, new_gripper_pouring_pose.pose, 30, pour_plan, last_computed_pose2, 0.5);

      if (!success_pouring_first_phase)
      {
        return false;
      }

      // Last point of the trajectory
      armPositions = pour_plan.trajectory_.joint_trajectory.points.back().positions;
      start_state.setJointGroupPositions(joint_model_group, armPositions);

      groupPtr->setStartState(start_state);
      
      createPouringPose(object_poses, object_shapes, reference_frames, index_object_from, object_for_task_name1, index_object_to, object_for_task_name2,
                        grasping_pose, top_object_pose, 30, 0.15, new_gripper_pouring_pose);

      bool success_up = computeSmoothCartesianPath(groupPtr, last_computed_pose2, new_gripper_pouring_pose.pose, 30, up_plan, last_computed_pose3, 0.8);


      if (!success_up)
      {
        return false;
      }

      // Last point of the trajectory
      armPositions = up_plan.trajectory_.joint_trajectory.points.back().positions;
      start_state.setJointGroupPositions(joint_model_group, armPositions);

      groupPtr->setStartState(start_state);
      bool success_place = computeSmoothCartesianPath(groupPtr, last_computed_pose3, grasping_pose, 30, place_plan, last_computed_pose4, 0.9);

      if(!success_place){
        return false;
      }



      groupPtr->execute(move_to_start_pouring_plan);
          // pour_plan.trajectory_.joint_trajectory.header.stamp = ros::Time::now();
          // for (int i=0; i<pour_plan.trajectory_.joint_trajectory.points.size(); i++)
          // {
          //   ROS_INFO("  Time From Start: %f", pour_plan.trajectory_.joint_trajectory.points[i].time_from_start.toSec());
          // }
      groupPtr->execute(pour_plan);

      ros::Duration(3.0).sleep();
      groupPtr->execute(up_plan);
      groupPtr->execute(place_plan);


      moveGripper(openGripperPositions_, arm);


      removeAllAttachedObjects();

      geometry_msgs::PoseArray grasping_poses;
      grasping_poses.poses.push_back(grasping_pose);
      geometry_msgs::PoseArray pregrasp_poses;

      getPregrasp(grasping_poses, pregrasp_poses, reachingDistance_);

      groupPtr->setStartStateToCurrentState();

      geometry_msgs::Pose last_computed_pose5;

      bool success_move_backwards = computeSmoothCartesianPath(groupPtr, grasping_pose, pregrasp_poses.poses[0], 10, backwards_plan, last_computed_pose5, 0.8);
      
      if (!success_move_backwards)
      {
        return false;
      }
      groupPtr->execute(backwards_plan);

      moveGripper(closeGripperPositions_, arm);

      return true;
      

      
      //     geometry_msgs::PoseStamped pouring_pose2;
      //     pouring_pose2.header.frame_id = reference_frames[index_object_to];
      //     pouring_pose2.header.stamp = ros::Time::now();
      //     pouring_pose2.pose.position.x = object_poses[index_object_to].position.x;
      //     pouring_pose2.pose.position.y = object_poses[index_object_to].position.y;
      //     pouring_pose2.pose.position.z = object_poses[index_object_to].position.z + 2*height;    
      //     pouring_pose2.pose.orientation = top_object_pose.orientation;
      //     pouring_pose2.pose = rotatePoseAroundLocalZ(pouring_pose2.pose, 90); 
      //     pouring_pose_top_pub_.publish(pouring_pose2);
      //     geometry_msgs::PoseStamped new_grasping_pose2;
      //     new_grasping_pose2.header = new_gripper_pouring_pose.header;
      //     new_grasping_pose2.pose = computeNewPose(pouring_pose2.pose, T_top_grasp);
      //     new_grasping_pose_pub_.publish(new_grasping_pose2);


      //     geometry_msgs::Pose start_pose;
      //     start_pose.position.x = last_computed_pose.position.x;
      //     start_pose.position.y = last_computed_pose.position.y;
      //     start_pose.position.z = last_computed_pose.position.z;

      //     start_pose.orientation.x = last_computed_pose.orientation.x;
      //     start_pose.orientation.y = last_computed_pose.orientation.y;
      //     start_pose.orientation.z = last_computed_pose.orientation.z;
      //     start_pose.orientation.w = last_computed_pose.orientation.w;

      //     ROS_INFO("%f %f %f", new_grasping_pose.pose.position.x, new_grasping_pose.pose.position.y, new_grasping_pose.pose.position.z);


      //     geometry_msgs::Pose last_computed_pose2;



      //     bool success_pouring_first_phase = computeSmoothCartesianPath(groupPtr, start_pose, new_grasping_pose2.pose, 25, pour_plan, last_computed_pose2, 0.8);

      //     if (success_pouring_first_phase) {  // If the path is successfully planned
      //         groupPtr->execute(pour_plan);
      //         geometry_msgs::PoseStamped pouring_pose3;
      //         pouring_pose3.header.frame_id = reference_frames[index_object_to];
      //         pouring_pose3.header.stamp = ros::Time::now();
      //         pouring_pose3.pose.position.x = object_poses[index_object_to].position.x;
      //         pouring_pose3.pose.position.y = object_poses[index_object_to].position.y;
      //         pouring_pose3.pose.position.z = object_poses[index_object_to].position.z + 2.0*height;    
      //         pouring_pose3.pose.orientation = top_object_pose.orientation;
      //         pouring_pose3.pose = rotatePoseAroundLocalZ(pouring_pose3.pose, 105); // Maybe is not always positive 
      //         pouring_pose_top_pub_.publish(pouring_pose3);
      //         geometry_msgs::PoseStamped new_grasping_pose3;
      //         new_grasping_pose3.header = new_gripper_pouring_pose.header;
      //         new_grasping_pose3.pose = computeNewPose(pouring_pose3.pose, T_top_grasp);
      //         new_grasping_pose_pub_.publish(new_grasping_pose3);
              
      //         geometry_msgs::Pose last_computed_pose3;

      //         bool success_pouring_second_phase = computeSmoothCartesianPath(groupPtr, last_computed_pose2, new_grasping_pose3.pose, 3, pour2_plan, last_computed_pose3,0.2);
      //         if (success_pouring_second_phase) {  // If the path is successfully planned
      //           groupPtr->execute(pour2_plan);
      //         }

      //     }
      // }
    }
  }



  geometry_msgs::Pose rotatePoseAroundLocalZ(const geometry_msgs::Pose& pose, double angle_degrees) {
    geometry_msgs::Pose new_pose = pose;  // Copy input pose

    // Convert input orientation to a quaternion
    tf2::Quaternion current_orientation(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

    // Convert desired rotation angle from degrees to radians
    double angle_radians = angle_degrees * M_PI / 180.0;

    // Create a quaternion representing the rotation around the local Z-axis
    tf2::Quaternion rotation;
    rotation.setRPY(0, 0, angle_radians);  // Roll = 0, Pitch = 0, Yaw = rotation around Z-axis

    // Apply the rotation (multiplication applies rotation in local frame)
    tf2::Quaternion new_orientation = current_orientation * rotation;
    new_orientation.normalize();  // Normalize to avoid numerical errors

    // Update the pose with the new orientation
    new_pose.orientation.x = new_orientation.x();
    new_pose.orientation.y = new_orientation.y();
    new_pose.orientation.z = new_orientation.z();
    new_pose.orientation.w = new_orientation.w();

    return new_pose;
  }

  // Function to compute the relative transform from pose_top_aruco to pose_center_aruco
  tf2::Transform computeRelativeTransform(const geometry_msgs::Pose& pose_ref_center, const geometry_msgs::Pose& pose_ref_top) {
    // Convert poses to tf2::Transform
    tf2::Transform T_ref_center, T_ref_top;
    T_ref_center.setOrigin(tf2::Vector3(pose_ref_center.position.x, pose_ref_center.position.y, pose_ref_center.position.z));
    T_ref_center.setRotation(tf2::Quaternion(pose_ref_center.orientation.x, pose_ref_center.orientation.y, pose_ref_center.orientation.z, pose_ref_center.orientation.w));

    T_ref_top.setOrigin(tf2::Vector3(pose_ref_top.position.x, pose_ref_top.position.y, pose_ref_top.position.z));
    T_ref_top.setRotation(tf2::Quaternion(pose_ref_top.orientation.x, pose_ref_top.orientation.y, pose_ref_top.orientation.z, pose_ref_top.orientation.w));

    // Compute relative transform: T_top_center = T_ref_center * T_ref_top⁻¹
    tf2::Transform T_top_center = T_ref_top.inverse() * T_ref_center;
    return T_top_center;
  }

  // Function to compute new pose of pose_center_aruco given new pose_top_aruco and relative transformation
  geometry_msgs::Pose computeNewPose(const geometry_msgs::Pose& new_pose_top, const tf2::Transform& T_top_center) {
      tf2::Transform T_ref_new_top;
      T_ref_new_top.setOrigin(tf2::Vector3(new_pose_top.position.x, new_pose_top.position.y, new_pose_top.position.z));
      T_ref_new_top.setRotation(tf2::Quaternion(new_pose_top.orientation.x, new_pose_top.orientation.y, new_pose_top.orientation.z, new_pose_top.orientation.w));

      // Compute new pose of center: T_ref_new_center = T_ref_new_top * T_top_center
      tf2::Transform T_ref_new_center = T_ref_new_top * T_top_center;

      // Convert back to geometry_msgs::Pose
      geometry_msgs::Pose new_pose_center;
      new_pose_center.position.x = T_ref_new_center.getOrigin().x();
      new_pose_center.position.y = T_ref_new_center.getOrigin().y();
      new_pose_center.position.z = T_ref_new_center.getOrigin().z();
      new_pose_center.orientation.x = T_ref_new_center.getRotation().x();
      new_pose_center.orientation.y = T_ref_new_center.getRotation().y();
      new_pose_center.orientation.z = T_ref_new_center.getRotation().z();
      new_pose_center.orientation.w = T_ref_new_center.getRotation().w();

      return new_pose_center;
  }

  bool computeSmoothCartesianPath(moveit::planning_interface::MoveGroupInterface* groupPtr, const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2, const int &steps, 
                          moveit::planning_interface::MoveGroupInterface::Plan& result_plan, geometry_msgs::Pose &last_computed_pose, const double &fraction_threshold) 
  {
    if (!groupPtr) {
        ROS_ERROR("MoveGroup pointer is null!");
        return false;
    }

    tf2::Quaternion q_start, q_target;
    tf2::convert(pose1.orientation, q_start);
    tf2::convert(pose2.orientation, q_target);


    // Generate waypoints with interpolated position and orientation
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::PoseArray pour_trajectory;
    pour_trajectory.header.frame_id = "aruco_base";
    pour_trajectory.header.stamp = ros::Time::now();
    double alpha;
    ROS_INFO("steps:%d", steps);
    for (int i = 1; i <= steps; i++) {
        alpha = static_cast<double>(i) / steps;
        ROS_INFO("ALPHA:%f", alpha);
        geometry_msgs::Pose intermediate_pose;
        
        // Linearly interpolate position
        intermediate_pose.position.x = pose1.position.x + alpha * (pose2.position.x - pose1.position.x);
        intermediate_pose.position.y = pose1.position.y + alpha * (pose2.position.y - pose1.position.y);
        intermediate_pose.position.z = pose1.position.z + alpha * (pose2.position.z - pose1.position.z);

        // Slerp for smooth orientation transition
        
        tf2::Quaternion q_intermediate;
        if(q_start == q_target){
          q_intermediate = q_start;
        }else{
         q_intermediate = q_start.slerp(q_target, alpha);
        }
        intermediate_pose.orientation.x = q_intermediate.x();
        intermediate_pose.orientation.y = q_intermediate.y();
        intermediate_pose.orientation.z = q_intermediate.z();
        intermediate_pose.orientation.w = q_intermediate.w();
        pour_trajectory.poses.push_back(intermediate_pose);
        waypoints.push_back(intermediate_pose);
    }

    trajectory_pub_.publish(pour_trajectory);

    // Compute Cartesian path
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = groupPtr->computeCartesianPath(waypoints, alpha, 0.0, trajectory);
    ROS_INFO("Cartesian path computed (%.2f%% achieved)", fraction * 100.0);
    
    if (fraction > fraction_threshold) {
        result_plan.trajectory_ = trajectory;
        


        // Extract the last computed pose from the trajectory
        int last_traj_index = trajectory.joint_trajectory.points.size() - 1;

        if (last_traj_index >= 0) {
            // Get last joint state from the trajectory
            moveit::core::RobotState last_robot_state(*groupPtr->getCurrentState());
            last_robot_state.setVariablePositions(trajectory.joint_trajectory.joint_names, 
                                                  trajectory.joint_trajectory.points[last_traj_index].positions);

            // Convert joint state to Cartesian pose
            const Eigen::Isometry3d& end_effector_state = last_robot_state.getGlobalLinkTransform(groupPtr->getEndEffectorLink());

            last_computed_pose.position.x = end_effector_state.translation().x()-0.8; // Make the correct transformation please
            last_computed_pose.position.y = end_effector_state.translation().y();
            last_computed_pose.position.z = end_effector_state.translation().z();

            Eigen::Quaterniond quat(end_effector_state.rotation());
            last_computed_pose.orientation.x = quat.x();
            last_computed_pose.orientation.y = quat.y();
            last_computed_pose.orientation.z = quat.z();
            last_computed_pose.orientation.w = quat.w();

            ROS_INFO("Last computed pose: Position [%f, %f, %f] Orientation [%f, %f, %f, %f]",
                    last_computed_pose.position.x, last_computed_pose.position.y, last_computed_pose.position.z,
                    last_computed_pose.orientation.x, last_computed_pose.orientation.y,
                    last_computed_pose.orientation.z, last_computed_pose.orientation.w);
          }
        return true;

    }
    else {
        ROS_WARN("Cartesian path planning failed. Only %.2f%% achieved.", fraction * 100.0);
        return false;
    }
  }


    bool initializeArmPosition( moveit::planning_interface::MoveGroupInterface* groupPtr, const std::vector<double> &initArmPositions)
    {
        ROS_INFO("Setting Arm to init position: (%f %f %f %f %f %f %f)", initArmPositions[0], initArmPositions[1], initArmPositions[2],
                 initArmPositions[3], initArmPositions[4], initArmPositions[5], initArmPositions[6]);

        groupPtr->setStartStateToCurrentState();
        groupPtr->setJointValueTarget(initArmPositions);
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        moveit::planning_interface::MoveItErrorCode code = groupPtr->plan(plan);
        bool successPlanning = (code == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO("Arm planning result: %d", successPlanning);
        if (successPlanning)
        {
            moveit::planning_interface::MoveItErrorCode e = groupPtr->move();
            if (e == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            {
                ROS_INFO("Arm success in moving to the initial joints position.");
                return true;
            }
            else
            {
                ROS_INFO("Arm Error in moving  to the initial joints position.");
                return false;
            }
        }
        else
        {
            return false;
        }
    }

    bool aproachGraspedObjectToUser(moveit::planning_interface::MoveGroupInterface *groupPtr, moveit::planning_interface::PlanningSceneInterface *planningSceneInterfacePtr,
                                   std::string object_for_task_name1, geometry_msgs::PoseArray grasping_poses, int index_grasp, const std::vector<geometry_msgs::Point> &bounding_zone)
    {
      moveit::planning_interface::MoveGroupInterface::Plan handover_plan;

      
      if (index_grasp < grasping_poses.poses.size())
      {
          ROS_INFO("Found a valid grasp pose at index: %d", index_grasp);
          // Attach the object to the gripper
          moveit_msgs::AttachedCollisionObject attached_object;

          std::string group_name = groupPtr->getName();
          std::string link_name;
          std::string arm;

          if (group_name.find("left") != std::string::npos)
          {
            link_name = "gripper_left_tool_link";
            arm = "left";
          }else if(group_name.find("right") != std::string::npos){
            link_name = "gripper_right_tool_link";
            arm = "right";
          }

          attached_object.link_name = link_name;
          attached_object.object.id = object_for_task_name1;
          attached_object.object.header.frame_id = "aruco_base";
          attached_object.object.operation = attached_object.object.ADD;
          planningSceneInterface_.applyAttachedCollisionObject(attached_object);
                  
          ROS_INFO("Attached object");

          for (int i = 0; i < bounding_zone.size(); i++)
          {
              ROS_INFO("Bounding zone %d: x=%f, y=%f, z=%f", i, bounding_zone[i].x, bounding_zone[i].y, bounding_zone[i].z);
          }


          groupPtr->setMaxVelocityScalingFactor(0.3);
          groupPtr->setMaxAccelerationScalingFactor(0.3);

      
          groupPtr->setStartStateToCurrentState();

          geometry_msgs::Pose grasp_pose =  grasping_poses.poses[index_grasp];

          // Calculate the center of the bounding zone
          geometry_msgs::Point center;
          center.x = (bounding_zone[0].x + bounding_zone[1].x + bounding_zone[2].x + bounding_zone[3].x) / 4.0;
          center.y = (bounding_zone[0].y + bounding_zone[1].y + bounding_zone[2].y + bounding_zone[3].y) / 4.0;
          center.z = (bounding_zone[0].z + bounding_zone[1].z + bounding_zone[2].z + bounding_zone[3].z) / 4.0;



          bool success = false;
          for (double step = 0.1; step <= 1.0; step += 0.1) 
          {  // Start from a small positive step
              for (double alpha = std::max(0.0, 0.5 - step); alpha <= std::min(1.0, 0.5 + step); alpha += step) 
              {
                  for (double beta = std::max(0.0, 0.5 - step); beta <= std::min(1.0, 0.5 + step); beta += step) 
                  {
                      geometry_msgs::Pose handover_pose = interpolate(alpha, beta, grasp_pose.orientation, bounding_zone);

                      //Plan a cartesian path to the handover pose
                      ROS_INFO("Planning cartesian path to handover pose %f %f %f", handover_pose.position.x, handover_pose.position.y, handover_pose.position.z);
                      std::vector<geometry_msgs::Pose> waypoints;
                      // waypoints.push_back(grasp_pose);
                      waypoints.push_back(handover_pose);

                      moveit_msgs::RobotTrajectory trajectory;
                      const double jump_threshold = 0.0;
                      const double eef_step = 0.01;

                      double fraction = groupPtr->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

                      if (fraction>=0.8)
                      {
                          ROS_INFO("Computed Cartesian path successfully. Executing...");
                          handover_plan.trajectory_ = trajectory;
                          success = true;
                          break;
                      }
                                  
                  }
                  if (success) 
                  {
                      break;
                  }
                            
                }
                if (success) 
                {
                    break;
                }
          }
          
          if (success)
          {
              ROS_INFO("Handover successfully planned");

              // execute the handover plan
              groupPtr->execute(handover_plan);
              // wait seconds
              ros::Duration(2.0).sleep();

              removeAllAttachedObjects();
              // open gripper
              moveGripper(openGripperPositions_, arm);

              // remove the collision object specific
              planningSceneInterface_.removeCollisionObjects({object_for_task_name1});                


              bool init = initializeArmPosition(groupPtr,initRightArmPositions_);

              if (init){
                return true;
              }
          }
          else{
              return false;
          }
          
      }
    }
    


  // bool goToGraspingPose(const geometry_msgs::Pose &graspingPose)
  // {

  //   moveit::planning_interface::MoveGroupInterface *groupAuxArmTorsoPtr_;

  //   groupAuxArmTorsoPtr_ = groupRightArmPtr_;

  //   groupAuxArmTorsoPtr_->setMaxVelocityScalingFactor(0.1);
  //   groupAuxArmTorsoPtr_->setMaxAccelerationScalingFactor(0.1);

  //   geometry_msgs::PoseStamped currentPose = groupAuxArmTorsoPtr_->getCurrentPose();
  //   KDL::Frame frameEndWrtBase;
  //   tf::poseMsgToKDL(currentPose.pose, frameEndWrtBase);
  //   KDL::Frame frameToolWrtEnd;
  //   frameToolWrtEnd.p[0] = +reachingDistance_;
  //   KDL::Frame frameToolWrtBase = frameEndWrtBase * frameToolWrtEnd;

  //   geometry_msgs::Pose toolPose;
  //   tf::poseKDLToMsg(frameToolWrtBase, toolPose);

  //   std::vector<geometry_msgs::Pose> waypoints;
  //   waypoints.push_back(toolPose);

  //   groupAuxArmTorsoPtr_->setStartStateToCurrentState();

  //   moveit_msgs::RobotTrajectory trajectory;
  //   const double jump_threshold = 0.00;
  //   const double eef_step = 0.001;
  //   double fraction = groupAuxArmTorsoPtr_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  //   ROS_INFO("ObjectManipulationServer] plan (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

  //   moveit::planning_interface::MoveGroupInterface::Plan planAproach;

  //   planAproach.trajectory_ = trajectory;

  //   sleep(1.0);

  //   moveit::planning_interface::MoveItErrorCode e = groupAuxArmTorsoPtr_->execute(planAproach);

  //   // check if there is an error
  //   if (e == moveit::planning_interface::MoveItErrorCode::SUCCESS){
  //     return true;
  //   }
  //   else{
  //     return false;
  //   }
  // }

  // bool goToDistancedPose(double distanced)
  // {

  //   moveit::planning_interface::MoveGroupInterface *groupAuxArmTorsoPtr_;

  //   groupAuxArmTorsoPtr_ = groupRightArmPtr_;

  //   groupAuxArmTorsoPtr_->setMaxVelocityScalingFactor(0.1);
  //   groupAuxArmTorsoPtr_->setMaxAccelerationScalingFactor(0.1);

  //   geometry_msgs::PoseStamped currentPose = groupAuxArmTorsoPtr_->getCurrentPose();
  //   KDL::Frame frameEndWrtBase;
  //   tf::poseMsgToKDL(currentPose.pose, frameEndWrtBase);
  //   KDL::Frame frameToolWrtEnd;
  //   frameToolWrtEnd.p[0] = - distanced;
  //   KDL::Frame frameToolWrtBase = frameEndWrtBase * frameToolWrtEnd;

  //   geometry_msgs::Pose toolPose;
  //   tf::poseKDLToMsg(frameToolWrtBase, toolPose);

  //   std::vector<geometry_msgs::Pose> waypoints;
  //   waypoints.push_back(toolPose);

  //   groupAuxArmTorsoPtr_->setStartStateToCurrentState();

  //   moveit_msgs::RobotTrajectory trajectory;
  //   const double jump_threshold = 0.00;
  //   const double eef_step = 0.001;
  //   double fraction = groupAuxArmTorsoPtr_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  //   ROS_INFO("ObjectManipulationServer] plan (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

  //   moveit::planning_interface::MoveGroupInterface::Plan planAproach;

  //   planAproach.trajectory_ = trajectory;

  //   sleep(1.0);

  //   moveit::planning_interface::MoveItErrorCode e = groupAuxArmTorsoPtr_->execute(planAproach);

  //   return true;
  // }

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

//   void moveGripper(const float positions[2], std::string name)
// {
//     follow_joint_control_client_Ptr auxGripperClient;

//     if (name == "right")
//     {
//         auxGripperClient = rightGripperClient_;
//     }
//     else if (name == "left")
//     {
//         auxGripperClient = leftGripperClient_;
//     }

//     control_msgs::FollowJointTrajectoryGoal gripperGoal;
//     ROS_INFO("[ObjectManipulationServer] Setting gripper %s position: (%f ,%f)", name.c_str(), positions[0], positions[1]);
//     waypointGripperGoal(name, gripperGoal, positions, 0.5);

//     // Sends the command to start the given trajectory now
//     gripperGoal.trajectory.header.stamp = ros::Time(0);
//     auxGripperClient->sendGoal(gripperGoal);

//     ros::Time start_time = ros::Time::now();
//     ros::Duration max_duration(1.0);  // Set a time limit of 1 second

//     // Define a tolerance value
//     const float tolerance = 0.01;
//     float current_position[2];

//     while (ros::ok())
//     {
//         // Check if the goal state is done
//         if (auxGripperClient->getState().isDone())
//         {
//             ROS_INFO("[ObjectManipulationServer] Gripper goal state is done, checking if goal is reached.");

//             // Get the current position of the gripper (replace this with actual implementation)
//             getCurrentGripperPose(name, current_position);

//             // Check if the current position is close to the goal
//             if (std::abs(current_position[0] - positions[0]) < tolerance && 
//                 std::abs(current_position[1] - positions[1]) < tolerance)
//             {
//                 ROS_INFO("[ObjectManipulationServer] Gripper position reached the goal: (%f, %f)", current_position[0], current_position[1]);
//                 break;  // Exit the loop if the goal is reached
//             }
//             else
//             {
//                 ROS_WARN("[ObjectManipulationServer] Gripper did not reach goal yet. Resending current pose as goal.");
//                 waypointGripperGoal(name, gripperGoal, current_position, 0.5);
//                 gripperGoal.trajectory.header.stamp = ros::Time(0);
//                 auxGripperClient->sendGoal(gripperGoal);
//             }
//         }

//         // Check if the timeout is reached
//         if (ros::Time::now() - start_time > max_duration)
//         {
//             ROS_INFO("[ObjectManipulationServer] Timeout reached, sending current pose as goal.");

//             // Get the current position of the gripper
//             getCurrentGripperPose(name, current_position);

//             // Send current position as a goal
//             waypointGripperGoal(name, gripperGoal, current_position, 0.5);
//             gripperGoal.trajectory.header.stamp = ros::Time(0);
//             auxGripperClient->sendGoal(gripperGoal);

//             // Reset the timer
//             start_time = ros::Time::now();
//         }

//         ros::Duration(0.1).sleep();  // sleep for 0.1 seconds
//     }

//     ROS_INFO("[ObjectManipulationServer] Gripper set to position: (%f, %f)", positions[0], positions[1]);
// }
// // Callback function for the joint states topic
void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    // Assuming the gripper joints are named "gripper_right_left_finger_joint" and "gripper_right_right_finger_joint"
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        if (msg->name[i] == "gripper_right_left_finger_joint")
        {
            current_gripper_positions_[0] = msg->position[i];  // Update left finger position
        }
        else if (msg->name[i] == "gripper_right_right_finger_joint")
        {
            current_gripper_positions_[1] = msg->position[i];  // Update right finger position
        }
    }
}

// Function to get the current gripper pose
void getCurrentGripperPose(std::string name, float positions[2])
{
    // Copy the current gripper positions to the output array
    positions[0] = current_gripper_positions_[0];
    positions[1] = current_gripper_positions_[1];

    ROS_INFO("[ObjectManipulationServer] Current gripper %s position: (%f, %f)", name.c_str(), positions[0], positions[1]);
}

  bool moveToConfortablePose(const geometry_msgs::Pose &pose)
  {


    moveit::planning_interface::MoveGroupInterface *groupAuxArmTorsoPtr_;

    groupAuxArmTorsoPtr_ = groupRightArmPtr_;

    groupAuxArmTorsoPtr_->setMaxVelocityScalingFactor(0.3);
    groupAuxArmTorsoPtr_->setMaxAccelerationScalingFactor(0.3);

    geometry_msgs::PoseStamped grasp_pose_stamped = groupAuxArmTorsoPtr_->getCurrentPose();
    geometry_msgs::Pose grasp_pose = grasp_pose_stamped.pose;

    // Define comfortable pose with fixed position
    geometry_msgs::Pose comfortable_pose = grasp_pose;
    // comfortable_pose.position.x = 0.527;
    // comfortable_pose.position.y = -0.325;
    comfortable_pose.position.z +=0.1;
    
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
    for (double step = 0.1; step <= 1.0; step += 0.1) {  // Start from a small positive step
        for (double alpha = std::max(0.0, 0.5 - step); alpha <= std::min(1.0, 0.5 + step); alpha += step) {
            for (double beta = std::max(0.0, 0.5 - step); beta <= std::min(1.0, 0.5 + step); beta += step) {
                geometry_msgs::Pose target_pose = interpolate(alpha, beta);

                groupAuxArmTorsoPtr_->setPoseTarget(target_pose);

                geometry_msgs::PoseStamped target_pose_stamped;
                target_pose_stamped.header.frame_id = "base_footprint"; // Change to your reference frame
                target_pose_stamped.header.stamp = ros::Time::now();
                target_pose_stamped.pose = target_pose;
                target_pose_pub_.publish(target_pose_stamped);

                moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                success = (groupAuxArmTorsoPtr_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                ROS_INFO("Alpha: %f, Beta: %f", alpha, beta);
                ROS_INFO("SUCCESS: %d", success);
                ROS_INFO("Target pose: x=%f, y=%f, z=%f", target_pose.position.x, target_pose.position.y, target_pose.position.z);
                
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

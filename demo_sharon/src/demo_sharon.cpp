#include "demo_sharon/demo_sharon.hpp"

namespace demo_sharon
{
    DemoSharon::DemoSharon(ros::NodeHandle nh) : nodeHandle_(nh)
    {
        ROS_INFO("[DemoSharon] Node started.");

        // Need it asynspinner for moveit
        ros::AsyncSpinner spinner(1);
        spinner.start();
        init(); // Tiagos head and torso are at the initial positions


        removeCollisionObjectsPlanningScene();

        float dimensions[3] = {1.0, 0.9, 0.85};
        geometry_msgs::Pose tablePose;
        tablePose.orientation.w = 1.0;
        tablePose.position.x = 0.9;
        tablePose.position.y = 0.0;
        tablePose.position.z = 0.4;

        addTablePlanningScene(dimensions, tablePose);

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
        addSupequadricsPlanningScene();

        ROS_INFO("Planning to move %s to a target pose expressed in %s",groupRightArmTorsoPtr_->getEndEffectorLink().c_str(),  groupRightArmTorsoPtr_->getPlanningFrame().c_str());


        sharon_msgs::ComputeGraspPoses srvGraspingPoses;
        srvGraspingPoses.request.id = 1;
        geometry_msgs::PoseArray graspingPoses;
        groupRightArmTorsoPtr_->setStartStateToCurrentState();

        if (clientComputeGraspPoses_.call(srvGraspingPoses))
        {
            ROS_INFO("[DemoSharon] ComputeGraspPoses: %d", (bool)srvGraspingPoses.response.success);
            graspingPoses = srvGraspingPoses.response.poses;
        }
        ROS_INFO("[DemoSharon] NumberPoses: %d", (int)graspingPoses.poses.size());

        int indexFeasible = -1;
        bool successGoToReaching = goToAFeasibleReachingPose(graspingPoses, indexFeasible);
        
        if(successGoToReaching){
            ROS_INFO("[DemoSharon] Wiii!");

            // Open gripper
            moveGripper(openGripperPositions_, "right");



        }



        ROS_INFO("[DemoSharon] Done!");
    }

    DemoSharon::~DemoSharon()
    {
    }

    void DemoSharon::moveGripper(const float positions[2], std::string name){
        follow_joint_control_client_Ptr auxGripperClient;

        if(name == "right"){
            auxGripperClient = rightGripperClient_;
        }else if(name == "left"){
            auxGripperClient = leftGripperClient_;
        }
        control_msgs::FollowJointTrajectoryGoal gripperGoal;
        ROS_INFO("Setting gripper position: (%f ,%f)", positions[0], positions[1]);
        waypointGripperGoal(name,gripperGoal, positions, 2.0);

        // Sends the command to start the given trajectory now
        gripperGoal.trajectory.header.stamp = ros::Time::now();
        auxGripperClient->sendGoal(gripperGoal);

        // Wait for trajectory execution
        while (!(auxGripperClient->getState().isDone()) && ros::ok())
        {
            ros::Duration(1.0).sleep(); // sleep for 1 seconds
        }
        ROS_INFO("Gripper set to position: (%f, %f)", positions[0], positions[1]);

        
    }

    bool DemoSharon::goToAFeasibleReachingPose(const geometry_msgs::PoseArray &graspingPoses, int &indexFeasible){
        geometry_msgs::Pose reachingPose;
        bool successPlanning = false;
        for (int idx = 0; idx < graspingPoses.poses.size(); idx++)
        {
            ROS_INFO("[DemoSharon] idx: %d", idx);
            ROS_INFO("Grasping Pose[%d]: %f %f %f", idx, graspingPoses.poses[idx].position.x, graspingPoses.poses[idx].position.y, graspingPoses.poses[idx].position.z);

            KDL::Frame frameEndWrtBase;
            tf::poseMsgToKDL(graspingPoses.poses[idx], frameEndWrtBase);
            KDL::Frame frameReachingWrtEnd;
            frameReachingWrtEnd.p[0] = -reachingDistance_-DISTANCE_TOOL_LINK_GRIPPER_LINK;
            KDL::Frame frameReachingWrtBase = frameEndWrtBase * frameReachingWrtEnd;

            tf::poseKDLToMsg(frameReachingWrtBase, reachingPose);
            geometry_msgs::PoseStamped goal_pose;
            goal_pose.header.frame_id = "base_footprint";
            goal_pose.pose = graspingPoses.poses[idx];
            groupRightArmTorsoPtr_->setPoseTarget(reachingPose);
            ROS_INFO("SET POSE TARGET");
            moveit::planning_interface::MoveItErrorCode code = groupRightArmTorsoPtr_->plan(plan_);
            successPlanning = (code == moveit::planning_interface::MoveItErrorCode::SUCCESS);

            // if(groupRightArmTorsoPtr_->plan(plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            //     successPlanning = true;
            // else{
            //     successPlanning = false;
            // }
            // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", successPlanning ? "" : "FAILED");

            if(successPlanning){
                indexFeasible = idx;
                break;
            }
            // ROS_INFO("AQUI");
        }
        if(successPlanning){
            moveit::planning_interface::MoveItErrorCode e = groupRightArmTorsoPtr_->move();
            if (e == moveit::planning_interface::MoveItErrorCode::SUCCESS){
                ROS_INFO("[DemoSharon] Success in moving the robot to the reaching pose.");
                return true;
            }
            else{
                ROS_INFO("[DemoSharon] Error in moving the robot to the reaching pose.");
                return false;
            }
        }
        else{
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

    void DemoSharon::addTablePlanningScene(float dimensions[3], const geometry_msgs::Pose &tablePose)
    {
        ROS_INFO("[DemoSharon] Add table collision objects to the planning scene");
        // Collision object
        moveit_msgs::CollisionObject collisionObject;
        collisionObject.header.frame_id = groupRightArmTorsoPtr_->getPlanningFrame();
        ROS_INFO("[DemoSharon] Planning_frame: %s", groupRightArmTorsoPtr_->getPlanningFrame().c_str());
        shape_msgs::SolidPrimitive table;
        table.type = table.BOX;
        table.dimensions.resize(3);
        table.dimensions[0] = 1.0;
        table.dimensions[1] = 0.9;
        table.dimensions[2] = 0.85;

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

            collisionObject.id = "object_" + std::to_string(i);

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

    void DemoSharon::init()
    {
        ROS_INFO("[DemoSharon] init().");

        ROS_INFO("[DemoSharon] creating clients...");
        clientActivateSuperquadricsComputation_ = nodeHandle_.serviceClient<sharon_msgs::ActivateSupercuadricsComputation>("/grasp_objects/activate_superquadrics_computation");
        clientGetSuperquadrics_ = nodeHandle_.serviceClient<sharon_msgs::GetSuperquadrics>("/grasp_objects/get_superquadrics");
        clientComputeGraspPoses_ = nodeHandle_.serviceClient<sharon_msgs::ComputeGraspPoses>("/grasp_objects/compute_grasp_poses");

        ros::param::get("demo_sharon/reaching_distance", reachingDistance_);
        ros::param::get("demo_sharon/elimit1", elimit1_);
        ros::param::get("demo_sharon/elimit2", elimit2_);
        ros::param::get("demo_sharon/inflate_size", inflateSize_);

        ROS_INFO("[DemoSharon] demo_sharon/reaching_distance set to %f", reachingDistance_);

        groupRightArmTorsoPtr_ = new moveit::planning_interface::MoveGroupInterface(nameTorsoRightArmGroup_);
        groupLeftArmTorsoPtr_ = new moveit::planning_interface::MoveGroupInterface(nameTorsoLeftArmGroup_);
        groupRightArmTorsoPtr_->setPlanningTime(1.0);
        groupRightArmTorsoPtr_->setPlannerId("SBLkConfigDefault");
        groupRightArmTorsoPtr_->setPoseReferenceFrame("base_footprint");
        groupRightArmTorsoPtr_->setMaxVelocityScalingFactor(1.0);
        createClient(headClient_, std::string("head"));
        createClient(torsoClient_, std::string("torso"));
        createClient(rightGripperClient_, std::string("gripper_right"));

        control_msgs::FollowJointTrajectoryGoal headGoal;
        control_msgs::FollowJointTrajectoryGoal torsoGoal;


        float initHeadPositions[2] = {0, -0.4};
        ROS_INFO("Setting head to init position: (%f ,%f)", initHeadPositions[0], initHeadPositions[1]);

        waypointHeadGoal(headGoal, initHeadPositions, 3.0);

        // Sends the command to start the given trajectory now
        headGoal.trajectory.header.stamp = ros::Time::now();
        headClient_->sendGoal(headGoal);

        // Wait for trajectory execution
        while (!(headClient_->getState().isDone()) && ros::ok())
        {
            ros::Duration(1.0).sleep(); // sleep for 1 seconds
        }
        ROS_INFO("Head set to position: (%f, %f)", initHeadPositions[0], initHeadPositions[1]);
        float initTorsoPosition = 0.3;
        ROS_INFO("Setting torso to init position: (%f)", initTorsoPosition);

        waypointTorsoGoal(torsoGoal, initTorsoPosition, 3.0);

        // Sends the command to start the given trajectory now
        torsoGoal.trajectory.header.stamp = ros::Time::now();
        torsoClient_->sendGoal(torsoGoal);

        // Wait for trajectory execution
        while (!(torsoClient_->getState().isDone()) && ros::ok())
        {
            ros::Duration(1.0).sleep(); // sleep for 1 seconds
        }

        ROS_INFO("Torso set to position: (%f)", initTorsoPosition);

        return;
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
        while (!actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations)
        {
            ROS_DEBUG("Waiting for the %s_controller_action server to come up", name.c_str());
            ++iterations;
        }

        if (iterations == max_iterations)
            ROS_ERROR("createClient: %s controller action server not available", name.c_str());
    }

    // Generates a waypoint to move TIAGo's head
    void DemoSharon::waypointHeadGoal(control_msgs::FollowJointTrajectoryGoal &goal, const float positions[2], const float &timeToReach)
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
        goal.trajectory.points[index].time_from_start = ros::Duration(timeToReach);
    }

    void DemoSharon::waypointGripperGoal(std::string name,control_msgs::FollowJointTrajectoryGoal &goal, const float positions[2], const float &timeToReach){
        // The joint names, which apply to all waypoints
        std::string right_finger = "gripper_"+name+"_right_finger_joint";
        std::string left_finger = "gripper_"+name+"_left_finger_joint";

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

}
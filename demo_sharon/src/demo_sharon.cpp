#include "demo_sharon/demo_sharon.hpp"

namespace demo_sharon
{
    DemoSharon::DemoSharon(ros::NodeHandle nh) : nodeHandle_(nh)
    {
        ROS_INFO("[DemoSharon] Node started.");
        init(); // Tiagos head and torso are at the initial positions
        
        // Start computation of the superquadrics from the pointcloud
        activateSuperquadricsComputation(true);
        ros::Duration(2.0).sleep(); // sleep for 2 seconds
        
        // Stop computation of the superquadrics from the pointcloud. Our objects don't move, so there is no need to
        // continuously recompute the superquadrics
        activateSuperquadricsComputation(false);
        ros::Duration(2.0).sleep(); // sleep for 2 seconds

        // Get the previously computed superquadrics
        if (!getSuperquadrics()){ // If it's empty, there is no objects to grasp
            return;
        }

        ROS_INFO("[DemoSharon] We have %d supequadrics.", superquadricsMsg_.superquadrics.size());



    }

    DemoSharon::~DemoSharon()
    {
    }

    void DemoSharon::init()
    {
        ROS_INFO("[DemoSharon] init().");

        ROS_INFO("[DemoSharon] creating clients...");
        clientActivateSuperquadricsComputation_ = nodeHandle_.serviceClient<sharon_msgs::ActivateSupercuadricsComputation>("/grasp_objects/activate_superquadrics_computation");
        clientGetSuperquadrics_ = nodeHandle_.serviceClient<sharon_msgs::GetSuperquadrics>("/grasp_objects/get_superquadrics");

        createClient(headClient_, std::string("head"));
        createClient(torsoClient_, std::string("torso"));

        control_msgs::FollowJointTrajectoryGoal headGoal;
        control_msgs::FollowJointTrajectoryGoal torsoGoal;

        float initHeadPositions[2] = {0,-0.4};
        ROS_INFO("Setting head to init position: (%f ,%f)", initHeadPositions[0],  initHeadPositions[1]);

        waypointHeadGoal(headGoal,initHeadPositions, 5.0);

        // Sends the command to start the given trajectory now
        headGoal.trajectory.header.stamp = ros::Time::now();
        headClient_->sendGoal(headGoal);

        // Wait for trajectory execution
        while(!(headClient_->getState().isDone()) && ros::ok())
        {
            ros::Duration(1.0).sleep(); // sleep for 1 seconds
        }
        ROS_INFO("Head set to position: (%f, %f)", initHeadPositions[0],  initHeadPositions[1]);
        float initTorsoPosition = 0.3;
        ROS_INFO("Setting torso to init position: (%f)", initTorsoPosition);

        waypointTorsoGoal(torsoGoal, initTorsoPosition, 5.0);

        // Sends the command to start the given trajectory now
        torsoGoal.trajectory.header.stamp = ros::Time::now();
        torsoClient_->sendGoal(torsoGoal);

        // Wait for trajectory execution
        while(!(torsoClient_->getState().isDone()) && ros::ok())
        {
            ros::Duration(1.0).sleep(); // sleep for 1 seconds
        }

        ROS_INFO("Torso set to position: (%f)", initTorsoPosition);


        return;

    }

    bool DemoSharon::getSuperquadrics(){
        sharon_msgs::GetSuperquadrics srvSq;
        
        if(clientGetSuperquadrics_.call(srvSq)){
            superquadricsMsg_ = srvSq.response.superquadrics;
            if(superquadricsMsg_.superquadrics.size() != 0){
                return true;
            }
            else{
                return false;
            }
        }
    }

    void DemoSharon::activateSuperquadricsComputation(bool activate){
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

        std::string controller_name = name+"_controller/follow_joint_trajectory";

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
    void DemoSharon::waypointHeadGoal(control_msgs::FollowJointTrajectoryGoal &goal,const float positions[2],const float &timeToReach)
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
    void DemoSharon::waypointTorsoGoal(control_msgs::FollowJointTrajectoryGoal &goal,const float &position,const float &timeToReach)
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



}
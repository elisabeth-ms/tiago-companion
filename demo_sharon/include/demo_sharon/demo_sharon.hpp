#include <exception>
#include <string>

// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/topic.h>

// Moveit headers
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// KDL
#include <kdl_conversions/kdl_msg.h>

// Sharon headers

#include "sharon_msgs/SuperquadricMultiArray.h"
#include "sharon_msgs/ActivateSupercuadricsComputation.h"
#include "sharon_msgs/GetSuperquadrics.h"
#include "sharon_msgs/ComputeGraspPoses.h"

// Action interface type for moving TIAGo, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> follow_joint_control_client;
typedef boost::shared_ptr< follow_joint_control_client>  follow_joint_control_client_Ptr;

// TODO: ADD AS ROS PARAMS
#define DISTANCE_TOOL_LINK_GRIPPER_LINK 0.151

namespace demo_sharon{


    class DemoSharon{
        public:
        
        explicit DemoSharon(ros::NodeHandle nh);

        ~DemoSharon();

        void init();

        void createClient(follow_joint_control_client_Ptr &actionClient, std::string name);
        
        void waypointHeadGoal(control_msgs::FollowJointTrajectoryGoal &goal,const float positions[2],const float &timeToReach);

        void waypointTorsoGoal(control_msgs::FollowJointTrajectoryGoal &goal,const float &position,const float &timeToReach);

        void activateSuperquadricsComputation(bool activate);

        bool getSuperquadrics();

        void addTablePlanningScene(float dimensions[3], const geometry_msgs::Pose &tablePose);

        void removeCollisionObjectsPlanningScene();

        void addSupequadricsPlanningScene();

        bool goToAFeasibleReachingPose(const geometry_msgs::PoseArray &graspingPoses, int &indexFeasible);

        void waypointGripperGoal(std::string name,control_msgs::FollowJointTrajectoryGoal &goal,  const float positions[2], const float &timeToReach);

        void moveGripper(const float positions[2], std::string name);

        private:
        //! ROS node handle.
        ros::NodeHandle nodeHandle_;
        follow_joint_control_client_Ptr headClient_;
        follow_joint_control_client_Ptr torsoClient_;
        follow_joint_control_client_Ptr rightGripperClient_;
        follow_joint_control_client_Ptr leftGripperClient_;


        ros::ServiceClient clientActivateSuperquadricsComputation_; 
        ros::ServiceClient clientComputeGraspPoses_;
        ros::ServiceClient clientGetSuperquadrics_;

        sharon_msgs::SuperquadricMultiArray superquadricsMsg_;
        std::string nameTorsoRightArmGroup_="arm_right_torso";
        std::string nameTorsoLeftArmGroup_="arm_left_torso";;
        
        moveit::planning_interface::MoveGroupInterface * groupRightArmTorsoPtr_;
        moveit::planning_interface::MoveGroupInterface * groupLeftArmTorsoPtr_;

        moveit::planning_interface::PlanningSceneInterface planningSceneInterface_;

        float reachingDistance_;
        float elimit1_;
        float elimit2_;
        float inflateSize_;
        float openGripperPositions_[2] = {0.04, 0.04};

        moveit::planning_interface::MoveGroupInterface::Plan plan_;

};
}
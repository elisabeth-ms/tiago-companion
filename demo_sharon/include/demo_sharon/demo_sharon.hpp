#include <exception>
#include <string>

// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/topic.h>

// Sharon headers

#include "sharon_msgs/SuperquadricMultiArray.h"
#include "sharon_msgs/ActivateSupercuadricsComputation.h"
#include "sharon_msgs/GetSuperquadrics.h"
#include "sharon_msgs/ComputeGraspPoses.h"

// Action interface type for moving TIAGo, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> follow_joint_control_client;
typedef boost::shared_ptr< follow_joint_control_client>  follow_joint_control_client_Ptr;

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


        private:
        //! ROS node handle.
        ros::NodeHandle nodeHandle_;
        follow_joint_control_client_Ptr headClient_;
        follow_joint_control_client_Ptr torsoClient_;

        ros::ServiceClient clientActivateSuperquadricsComputation_; 
        ros::ServiceClient clientComputeGraspPoses_;
        ros::ServiceClient clientGetSuperquadrics_;

        sharon_msgs::SuperquadricMultiArray superquadricsMsg_;




};
}
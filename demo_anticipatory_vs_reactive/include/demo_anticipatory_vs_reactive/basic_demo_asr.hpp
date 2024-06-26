#include <exception>
#include <string>

// Boost headers
#include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp>
#include <sys/stat.h>
#include <boost/filesystem.hpp>

// ROS headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>
#include <ros/topic.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>

// Moveit headers
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/MoveGroupAction.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>
#include <actionlib/client/simple_action_client.h>

// KDL
#include <kdl_conversions/kdl_msg.h>

// Sharon headers

#include "companion_msgs/SuperquadricMultiArray.h"
#include "companion_msgs/ActivateSupercuadricsComputation.h"
#include "companion_msgs/GetSuperquadrics.h"
#include "companion_msgs/ComputeGraspPoses.h"
#include "companion_msgs/BoundingBoxes.h"
#include "companion_msgs/GetBboxes.h"
#include "companion_msgs/GlassesData.h"

// Speech
#include <actionlib/client/simple_action_client.h>
#include <pal_interaction_msgs/TtsAction.h>
#include <pal_interaction_msgs/TtsGoal.h>

// Asr
#include "companion_msgs/ActivateASR.h"

// darknet_ros
#include "darknet_ros_msgs/BoundingBoxes.h"

// STD HEADERS
#include <mutex>

#include <pthread.h>

// write && read csv files
#include <iostream>
#include <fstream>
#include <experimental/filesystem> 

namespace fs = std::experimental::filesystem;


#include <ros/package.h>

// Action interface type for moving TIAGo, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> follow_joint_control_client;
typedef boost::shared_ptr<follow_joint_control_client> follow_joint_control_client_Ptr;

// TODO: ADD AS ROS PARAMS
#define DISTANCE_TOOL_LINK_GRIPPER_LINK 0.185
#define INITIALIZE 0
#define WAIT_FOR_COMMAND 1
#define COMPUTE_GRASP_POSES 2
#define FIND_REACHING_GRASP_IK 3
#define PLAN_TO_REACHING_JOINTS 4
#define EXECUTE_PLAN_TO_REACHING_JOINTS 5
#define OPEN_GRIPPER 6
#define APROACH_TO_GRASP 7
#define CLOSE_GRIPPER 8
#define GO_UP 9
#define RELEASE_OBJECT 10
#define OBJECT_DELIVERED 11
#define ROBOT_IN_HOME_POSITION 12
#define BRING_CLOSER 15
#define GO_BACKWARDS 16
#define TURN_ANTICLOCKWISE 17
#define UNABLE_TO_REACHING_GRASP_IK -2
#define DEBUG_STATE -10
#define WAIT_TO_EXECUTE 13
#define NEW_GAZE 14
namespace demo_anticipatory_vs_reactive
{
    struct SqCategory
    {
        int idSq;
        std::string category;
    };

    struct TrajectoryToGraspObject
    {
        std::string category;
        int idSq;
        std::string arm; // "right" or "left"
        geometry_msgs::Pose goalReachPose;
        std::vector<double> goalReachJointValues;
        geometry_msgs::Pose goalGraspPose;
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        float closeGripperPositions[2] = {0.0, 0.0};
    };

    class BasicDemoAsr
    {

    public:
        explicit BasicDemoAsr(ros::NodeHandle nh);

        ~BasicDemoAsr();

        void init();

        void createClient(follow_joint_control_client_Ptr &actionClient, std::string name);

        void waypointHeadGoal(control_msgs::FollowJointTrajectoryGoal &goal, const std::vector<float> &positions, const float &timeToReach);

        void waypointTorsoGoal(control_msgs::FollowJointTrajectoryGoal &goal, const float &position, const float &timeToReach);

        void activateSuperquadricsComputation(bool activate);

        void activateDetectHitFt(bool activate);

        bool getSuperquadrics();

        void addTablePlanningScene(const std::vector<float> &dimensions, const geometry_msgs::Pose &tablePose, const std::string &id);

        void removeCollisionObjectsPlanningScene();

        void addSupequadricsPlanningScene();

        bool goToAFeasibleReachingPose(const geometry_msgs::PoseArray &graspingPoses, int &indexFeasible);

        void waypointGripperGoal(std::string name, control_msgs::FollowJointTrajectoryGoal &goal, const float positions[2], const float &timeToReach);

        void moveGripper(const float positions[2], std::string name);

        bool goToGraspingPose(const geometry_msgs::Pose &graspingPose);

        bool goUp(moveit::planning_interface::MoveGroupInterface *groupArmTorsoPtr, float upDistance);

        bool bringCloser(moveit::planning_interface::MoveGroupInterface *groupArmTorsoPtr, float bringCloserDistance);
        
        bool initDemo(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
        
        bool releaseGripper(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

        bool moveToHomePosition(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

        bool getBoundingBoxesFromSupercuadrics();

        void asrCallback(const std_msgs::StringConstPtr &asrMsg);

        bool computeIntersectionOverUnion(const std::array<int, 4> &bboxYolo, const std::array<int, 4> &bboxSq, float &IoU);

        void initializeHeadPosition(const std::vector<float> &initHeadPositions);

        void initializeTorsoPosition(float initTorsoPosition, float executionTime);

        bool initializeRightArmPosition(const std::vector<double> &initRightArmPositions);

        bool initializeLeftArmPosition(const std::vector<double> &initLeftArmPositions);

        void demoASR();

        void *computeGraspPosesThread(void *ptr);
        static void *sendcomputeGraspPosesThreadWrapper(void *object);

        void *findReachGraspIKThread(void *ptr);
        static void *sendFindReachGraspIKThreadWrapper(void *object);

        static void *sendPlanToReachJointsThreadWrapper(void *object);
        void *planToReachJointsThread(void *ptr);

        bool goalReached(moveit::planning_interface::MoveGroupInterface *&groupArmTorsoPtr_);

        void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &amclPoseMsg);

        void stopDemoCallback(const std_msgs::EmptyConstPtr & stop);

        void okDemoCallback(const std_msgs::EmptyConstPtr &msg);
    
    private:
        //! ROS node handle.
        //  moveit_visual_tools::MoveItVisualToolsPtr visualTools_;
        ros::NodeHandle nodeHandle_;

        int state_;
        follow_joint_control_client_Ptr headClient_;
        follow_joint_control_client_Ptr torsoClient_;
        follow_joint_control_client_Ptr rightGripperClient_;
        follow_joint_control_client_Ptr leftGripperClient_;
        follow_joint_control_client_Ptr rightArmClient_;
        follow_joint_control_client_Ptr torsoRightArmClient_;
        follow_joint_control_client_Ptr leftArmClient_;

        ros::ServiceClient clientActivateSuperquadricsComputation_;
        ros::ServiceClient clientActivateDetectHit_;
        ros::ServiceClient clientActivateAsr_;

        ros::ServiceClient clientComputeGraspPoses_;
        ros::ServiceClient clientGetSuperquadrics_;
        ros::ServiceClient clientGetBboxesSuperquadrics_;

        ros::ServiceServer serviceReleaseGripper_;
        ros::ServiceServer serviceInitDemo_;
        ros::ServiceServer serviceMoveToHomePosition_;
        ros::Subscriber asrSubscriber_;
        ros::Subscriber glassesDataSubscriber_;
        ros::Subscriber moveGroupStatusSubscriber_;
        ros::Subscriber amclPoseSubscriber_;
        ros::Subscriber stopDemoSubscriber_;
        ros::Subscriber okDemoSubscriber_;


        ros::Publisher statePublisher_;
        ros::Publisher superquadricsBBoxesPublisher_;
        ros::Publisher reachingPosePublisher_;
        ros::Publisher planPublisher_;
        ros::Publisher cmdVelPublisher_;

        geometry_msgs::Pose basePose_;
        actionlib::SimpleActionClient<pal_interaction_msgs::TtsAction> *acPtr_;

        bool releaseGripper_ = false;
        bool moveToHomePosition_ = false;
        bool initDemo_ = false;
        companion_msgs::BoundingBoxes bboxesMsg_;
        darknet_ros_msgs::BoundingBoxes yoloBBoxesMsg_;
        geometry_msgs::Pose reachingPose_;

        companion_msgs::SuperquadricMultiArray superquadricsMsg_;
        std::string nameTorsoRightArmGroup_ = "arm_right_torso";
        std::string nameRightArmGroup_ = "arm_right";
        std::string nameTorsoLeftArmGroup_ = "arm_left_torso";
        std::string nameLeftArmGroup_ = "arm_left";

        moveit::planning_interface::MoveGroupInterface *groupRightArmTorsoPtr_;
        moveit::planning_interface::MoveGroupInterface *groupRightArmPtr_;
        moveit::planning_interface::MoveGroupInterface *groupLeftArmTorsoPtr_;
        moveit::planning_interface::MoveGroupInterface *groupLeftArmPtr_;

        moveit::planning_interface::PlanningSceneInterface planningSceneInterface_;

        float reachingDistance_;
        float elimit1_;
        float elimit2_;
        float inflateSize_;
        bool useAsr_;
        bool waitingForAsrCommand_;
        bool waitingForGlassesCommand_;

        bool successPlanning_;

        bool asrCommandReceived_;
        float thresholdPlanTrajectory_;
        float thresholdExecuteTrajectory_;
        double goalJointTolerance_;
        int userId_;
        bool stopDemo_;
        bool okDemo_;
        std::string initVerbalMessage_;
        std::string passObjectVerbalMessage_;

        std::string arm_;

        float openGripperPositions_[2] = {0.07, 0.07};
        float closeGripperPositions_[2] = {0.013, 0.013};
        float maxTorsoPosition_;
        float maxErrorJoints_;
        std::vector<float> initHeadPositions_;
        float initTorsoPosition_;
        std::vector<float> tableDimensions_;
        std::vector<float> tablePosition_;

        std::vector<float> tableDimensions2_;
        std::vector<float> tablePosition2_;

        std::vector<float> tableDimensions3_;
        std::vector<float> tablePosition3_;

        std::vector<double> initRightArmPositions_;
        std::vector<double> initLeftArmPositions_;

        std::string asr_;
        std::vector<SqCategory> sqCategories_;
        bool foundAsr_ = false;
        bool foundAsrDifferent_ = false;
        bool foundGlasses_ = false;
        std::string glassesCategory_;
        std::string prevGlassesCategory_;
        float currentDecisionProb_;

        int indexSqCategory_ = -1;
        int indexGlassesSqCategory_ = -1;
        int indexSqCategoryAsr_ = -1;

        geometry_msgs::PoseArray graspingPoses_;
        std::vector<float> width_;
        std::vector<double> reachJointValues_;
        std::vector<double> goalJoints_;

        bool foundReachIk_;
        bool stopMotion_;
        int indexGraspingPose_;

        float closeLeftGripperDeviation_[2] = {0.033, 0.033};
        float closeRightGripperDeviation_[2] = {0.033, 0.027};

        float closeMorePosition = 0.03;

        bool greaterThanExecutionThreshold_;

        moveit::planning_interface::MoveGroupInterface::Plan plan_;

        robot_model::RobotModelPtr kinematicModel_;
        robot_state::RobotStatePtr kinematicState_;
        const robot_state::JointModelGroup *jointModelGroupTorsoRightArm_;
        const robot_state::JointModelGroup *jointModelGroupTorsoLeftArm_;

        std::mutex mtxASR_;
        pthread_t threadComputeGraspPoses_;
        pthread_t threadFindReachGraspIK_;
        pthread_t threadPlanToReachJoints_;

        std::vector<TrajectoryToGraspObject> listTrajectoriesToGraspObjects;

        bool alreadyAvailable_;
        int indexListTrajectories_;

        ros::Time gazeCommandTime_;
        ros::Time startDemoTime_;
        ros::Time computeGraspPosesTime_;
        ros::Time feasibleReachingPoseTime_;
        ros::Time planTrajectoryReachingPoseTime_;
        ros::Time startExecutionTrajectoryTime_;
        ros::Time decisisionProbGreaterExecuteThreshold_;
        ros::Time reachingPoseTime_;
        ros::Time asrTime_;
        ros::Time objectGrasppedTime_;
        ros::Time objectUpTime_;
        ros::Time bringCloserTime_;
        ros::Time goBackwardsStartTime_;
        ros::Time goBackwardsEndTime_;
        ros::Time turnAnticlockwiseStartTime_;
        ros::Time objectDeliveredTime_;
        bool firstInState;

        std::ofstream timesFile_;
        std::mutex mtxWriteFile_;
    };
}

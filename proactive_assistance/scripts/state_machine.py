#!/usr/bin/env python
import rospy
import rospkg
import yaml

import smach
import smach_ros
import actionlib
from proactive_assistance.msg import NavigateWaypointAction, NavigateWaypointGoal, NewWaypointAction, NewWaypointGoal, NewWaypointResult
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from control_msgs.msg import PointHeadAction, PointHeadGoal
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from companion_msgs.msg import TaskNavigationGoal, BoundingBoxesLabels
from companion_msgs.srv import ActivateSupercuadricsComputation, ActivateSupercuadricsComputationRequest, GetSuperquadrics, GetSuperquadricsRequest, GetSuperquadricsResponse
from companion_msgs.srv import GetBboxes, GetBboxesRequest, GetBboxesResponse, ComputeGraspPoses, ComputeGraspPosesRequest
from proactive_assistance.msg import ObjectManipulationAction, ObjectManipulationGoal
from geometry_msgs.msg import Pose, PoseArray, Point
from shape_msgs.msg import SolidPrimitive


def find_package_path(package_name):
    rospack = rospkg.RosPack()
    try:
        package_path = rospack.get_path(package_name)
        return package_path
    except rospkg.ResourceNotFound:
        return None


def load_config(config_file):
    print("config_file: ", config_file)
    with open(config_file, 'r') as file:
        config = yaml.safe_load(file)
    return config


class InitializeDemoState(smach.State):
    def __init__(self, config):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.head_client = actionlib.SimpleActionClient('/head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.torso_client = actionlib.SimpleActionClient('/torso_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.config = config
        rospy.loginfo("Waiting for head and torso action servers...")
        self.head_client.wait_for_server()
        self.torso_client.wait_for_server()
        rospy.loginfo("Head and torso action servers are up.")

    def execute(self, userdata):
        rospy.loginfo("Initializing demo state...")

        # Torso position goal
        torso_goal = FollowJointTrajectoryGoal()
        torso_trajectory = JointTrajectory()
        torso_trajectory.joint_names = ['torso_lift_joint']
        torso_point = JointTrajectoryPoint()
        initial_state = self.config['initial_state']
        torso_point.positions = [initial_state['torso_controller']['height']]  
        torso_point.time_from_start = rospy.Duration(3.0)
        torso_trajectory.points.append(torso_point)
        torso_goal.trajectory = torso_trajectory
        
        head_goal = FollowJointTrajectoryGoal()
        head_trajectory = JointTrajectory()
        head_trajectory.joint_names = ['head_1_joint', 'head_2_joint']
        
        head_point = JointTrajectoryPoint()
        head_point.positions = [initial_state['head_controller']['pan'], initial_state['head_controller']['tilt']]
        head_point.time_from_start = rospy.Duration(2.0)
        head_trajectory.points.append(head_point)
        head_goal.trajectory = head_trajectory



        self.torso_client.send_goal(torso_goal)
        self.head_client.send_goal(head_goal)

        self.torso_client.wait_for_result()
        self.head_client.wait_for_result()

        if self.torso_client.get_state() == actionlib.GoalStatus.SUCCEEDED and self.head_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Robot initialized to starting state.")
            return 'succeeded'
        else:
            rospy.logwarn("Failed to initialize robot to starting state.")
            return 'aborted'

class WaitToPick(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['new_goal', 'aborted', 'preempted'],
                             output_keys=['task_navigation_goal'])
        self.goal_pose = PoseStamped()
        self.goal_received = False
        self.goal_subscriber = rospy.Subscriber('/proactive_assistance/task_navigation_goal', TaskNavigationGoal, self.goal_cb)
        # self._goal_update_client = actionlib.SimpleActionClient('goal_update', NewWaypointAction)
        self.task_navigation_goal = TaskNavigationGoal()
        


    def goal_cb(self, msg):
        rospy.loginfo("Received new task navigation goal.")
        self.task_navigation_goal = msg
        self.goal_received = True


    def execute(self, userdata):
        rospy.loginfo("WaitToPick")
        rospy.loginfo('Waiting for goal...')
        while not self.goal_received:
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            # self._goal_update_client.send_goal(NewWaypointGoal())
            # if self._goal_update_client.wait_for_result(rospy.Duration(0.5)):
            #     new_goal_result = self._goal_update_client.get_result()
            #     if new_goal_result.result == "new_goal":
            #         self.goal_pose = new_goal_result.target_pose
            #         print("new_goal: ", self.goal_pose)
            rospy.sleep(0.1)
        if self.goal_received:
            self.goal_received = False
            userdata.task_navigation_goal = self.task_navigation_goal
            return 'new_goal'

class NavToPick(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['goal_reached', 'goal_unreachable', 'update_goal', 'preempted', 'aborted'],
                             input_keys=['task_navigation_goal'], output_keys=['task_navigation_goal'])
        self._ac = actionlib.SimpleActionClient('navigate_waypoint', NavigateWaypointAction)
        
    def execute(self, userdata):
        rospy.loginfo('Navigating to goal...')
        self._ac.wait_for_server()

        goal = NavigateWaypointGoal()
        goal.object_name = userdata.task_navigation_goal.object_name
        goal.target_pose = userdata.task_navigation_goal.target_pose
        # self._ac.send_goal(goal, feedback_cb=self.feedback_cb)
        self._ac.send_goal(goal)

        state = self._ac.wait_for_result()

        print("State: ", state)
        result = self._ac.get_result()
        print("Result: ", result)
        print("Result: ", result.result)
        
        # if result.result == "new_goal":
        #     print("new_goal")
        #     userdata.task_navigation_goal.target_pose = result.updated_goal
        #     return 'update_goal'
        if result.result == 'succeeded':
            print("succeeded")
            return 'goal_reached'
        elif result.result == 'failed':
            print("aborted")
            return 'goal_unreachable'
        elif result.result == "preempted":
            print("preempted")
            return 'preempted'

    # def feedback_cb(self, feedback):
        # Simulating a condition to update the goal
        # if rospy.Time.now().to_sec() % 10 < 1:  # Change condition as needed
        #     rospy.loginfo('Goal update required.')
        #     self._ac.cancel_goal()
        #     return 'update_required'

class ObjectBasedGoal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['goal_updated', 'goal_not_updated', 'close_enough'],
                             input_keys=['task_navigation_goal'], output_keys=['task_navigation_goal'])
        self._goal_update_client = actionlib.SimpleActionClient('goal_update', NewWaypointAction)
        self._goal_update_client.wait_for_server()
        self.need_update = True

    def execute(self, userdata):
        rospy.loginfo('ObjectBasedGoal: Executing goal update check...')
        
        rate = rospy.Rate(10)  # 1 Hz
            # Call the goal_update action server
        goal = NewWaypointGoal()
        goal.object_name = userdata.task_navigation_goal.object_name
        self._goal_update_client.send_goal(goal)
            
        # Wait for result with a timeout
        if self._goal_update_client.wait_for_result(rospy.Duration(1.0)):
            result = self._goal_update_client.get_result()
            print("result: ", result.result)
            print("result current_pose: ", result.current_pose)
            
            if result.result == "new_goal":
                new_goal_pose = result.target_pose

                # Check if the goal is far enough to the previous navigation goal
                rospy.loginfo('ObjectBasedGoal: Checking if the new goal is far enough...')
                if not self.is_close_enough(userdata.task_navigation_goal.target_pose, new_goal_pose, 0.25) or self.need_update:
                    rospy.loginfo('ObjectBasedGoal: Navigation goal updated.')
                    userdata.task_navigation_goal.target_pose = new_goal_pose
                    self.need_update = False
                    return 'goal_updated'
                rospy.loginfo('ObjectBasedGoal: No need to update the goal.')
                print("goal_pose: ", userdata.task_navigation_goal.target_pose)
                print("new_goal_pose: ", new_goal_pose)
                if not self.need_update and self.is_close_enough(userdata.task_navigation_goal.target_pose, result.current_pose, 0.25):
                    rospy.loginfo('ObjectBasedGoal: Close enough to the new goal.')
                    return 'close_enough'
                else:
                    rospy.loginfo('ObjectBasedGoal: Not close enough to the new goal. Goal not updated.')
                    return 'goal_not_updated'
            else:
                rospy.loginfo('ObjectBasedGoal: No new goal.')
                if self.is_close_enough(userdata.task_navigation_goal.target_pose, result.current_pose, 0.25):
                    return 'close_enough'
                else:
                    return 'goal_not_updated'


    def is_close_enough(self, current_pose, new_pose, distance_threshold):
        print("Inside is_close_enough")
        print("Position1: ", current_pose.pose.position)
        print("Position2: ", new_pose.pose.position)
        distance = self.calculate_distance(current_pose.pose.position, new_pose.pose.position)
        rospy.loginfo('ObjectBasedGoal: Distance: %f', distance)
        return distance <= distance_threshold

    def calculate_distance(self, position1, position2):
        return ((position1.x - position2.x) ** 2 + 
                (position1.y - position2.y) ** 2 + 
                (position1.z - position2.z) ** 2) ** 0.5

class MoveHeadState(smach.State):
    def __init__(self, head_pan, head_tilt):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.head_client = actionlib.SimpleActionClient('/head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

        rospy.loginfo("Waiting for head action server...")
        self.head_client.wait_for_server()
        rospy.loginfo("Head action server is up.")
        self.head_pan = head_pan
        self.head_tilt = head_tilt

    def execute(self, userdata):
        rospy.loginfo("Moving head to the desired position...")
        
        head_goal = FollowJointTrajectoryGoal()
        head_trajectory = JointTrajectory()
        head_trajectory.joint_names = ['head_1_joint', 'head_2_joint']
        
        head_point = JointTrajectoryPoint()
        head_point.positions = [self.head_pan, self.head_tilt]
        head_point.time_from_start = rospy.Duration(2.0)
        head_trajectory.points.append(head_point)
        head_goal.trajectory = head_trajectory



        self.head_client.send_goal(head_goal)

        self.head_client.wait_for_result()

        if  self.head_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Head moved to the desired position.")
            return 'succeeded'
        else:
            rospy.logwarn("Failed to move head to the desired position.")
            return 'aborted'



class IdentifyObjectsState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['object_identified', 'object_not_identified'],
                                   input_keys=['task_navigation_goal'],
                                   output_keys=['task_navigation_goal', 'superquadrics', 'object_superquadric_id'])
        
        # Two services to call for object shape and pose identification using python client
        self.client_activate_object_shape_computation_ =rospy.ServiceProxy("/grasp_objects/activate_superquadrics_computation", ActivateSupercuadricsComputation)
        # rospy.wait_for_service('/grasp_objects/activate_object_shape_computation')
        self.client_get_superquadrics_ = rospy.ServiceProxy("/grasp_objects/get_superquadrics", GetSuperquadrics)
        
        self.client_get_bboxes_ = rospy.ServiceProxy("/grasp_objects/get_bboxes_superquadrics", GetBboxes)
        # rospy.wait_for_service('/grasp_objects/get_superquadrics')
        
        # wait for a message to be published

    def execute(self, userdata):
        # Identify objects logic here
        rospy.loginfo('Identifying objects...')
        boundingBoxes_msg = rospy.wait_for_message("/xtion/bounding_boxes", BoundingBoxesLabels)
        print("boundingBoxes_msg: ", boundingBoxes_msg)
        
        object_name = userdata.task_navigation_goal.object_name
        object_index_object_detection = -1
        # find the object bounding box in the message
        for i in range(len(boundingBoxes_msg.classes)):
            if boundingBoxes_msg.classes[i] == object_name:
                object_index_object_detection = i
                break
        boundingBoxes_msg.classes 
        
        activateRequest = ActivateSupercuadricsComputationRequest()
        activateRequest.activate = True
        self.client_activate_object_shape_computation_.call(activateRequest)
        
        srvSq = GetSuperquadricsRequest()
        superquadrics_msg = self.client_get_superquadrics_.call(srvSq)
        print("superquadrics_msg: ", superquadrics_msg)
        
        tlx_bbox = boundingBoxes_msg.bounding_boxes[object_index_object_detection].tlx
        tly_bbox = boundingBoxes_msg.bounding_boxes[object_index_object_detection].tly
        brx_bbox = boundingBoxes_msg.bounding_boxes[object_index_object_detection].brx
        bry_bbox = boundingBoxes_msg.bounding_boxes[object_index_object_detection].bry
        
        if len(superquadrics_msg.superquadrics.superquadrics) != 0:
            rospy.loginfo('Objects identified.')
            bboxes = self.client_get_bboxes_.call(GetBboxesRequest())
            print("bboxes: ", bboxes)
            max_iou = 0
            for i in range(len(bboxes.bounding_boxes.bounding_boxes)):
                iou = self.calculate_iou(tlx_bbox, tly_bbox, brx_bbox, bry_bbox, bboxes.bounding_boxes.bounding_boxes[i].tlx,
                              bboxes.bounding_boxes.bounding_boxes[i].tly, bboxes.bounding_boxes.bounding_boxes[i].brx,
                              bboxes.bounding_boxes.bounding_boxes[i].bry)
                if iou>max_iou:
                    max_iou = iou
                    userdata.object_superquadric_id = bboxes.bounding_boxes.bounding_boxes[i].id
                    userdata.superquadrics = superquadrics_msg.superquadrics
            if max_iou > 0.0:
                return 'object_identified'
            return 'object_identified'
        else:
            rospy.loginfo('No objects identified.')
            return 'object_not_identified'
          
    def calculate_iou(self, tlx_ref, tly_ref, brx_ref, bry_ref, tlx_pred, tly_pred, brx_pred, bry_pred):
        # Coordinates of the intersection rectangle
        xA = max(tlx_ref, tlx_pred)
        yA = max(tly_ref, tly_pred)
        xB = min(brx_ref, brx_pred)
        yB = min(bry_ref, bry_pred)

        # Compute the area of intersection rectangle
        interWidth = max(0, xB - xA)
        interHeight = max(0, yB - yA)
        interArea = interWidth * interHeight

        # If there is no intersection, the intersection area will be zero
        if interArea == 0:
            return 0.0

        # Compute the area of both bounding boxes
        area_ref = (brx_ref - tlx_ref) * (bry_ref - tly_ref)
        area_pred = (brx_pred - tlx_pred) * (bry_pred - tly_pred)

        # Compute the union area
        unionArea = area_ref + area_pred - interArea

        # Compute IoU
        iou = interArea / float(unionArea)

        return iou

class GraspObjectState(smach.State):
    def __init__(self, config):
        smach.State.__init__(self, outcomes=['object_grasped', 'object_not_grasped'],
                                   input_keys=['task_navigation_goal', 'superquadrics', 'object_superquadric_id'],
                                   output_keys=['task_navigation_goal', 'manipulation_goal'])
        self.config = config
        self.client_get_grasp_poses = rospy.ServiceProxy("/grasp_objects/compute_grasp_poses", ComputeGraspPoses)
        self.client_object_manipulation = actionlib.SimpleActionClient('object_manipulation', ObjectManipulationAction)
        self.client_object_manipulation.wait_for_server()
        
    def execute(self, userdata):
        grasp_poses_request = ComputeGraspPosesRequest()
        grasp_poses_request.id = userdata.object_superquadric_id
        grasp_poses = self.client_get_grasp_poses.call(grasp_poses_request)
        
        print("grasp_poses: ", grasp_poses)
        goal = ObjectManipulationGoal()
        
        index_object = -1
        for i in range(len(userdata.superquadrics.superquadrics)):
            if userdata.superquadrics.superquadrics[i].id == userdata.object_superquadric_id:
                print("found")
                index_object = i
                break
        if grasp_poses.poses !=[]:
            goal.task = 'pick'
            goal.desired_object = userdata.superquadrics.superquadrics[index_object]
            # Define obstacles
            obstacle = SolidPrimitive()
            obstacle.type = SolidPrimitive.BOX
            obstacle.dimensions = self.config['robot_table']['dimensions']
            print("dimensions: ", obstacle.dimensions)
            goal.obstacles.append(obstacle)
            
            obstacle_pose = Pose()
            obstacle_pose.position.x = self.config['robot_table']['pose']['position']['x']
            obstacle_pose.position.y = self.config['robot_table']['pose']['position']['y']
            obstacle_pose.position.z = self.config['robot_table']['pose']['position']['z']
            obstacle_pose.orientation.w = self.config['robot_table']['pose']['orientation']['w']
            goal.obstacle_poses.append(obstacle_pose)  # Assuming same pose for simplicity
            
            goal.reference_frames_of_obstacles.append('base_footprint')
            
            goal.grasping_poses = grasp_poses.poses

            goal.gripper_empty = True
            goal.width = grasp_poses.width
            # Sends the goal to the action server
            self.client_object_manipulation.send_goal(goal, feedback_cb=self.feedback_cb)

            # Waits for the server to finish performing the action
            self.client_object_manipulation.wait_for_result()

            # Prints out the result of executing the action
            result = self.client_object_manipulation.get_result()
            
            userdata.manipulation_goal = goal
            print('[Result] Success: %s, Message: %s' % (result.success, result.message))
            rospy.loginfo('Object grasped.')
            return 'object_grasped'
        else:
            rospy.loginfo('Object not grasped.')
            return 'object_not_grasped'
          
    def feedback_cb(self,feedback):
        print('[Feedback] Current Status: %s' % feedback.status)

class SafetyArmPositionState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['arm_positioned', 'arm_not_positioned'],
                                   input_keys=['task_navigation_goal', 'manipulation_goal'],
                                   output_keys=['task_navigation_goal'])
        self.client_object_manipulation = actionlib.SimpleActionClient('object_manipulation', ObjectManipulationAction)
        
    def execute(self, userdata):
        rospy.loginfo('Move to confortable pose...')
        goal = ObjectManipulationGoal()
        goal.task = 'comfortable'
        goal.gripper_empty = False
        goal.obstacle_poses = userdata.manipulation_goal.obstacle_poses
        goal.obstacles = userdata.manipulation_goal.obstacles
        goal.reference_frames_of_obstacles = userdata.manipulation_goal.reference_frames_of_obstacles
        
        self.client_object_manipulation.send_goal(goal)
        self.client_object_manipulation.wait_for_result()
        result = self.client_object_manipulation.get_result()
        print('[Result] Success: %s, Message: %s' % (result.success, result.message))
        
        if result.success:
            goal = ObjectManipulationGoal()
            goal.task = 'add_remove_obstacles'
            self.client_object_manipulation.send_goal(goal)
            self.client_object_manipulation.wait_for_result()
            result = self.client_object_manipulation.get_result()
            print('[Result] Success: %s, Message: %s' % (result.success, result.message))
        
            return 'arm_positioned'
        else:
            return 'arm_not_positioned'
        
        
        
        

class GraspingPositionsState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['grasps_computed', 'grasps_computation_failed'],
                                   input_keys=['poses'],
                                   output_keys=['grasps'])

    def execute(self, userdata):
        # Compute grasping positions logic here
        
        grasps_computed = False
        computed_grasps = []  # List of computed grasps
        if grasps_computed:
            userdata.grasps = computed_grasps
            return 'grasps_computed'
        else:
            return 'grasps_computation_failed'

class InverseKinematicsState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ik_solution_found', 'check_next_grasp', 'ik_solution_not_found'],
                                   input_keys=['grasps', 'n_grasp'],
                                   output_keys=['ik_solution', 'arm', 'n_grasp'])

    def execute(self, userdata):
        # Inverse kinematics logic here
        ik_solution_found = False
        found_ik_solution = []  # List of found IK solutions
        selected_arm = ''  # Selected arm
        more_grasps_to_check = False
        if ik_solution_found:
            userdata.ik_solution = found_ik_solution
            userdata.arm = selected_arm
            return 'ik_solution_found'
        elif more_grasps_to_check:
            userdata.n_grasp += 1
            return 'check_next_grasp'
        else:
            return 'ik_solution_not_found'

class PlanTrajectoryState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['planned', 'check_next_grasp', 'trajectory_not_feasible'],
                                   input_keys=['ik_solution', 'arm'],
                                   output_keys=['trajectory'])

    def execute(self, userdata):
        # Plan trajectory logic here
        trajectory_planned = False
        planned_trajectory = []  # List of planned trajectory
        more_grasps_to_check = False
        if trajectory_planned:
            userdata.trajectory = planned_trajectory
            return 'planned'
        elif more_grasps_to_check:
            userdata.n_grasp += 1
            return 'check_next_grasp'
        else:
            return 'trajectory_not_feasible'


class ExecuteTrajectoryState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['trajectory_finished', 'grasp'],
                                   input_keys=['trajectory'],
                                   output_keys=['grasped'])

    def execute(self, userdata):
        # Execute trajectory logic here
        trajectory_executed_successfully = False
        if trajectory_executed_successfully:
            userdata.grasped = True
            return 'trajectory_finished'
        else:
            return 'grasp'

class SafetyPositionForNavigationState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['safety_position'],
                                   input_keys=['grasped', 'arm'],
                                   output_keys=['safe_position'])

    def execute(self, userdata):
        # Move to safety position logic here
        move_to_safety_position_successful = False
        if move_to_safety_position_successful:
            return 'safety_position'
          
# Grasp Object Substate Machine
def create_grasp_object_sm():
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'],
                            input_keys=['goal_pose'],
                            output_keys=['shapes', 'poses', 'grasps', 'ik_solution', 'arm', 'trajectory', 'grasped'])

    with sm:


        smach.StateMachine.add('GRASPING_POSITIONS', GraspingPositionsState(),
                               transitions={'grasps_computed': 'INVERSE_KINEMATICS',
                                            'grasps_computation_failed': 'failed'})

        smach.StateMachine.add('INVERSE_KINEMATICS', InverseKinematicsState(),
                               transitions={'ik_solution_found': 'PLAN_TRAJECTORY',
                                            'check_next_grasp': 'INVERSE_KINEMATICS',
                                            'ik_solution_not_found': 'failed'})

        smach.StateMachine.add('PLAN_TRAJECTORY', PlanTrajectoryState(),
                               transitions={'planned': 'EXECUTE_TRAJECTORY',
                                            'check_next_grasp': 'INVERSE_KINEMATICS',
                                            'trajectory_not_feasible': 'failed'})

        smach.StateMachine.add('EXECUTE_TRAJECTORY', ExecuteTrajectoryState(),
                               transitions={'trajectory_finished': 'SAFETY_POSITION_FOR_NAVIGATION',
                                            'grasp': 'INVERSE_KINEMATICS'})

        smach.StateMachine.add('SAFETY_POSITION_FOR_NAVIGATION', SafetyPositionForNavigationState(),
                               transitions={'safety_position': 'succeeded'})

    return sm


class NavToPlace(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['goal_reached', 'goal_unreachable', 'preempted'],
                             input_keys=['target_pose'], output_keys=['target_pose'])
        self._ac = actionlib.SimpleActionClient('navigate_waypoint', NavigateWaypointAction)

    def execute(self, userdata):
        rospy.loginfo('Navigating to place...')
        self._ac.wait_for_server()

        goal = NavigateWaypointGoal()
        goal.target_pose = userdata.target_pose

        self._ac.send_goal(goal, feedback_cb=self.feedback_cb)

        while not rospy.is_shutdown():
            if self.preempt_requested():
                rospy.loginfo('Preempting navigation to place.')
                self._ac.cancel_goal()
                self.service_preempt()
                return 'preempted'

            if self._ac.wait_for_result(rospy.Duration(0.1)):
                state = self._ac.get_state()
                if state == GoalStatus.SUCCEEDED:
                    return 'goal_reached'
                elif state == GoalStatus.PREEMPTED or state == GoalStatus.ABORTED:
                    return 'goal_unreachable'
                else:
                    userdata.target_pose = self._ac.get_result().updated_goal
                    return 'preempted'

        return 'preempted'

    def feedback_cb(self, feedback):
        rospy.loginfo('Navigating: %s' % feedback.feedback)


class RecognizeEmotion(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['positive_emotion', 'negative_emotion'])
        self._emotion_detected = False
        self._emotion = 'positive'

    def execute(self, userdata):
        rospy.loginfo('Recognizing emotion...')
        self._emotion_detected = False
        self._emotion = rospy.get_param('recognized_emotion', 'positive')
        rospy.Timer(rospy.Duration(0.1), self.check_emotion, oneshot=True)
        while not self._emotion_detected:
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            rospy.sleep(0.1)
        return 'positive_emotion' if self._emotion == 'positive' else 'negative_emotion'

    def check_emotion(self, event):
        self._emotion_detected = True


def child_termination_cb(outcome_map):
    if outcome_map['RECOGNIZE_EMOTION'] == 'negative_emotion':
        return True
    return False


def outcome_cb(outcome_map):
    if outcome_map['RECOGNIZE_EMOTION'] == 'negative_emotion':
        return 'incorrect_object'
    else:
        return 'goal_reached'


class IdentifyPlaceLocation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['location_identified', 'identify_computation_failed'],
                             input_keys=['zone', 'object', 'arm', 'grasped', 'correct_object'],
                             output_keys=['location_zone'])
        # Initialization code here

    def execute(self, userdata):
        # Logic to identify place location
        location_identified = True  # Placeholder
        if location_identified:
            userdata.location_zone = 'place_zone'  # Example value
            return 'location_identified'
        else:
            return 'identify_computation_failed'

class ComputePlacePose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['place_pose_found', 'place_computation_failed'],
                             input_keys=['zone', 'object', 'arm', 'grasped', 'correct_object', 'location_zone'],
                             output_keys=['place_poses', 'n_place_poses'])
        # Initialization code here

    def execute(self, userdata):
        # Logic to compute place pose
        place_pose_found = True  # Placeholder
        if place_pose_found:
            userdata.place_poses = ['pose1', 'pose2']  # Example values
            userdata.n_place_poses = 0
            return 'place_pose_found'
        else:
            return 'place_computation_failed'

class InverseKinematics(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ik_solution_found', 'check_next_place_pose', 'ik_solution_not_found'],
                             input_keys=['zone', 'object', 'arm', 'grasped', 'correct_object', 'place_poses', 'n_place_poses'],
                             output_keys=['ik', 'n_place_poses'])
        # Initialization code here

    def execute(self, userdata):
        # Logic to compute inverse kinematics
        ik_solution_found = True  # Placeholder
        if ik_solution_found:
            userdata.ik = 'ik_solution'  # Example value
            return 'ik_solution_found'
        else:
            userdata.n_place_poses += 1
            if userdata.n_place_poses < len(userdata.place_poses):
                return 'check_next_place_pose'
            else:
                return 'ik_solution_not_found'

class PlanTrajectory(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['planned', 'trajectory_not_feasible', 'check_next_place_pose'],
                             input_keys=['zone', 'object', 'arm', 'grasped', 'correct_object', 'place_poses', 'n_place_poses', 'ik'],
                             output_keys=['trajectory', 'n_place_poses'])
        # Initialization code here

    def execute(self, userdata):
        # Logic to plan trajectory
        trajectory_planned = True  # Placeholder
        if trajectory_planned:
            userdata.trajectory = 'trajectory_plan'  # Example value
            return 'planned'
        else:
            userdata.n_place_poses += 1
            if userdata.n_place_poses < len(userdata.place_poses):
                return 'check_next_place_pose'
            else:
                return 'trajectory_not_feasible'

class ExecuteTrajectory(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['trajectory_finished', 'trajectory_failed'],
                             input_keys=['zone', 'object', 'arm', 'grasped', 'correct_object', 'trajectory'])
        # Initialization code here

    def execute(self, userdata):
        # Logic to execute trajectory
        trajectory_finished = True  # Placeholder
        if trajectory_finished:
            return 'trajectory_finished'
        else:
            return 'trajectory_failed'

class VerifyPlacement(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['placement_user_table', 'placement_robot_table', 'placement_failed'],
                             input_keys=['zone', 'object', 'arm', 'grasped', 'correct_object'])
        # Initialization code here

    def execute(self, userdata):
        # Logic to verify placement
        placement_verified = True  # Placeholder
        if placement_verified:
            if userdata.zone == 'user_table':
                return 'placement_user_table'
            else:
              return 'placement_robot_table'
        else:
            return 'placement_failed'

class SafetyArmRetreat(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeded_safety'],
                             input_keys=['zone', 'object', 'arm', 'grasped', 'correct_object'])
        # Initialization code here

    def execute(self, userdata):
        # Logic to move arm to safety position
        return 'succeded_safety'

# Define the place_object state machine
def create_place_object_state_machine():
    sm_place_object = smach.StateMachine(outcomes=['place_object_succeeded', 'place_object_failed', 'nav_home', 'new_object'])
    sm_place_object.userdata.zone = ''
    sm_place_object.userdata.object = ''
    sm_place_object.userdata.arm = ''
    sm_place_object.userdata.grasped = ''
    sm_place_object.userdata.correct_object = ''
    sm_place_object.userdata.location_zone = ''
    sm_place_object.userdata.place_poses = []
    sm_place_object.userdata.n_place_poses = 0
    sm_place_object.userdata.ik = ''
    sm_place_object.userdata.trajectory = ''

    with sm_place_object:
        smach.StateMachine.add('IDENTIFY_PLACE_LOCATION', IdentifyPlaceLocation(),
                               transitions={'location_identified': 'COMPUTE_PLACE_POSE',
                                            'identify_computation_failed': 'place_object_failed'})
        
        smach.StateMachine.add('COMPUTE_PLACE_POSE', ComputePlacePose(),
                               transitions={'place_pose_found': 'INVERSE_KINEMATICS',
                                            'place_computation_failed': 'place_object_failed'})
        
        smach.StateMachine.add('INVERSE_KINEMATICS', InverseKinematics(),
                               transitions={'ik_solution_found': 'PLAN_TRAJECTORY',
                                            'check_next_place_pose': 'INVERSE_KINEMATICS',
                                            'ik_solution_not_found': 'place_object_failed'})
        
        smach.StateMachine.add('PLAN_TRAJECTORY', PlanTrajectory(),
                               transitions={'planned': 'EXECUTE_TRAJECTORY',
                                            'trajectory_not_feasible': 'place_object_failed',
                                            'check_next_place_pose': 'INVERSE_KINEMATICS'})
        
        smach.StateMachine.add('EXECUTE_TRAJECTORY', ExecuteTrajectory(),
                               transitions={'trajectory_finished': 'VERIFY_PLACEMENT',
                                            'trajectory_failed': 'place_object_failed'})
        
        smach.StateMachine.add('SAFETY_ARM_RETREAT', SafetyArmRetreat(),
                               transitions={'succeded_safety': 'VERIFY_PLACEMENT'})
        
        smach.StateMachine.add('VERIFY_PLACEMENT', VerifyPlacement(),
                               transitions={'placement_user_table': 'nav_home',
                                            'placement_failed': 'place_object_failed',
                                            'placement_robot_table': 'new_object'})
        
    
    return sm_place_object

def outcome_nav_pick_cb(outcome_map):
    print("outcome_map: ", outcome_map)
    if outcome_map['NAV_TO_PICK'] == 'goal_reached' and outcome_map['OBJECT_BASED_GOAL'] == 'close_enough':
        return 'succeeded'
    if outcome_map['NAV_TO_PICK'] == 'goal_unreachable':
        return 'goal_unreachable'
    if outcome_map['NAV_TO_PICK'] == 'preempted' or outcome_map['OBJECT_BASED_GOAL'] == 'preempted':
        return 'preempted'
    if outcome_map['OBJECT_BASED_GOAL'] == 'goal_updated':
        return 'goal_updated'
    if outcome_map['OBJECT_BASED_GOAL'] == 'goal_not_updated':
        return 'goal_not_updated'
    return 'goal_unreachable'


class ProactiveAssistanceStateMachine:
    def __init__(self, package_path):
        self.sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
        self.package_path = package_path
        initialize_demo_config = load_config(self.package_path+'/config/initialize_demo_config.yaml')
        # self.sis = smach_ros.IntrospectionServer('server_name', self.sm, '/SM_ROOT')
        # self.sis.start()
        with self.sm:
            smach.StateMachine.add('INITIALIZE_DEMO', InitializeDemoState(initialize_demo_config),
                                    transitions={'succeeded': 'WAIT_TO_PICK',
                                                  'aborted': 'aborted'})
                  
            smach.StateMachine.add('WAIT_TO_PICK', WaitToPick(),
                                   transitions={'new_goal': 'CONCURRENT_OBJECT_NAV',
                                                'aborted': 'aborted',
                                                'preempted': 'preempted'})

            sm_nav_pick_con = smach.Concurrence(outcomes=['succeeded', 'goal_unreachable','goal_updated', 'goal_not_updated','preempted'],
                                   default_outcome='goal_unreachable',
                                   input_keys=['task_navigation_goal'],
                                   output_keys=['task_navigation_goal'],
                                  child_termination_cb=lambda outcome_map: True,
                                   outcome_cb=outcome_nav_pick_cb)

            with sm_nav_pick_con:
                smach.Concurrence.add('NAV_TO_PICK', NavToPick(),
                                      remapping={'task_navigation_goal':'task_navigation_goal'})
                smach.Concurrence.add('OBJECT_BASED_GOAL', ObjectBasedGoal(),
                                      remapping={'task_navigation_goal':'task_navigation_goal'})

            smach.StateMachine.add('CONCURRENT_OBJECT_NAV', sm_nav_pick_con,
                                      transitions={'succeeded':'MOVE_HEAD',
                                                    'goal_unreachable':'aborted',
                                                    'preempted':'preempted',
                                                    'goal_updated':'CONCURRENT_OBJECT_NAV',
                                                    'goal_not_updated':'CONCURRENT_OBJECT_NAV'})
            
            smach.StateMachine.add('MOVE_HEAD', MoveHeadState(0.0, -0.7),  # Adjust pan and tilt values as needed
                                   transitions={'succeeded': 'IDENTIFY_OBJECTS',
                                                'aborted': 'aborted'})


            smach.StateMachine.add('IDENTIFY_OBJECTS', IdentifyObjectsState(),
                                   transitions={'object_identified': 'GRASP_OBJECT',
                                                'object_not_identified': 'WAIT_TO_PICK'},
                                   remapping={'task_navigation_goal':'task_navigation_goal',
                                              'superquadrics':'superquadrics',
                                              'object_superquadric_id':'object_superquadric_id'})
            
            smach.StateMachine.add('GRASP_OBJECT', GraspObjectState(initialize_demo_config),
                                    transitions={'object_grasped': 'SAFETY_ARM_POSITION',
                                                  'object_not_grasped': 'WAIT_TO_PICK'})
            smach.StateMachine.add('SAFETY_ARM_POSITION', SafetyArmPositionState(),
                                    transitions={'arm_positioned': 'succeeded',
                                                  'arm_not_positioned': 'aborted'})
            
            
            
            # smach.StateMachine.add('GRASP_OBJECT', create_grasp_object_sm(),
            #                    transitions={'succeeded': 'NAV_OBJECT_USER',
            #                                 'failed': 'NAV_TO_PICK'})
            
            # sm_con = smach.Concurrence(outcomes=['correct_object', 'incorrect_object'],
            #                        default_outcome='incorrect_object',
            #                        outcome_map={'correct_object':
            #                                     {'NAV_TO_PLACE': 'goal_reached',
            #                                      'RECOGNIZE_EMOTION': 'positive_emotion'},
            #                                     'incorrect_object':
            #                                     {'RECOGNIZE_EMOTION': 'negative_emotion'}},
            #                        child_termination_cb=child_termination_cb,
            #                        outcome_cb=outcome_cb)
            
          
            
            # with sm_con:
            #     smach.Concurrence.add('NAV_TO_PLACE', NavToPlace())
            #     smach.Concurrence.add('RECOGNIZE_EMOTION', RecognizeEmotion())

            # self.sm.add('NAV_OBJECT_USER', sm_con,
            #       transitions={'correct_object': 'succeeded',
            #                    'incorrect_object': 'NAV_TO_PLACE'})
            
            # self.sm.add('NAV_TO_PLACE', NavToPlace(),
            #       transitions={'goal_reached': 'succeeded',
            #                    'goal_unreachable': 'aborted',
            #                    'preempted': 'preempted'})

            # smach.StateMachine.add('PLACE_OBJECT', create_place_object_state_machine(),
            #                         transitions={'place_object_succeeded': 'succeeded',
            #                                       'place_object_failed': 'aborted',
            #                                       'nav_home': 'succeeded',
            #                                       'new_object': 'IDENTIFY_OBJECTS'})
            
            
            
                                                          
        sis = smach_ros.IntrospectionServer('proactive_assistance_introspection', self.sm, '/SM_ROOT')
        sis.start()

    def execute(self):
        rospy.loginfo('Starting state machine...')
        outcome = self.sm.execute()
        rospy.loginfo('State machine finished with outcome: %s' % outcome)


if __name__ == '__main__':
    rospy.init_node('navigation_state_machine')
    package_name = 'proactive_assistance'
    package_path = find_package_path(package_name)
    if package_path:
        nsm = ProactiveAssistanceStateMachine(package_path)
        nsm.execute()
        rospy.spin()
        nsm.sis.stop()
    else:
        print("Package", package_name, "not found.")
        

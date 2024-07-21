#!/usr/bin/env python
import rospy
import actionlib
from geometry_msgs.msg import Pose, PoseArray
from shape_msgs.msg import SolidPrimitive
from proactive_assistance.msg import ObjectManipulationAction, ObjectManipulationGoal
from companion_msgs.srv import ActivateSupercuadricsComputation, ActivateSupercuadricsComputationRequest, GetSuperquadrics, GetSuperquadricsRequest, GetSuperquadricsResponse
from companion_msgs.srv import ComputeGraspPoses, ComputeGraspPosesRequest, ComputeGraspPosesResponse
def feedback_cb(feedback):
    print('[Feedback] Current Status: %s' % feedback.status)

def call_server():
    rospy.init_node('test_object_manipulation_client')

    # Create a client for the ObjectManipulation action server
    client = actionlib.SimpleActionClient('object_manipulation', ObjectManipulationAction)
    client.wait_for_server()

    # Define a goal to send to the action server
    goal = ObjectManipulationGoal()
    goal.gripper_empty = True  # Assuming gripper is empty

    # Define desired object
    # Here you need to create an instance of your superquadric message
    # goal.desired_object = YourSuperquadricMessage()

    # Define grasping poses
    pose = Pose()
    pose.position.x = 1.0
    pose.position.y = 0.0
    pose.position.z = 0.5
    pose.orientation.w = 1.0
    grasping_poses = PoseArray()
    grasping_poses.poses.append(pose)
    goal.grasping_poses = grasping_poses

    # Define obstacles
    obstacle = SolidPrimitive()
    obstacle.type = SolidPrimitive.BOX
    obstacle.dimensions = [1.3, 3.0, 0.6]  # Example dimensions
    goal.obstacles.append(obstacle)
    
    obstacle_pose = Pose()
    obstacle_pose.position.x = 1.0
    obstacle_pose.position.y = 0.0
    obstacle_pose.position.z = 0.3
    obstacle_pose.orientation.w = 1.0
    grasping_poses = PoseArray()
    goal.obstacle_poses.append(obstacle_pose)  # Assuming same pose for simplicity
    
    
    goal.reference_frames_of_obstacles.append('base_footprint')
    
    client_activate_object_shape_computation =rospy.ServiceProxy("/grasp_objects/activate_superquadrics_computation", ActivateSupercuadricsComputation)
    client_get_superquadrics = rospy.ServiceProxy("/grasp_objects/get_superquadrics", GetSuperquadrics)
    client_get_grasp_poses = rospy.ServiceProxy("/grasp_objects/compute_grasp_poses", ComputeGraspPoses)
    
    
    rospy.loginfo('Identifying objects...')
    # activateRequest = ActivateSupercuadricsComputationRequest()
    # activateRequest.activate = True
    # client_activate_object_shape_computation.call(activateRequest)
        
    srvSq = GetSuperquadricsRequest()
    superquadrics_msg = client_get_superquadrics.call(srvSq)
    print("superquadrics_msg: ", superquadrics_msg)
    
    desired_id = 2
    id = 0
    for i in range(len(superquadrics_msg.superquadrics.superquadrics)):
        print("superquadrics_msg.superquadrics.superquadrics[i].id: ", superquadrics_msg.superquadrics.superquadrics[i].id)
        if superquadrics_msg.superquadrics.superquadrics[i].id == desired_id:
            print("found")
            id = i
            break
    print("id: ", id)
    goal.desired_object = superquadrics_msg.superquadrics.superquadrics[id]
    grasp_poses_request = ComputeGraspPosesRequest()
    grasp_poses_request.id = desired_id
    grasp_poses = client_get_grasp_poses.call(grasp_poses_request)
    goal.grasping_poses = grasp_poses.poses

    goal.gripper_empty = True
    print("grasp_poses: ", grasp_poses)
    
    
    # Sends the goal to the action server
    client.send_goal(goal, feedback_cb=feedback_cb)

    # Waits for the server to finish performing the action
    client.wait_for_result()

    # Prints out the result of executing the action
    result = client.get_result()
    print('[Result] Success: %s, Message: %s' % (result.success, result.message))

if __name__ == '__main__':
    try:
        call_server()
    except rospy.ROSInterruptException:
        pass

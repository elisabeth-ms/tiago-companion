#!/usr/bin/env python

import rospy
import tf
import numpy as np
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseStamped, Point, PoseWithCovarianceStamped
import actionlib
import sensor_msgs.point_cloud2 as pc2
from companion_msgs.msg import BoundingBoxesLabels
from proactive_assistance.msg import NewWaypointAction, NewWaypointGoal, NewWaypointResult


class ObjectBasedNavigationServer:
    def __init__(self, name):
        self._action_name = name
        rospy.loginfo("Starting FakeObjectBasedNavigationServer")
        
        self._as = actionlib.SimpleActionServer(self._action_name, NewWaypointAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        
        self._enabled = True
                        
        self.amcl_pose = PoseStamped()
        

    def execute_cb(self, request):
        
        rospy.loginfo("ObjectBasedNavigationServer: Received a goal update request")
        print("request: ", request)
        self.desired_class = request.object_name

        

        new_goal = NewWaypointResult()
        new_goal.result = "new_goal"
        new_goal.target_pose = PoseStamped()
        new_goal.target_pose.header.stamp = rospy.Time.now()
        new_goal.target_pose.header.frame_id = "map"
        new_goal.target_pose.pose.position.x = 0.0
        new_goal.target_pose.pose.position.y = 0.0
        new_goal.target_pose.pose.position.z = 0.0
        new_goal.target_pose.pose.orientation.z = 0.7212835242072623
        new_goal.target_pose.pose.orientation.w = 0.6923883162821826
        new_goal.current_pose = new_goal.target_pose
        rospy.loginfo("ObjectBasedNavigationServer: New goal generated at x: %f, y: %f",
                              new_goal.target_pose.pose.position.x, new_goal.target_pose.pose.position.y)

        self._as.set_succeeded(new_goal)


if __name__ == '__main__':
    rospy.init_node('object_based_navigation')
    obj_nav = ObjectBasedNavigationServer('goal_update')
    
    # Periodically process the bounding box and point cloud to send a new goal
    rate = rospy.Rate(1)  # Adjust the rate as needed

    rospy.spin()

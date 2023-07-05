#! /usr/bin/env python3

import os
import datetime
import numpy as np
import rospy
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64
import actionlib
from pal_common_msgs.msg import EmptyAction, EmptyActionGoal
from companion_msgs.srv import ActivateSupercuadricsComputation, ActivateSupercuadricsComputationResponse 

class DetectHitFT(object):
    def __init__(self):
        
        rospy.loginfo("Initalizing detect_hit_ft_node...")
        self.max_force = 15
        self.pub_force_left = rospy.Publisher('force_wrist_left', Float64, queue_size=10)        
        self.pub_force_right = rospy.Publisher('force_wrist_right', Float64, queue_size=10)
        self.subscriber_wrist_left_ft = rospy.Subscriber("/wrist_left_ft/corrected", WrenchStamped, self.wristLeftFTCallback,  queue_size = 5)
        self.subscriber_wrist_right_ft = rospy.Subscriber("/wrist_right_ft/corrected", WrenchStamped, self.wristRightFTCallback,  queue_size = 5)
        self.clientActivate = rospy.Service('/detect_hit_ft/activate', ActivateSupercuadricsComputation, self.handle_activate_detect_hit_ft)
        self.client = actionlib.SimpleActionClient('/gravity_compensation', EmptyAction)
        self.activate = True
        self.client.wait_for_server()
        rospy.loginfo("detect_hit_ft_node initialized.")


    def wristLeftFTCallback(self, ros_data):
        if self.activate:
            data = ros_data.wrench
            force = np.sqrt(data.force.x*data.force.x+data.force.y+data.force.y+data.force.z*data.force.z)
            self.pub_force_left.publish(force)
            if force>self.max_force or abs(data.force.x)>self.max_force or abs(data.force.y)>self.max_force or abs(data.force.z)>self.max_force:
                goal = EmptyActionGoal()
                self.client.send_goal(goal)
                print("Gravity compensation activated!!")
                self.client.wait_for_result()
                print(self.client.get_result())
            # print("Wrist left force(x: ", data.force.x, ", y: ", data.force.y, ", z: ", data.force.z, ")")

    def wristRightFTCallback(self, ros_data):
        if self.activate:
            data = ros_data.wrench
            force = np.sqrt(data.force.x*data.force.x+data.force.y+data.force.y+data.force.z*data.force.z)
            self.pub_force_right.publish(force)
            if force>self.max_force or abs(data.force.x)>self.max_force or abs(data.force.y)>self.max_force or abs(data.force.z)>self.max_force:
                goal = EmptyActionGoal()
                self.client.send_goal(goal)
                print("Gravity compensation activated!!")
                self.client.wait_for_result()
                print(self.client.get_result())
            # print("Wrist rightt force(x: ", data.force.x, ", y: ", data.force.y, ", z: ", data.force.z, ")")


    def handle_activate_detect_hit_ft(self, req):
        self.activate = req.activate
        print("detect_hit_ft activate: ", self.activate)
        return ActivateSupercuadricsComputationResponse(req.activate)


if __name__ == '__main__':
	rospy.init_node('detect_hit_ft_node')
	detectHitFT = DetectHitFT()
	rospy.spin()


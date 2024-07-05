#!/usr/bin/env python
import rospy
import actionlib
from proactive_assistance.msg import NewWaypointAction, NewWaypointGoal, NewWaypointResult
from geometry_msgs.msg import PoseStamped
import random
from std_srvs.srv import SetBool

class FakeGoalUpdateServer:
    def __init__(self, name):
        self._action_name = name
        
        self._as = actionlib.SimpleActionServer(self._action_name, NewWaypointAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        self._service = rospy.Service('fake_goal_update/enable', SetBool, self.enable_cb)
        self._enabled = False
        rospy.loginfo("FakeGoalUpdateServer started.")

    def execute_cb(self, request):
        if self._enabled == True:
          self._enabled = False
          rospy.loginfo("FakeGoalUpdateServer: Received a goal update request")

          # Simulate generating a new goal
          new_goal = NewWaypointResult()
          new_goal.result = "new_goal"
          new_goal.target_pose = PoseStamped()
          new_goal.target_pose.header.stamp = rospy.Time.now()
          new_goal.target_pose.header.frame_id = "map"

          # Randomly generate a new goal position within a certain range
          new_goal.target_pose.pose.position.x = random.uniform(-0.2, 0.8)
          new_goal.target_pose.pose.position.y = random.uniform(-0.2, 0.8)
          new_goal.target_pose.pose.position.z = 0.0
          new_goal.target_pose.pose.orientation.z = 1.0

          rospy.loginfo("FakeGoalUpdateServer: New goal generated at x: %f, y: %f",
                        new_goal.target_pose.pose.position.x, new_goal.target_pose.pose.position.y)

          self._as.set_succeeded(new_goal)
        else:
          rospy.loginfo("FakeGoalUpdateServer: Received a goal update request, but the server is disabled")
          self._as.set_aborted()
    
    def enable_cb(self, request):
        if request.data:
            self._enabled = True
            rospy.loginfo("FakeGoalUpdateServer: Enabled")
        else:
            self._enabled = False
            rospy.loginfo("FakeGoalUpdateServer: Disabled")
        return True, "enabled: " + str(self._enabled)

if __name__ == '__main__':
    rospy.init_node('fake_goal_update_server')
    server = FakeGoalUpdateServer('goal_update')
    rospy.spin()

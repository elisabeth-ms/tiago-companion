#! /usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from proactive_assistance.msg import NavigateWaypointFeedback, NavigateWaypointResult, NavigateWaypointAction, NewWaypointAction, NewWaypointGoal, NewWaypointResult

class NavigationMacroActionServerFake:
    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, NavigateWaypointAction, execute_cb=self.execute_cb, auto_start=False)
        self._feedback = NavigateWaypointFeedback()
        self._result = NavigateWaypointResult()
        rospy.loginfo("NavigationMacroActionServer started.")
        self._as.start()

    def execute_cb(self, goal):
        rospy.loginfo("NavigationMacroActionServer: Executing goal")
        

        rate = rospy.Rate(1)
        count = 0
        while not rospy.is_shutdown():
            if count == 5:
                self._result.result = "succeeded"
                self._result.updated_goal = PoseStamped()
                self._as.set_succeeded(self._result)
                return


            count += 1
            self._as.publish_feedback(NavigateWaypointFeedback(feedback="Navigating"))
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('navigation_macro_action_server')
    server = NavigationMacroActionServerFake('navigate_waypoint')
    rospy.spin()

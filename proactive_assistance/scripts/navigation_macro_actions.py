#! /usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from proactive_assistance.msg import NavigateWaypointFeedback, NavigateWaypointResult, NavigateWaypointAction, NewWaypointAction, NewWaypointGoal, NewWaypointResult

class NavigationMacroActionServer:
    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, NavigateWaypointAction, execute_cb=self.execute_cb, auto_start=False)
        self._feedback = NavigateWaypointFeedback()
        self._result = NavigateWaypointResult()
        self._move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self._goal_update_client = actionlib.SimpleActionClient('goal_update', NewWaypointAction)
        rospy.loginfo("Waiting for move_base action server...")
        self._move_base_client.wait_for_server()
        rospy.loginfo("Waiting for goal_update action server...")
        self._goal_update_client.wait_for_server()
        rospy.loginfo("NavigationMacroActionServer started.")
        self._as.start()

    def execute_cb(self, goal):
        rospy.loginfo("NavigationMacroActionServer: Executing goal")

        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose = goal.target_pose
        print("move_base_goal: ", goal)
        self._move_base_client.send_goal(move_base_goal)

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self._as.is_preempt_requested():
                rospy.loginfo('NavigationMacroActionServer: Preempted')
                self._move_base_client.cancel_goal()
                self._as.set_preempted()
                return


            # Check if an update is required
            self._goal_update_client.send_goal(NewWaypointGoal())
            if self._goal_update_client.wait_for_result(rospy.Duration(0.5)):
                new_goal_result = self._goal_update_client.get_result()
                print("new_goal_result: ", new_goal_result)
                print("---------------------------------------------------------------------------------------------------")
                if new_goal_result.result == "new_goal":
                    new_goal = new_goal_result.target_pose
                    if new_goal != move_base_goal.target_pose:
                        rospy.loginfo("NavigationMacroActionServer: Goal update required")
                        self._move_base_client.cancel_goal()
                        move_base_goal.target_pose = new_goal
                        self._feedback.feedback = 'new_goal'
                        self._as.publish_feedback(self._feedback)
                        print("new_goal: ", new_goal) 
                        self._result.result = "new_goal"
                        self._result.updated_goal = new_goal
                        self._as.set_preempted(self._result)
                        return 

            if self._move_base_client.wait_for_result(rospy.Duration(1.0)):
                if self._move_base_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                    rospy.loginfo("NavigationMacroActionServer: Goal succeeded")
                    self._result.result = "succeeded"
                    self._result.updated_goal = PoseStamped()
                    self._as.set_succeeded(self._result)
                    return
                elif self._move_base_client.get_state() == actionlib.GoalStatus.ABORTED:
                    rospy.loginfo("NavigationMacroActionServer: Goal failed")
                    self._result.result = "failed"
                    self._result.updated_goal = PoseStamped()
                    self._as.set_aborted(self._result)
                    return
                elif self._move_base_client.get_state() == actionlib.GoalStatus.ACTIVE:
                    rospy.loginfo("NavigationMacroActionServer: Goal active")


            self._as.publish_feedback(NavigateWaypointFeedback(feedback="Navigating"))
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('navigation_macro_action_server')
    server = NavigationMacroActionServer('navigate_waypoint')
    rospy.spin()

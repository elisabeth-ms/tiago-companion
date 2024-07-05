#!/usr/bin/env python
import rospy
import smach
import smach_ros
import actionlib
from proactive_assistance.msg import NavigateWaypointAction, NavigateWaypointGoal, NewWaypointAction, NewWaypointGoal, NewWaypointResult
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped

class WaitForGoalState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['goal_received', 'aborted', 'preempted'],
                             output_keys=['goal_pose'])
        self.goal_pose = PoseStamped()
        self.goal_received = False
        self.subscriber = rospy.Subscriber('/proactive_assistance/nav_goal', PoseStamped, self.goal_cb)
        self._goal_update_client = actionlib.SimpleActionClient('goal_update', NewWaypointAction)


    def goal_cb(self, msg):
        rospy.loginfo('Goal received.')
        self.goal_received = True
        self.goal_pose = msg

    def execute(self, userdata):
        rospy.loginfo('Waiting for goal...')
        while not self.goal_received:
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            self._goal_update_client.send_goal(NewWaypointGoal())
            if self._goal_update_client.wait_for_result(rospy.Duration(0.5)):
                new_goal_result = self._goal_update_client.get_result()
                if new_goal_result.result == "new_goal":
                    self.goal_pose = new_goal_result.target_pose
                    print("new_goal: ", self.goal_pose)
                    self.goal_received = True
            rospy.sleep(1)
        if self.goal_received:
            self.goal_received = False
            userdata.goal_pose = self.goal_pose
            return 'goal_received'

class NavigateToGoalState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted', 'new_goal'],
                             input_keys=['goal_pose'], output_keys=['goal_pose'])
        self._ac = actionlib.SimpleActionClient('navigate_waypoint', NavigateWaypointAction)
        
    def execute(self, userdata):
        rospy.loginfo('Navigating to goal...')
        self._ac.wait_for_server()

        goal = NavigateWaypointGoal()
        goal.target_pose = userdata.goal_pose
        print("goal: ", goal)
        # self._ac.send_goal(goal, feedback_cb=self.feedback_cb)
        self._ac.send_goal(goal)

        state = self._ac.wait_for_result()

        print("State: ", state)
        result = self._ac.get_result()
        print("Result: ", result)
        
        if result.result == "new_goal":
            print("new_goal")
            userdata.goal_pose = result.updated_goal
            return 'new_goal'
        elif result.result == "succeeded":
            print("succeeded")
            return 'succeeded'
        elif result.result == "failed":
            print("aborted")
            return 'aborted'
        elif result.result == "preempted":
            print("preempted")
            return 'preempted'

    # def feedback_cb(self, feedback):
        # Simulating a condition to update the goal
        # if rospy.Time.now().to_sec() % 10 < 1:  # Change condition as needed
        #     rospy.loginfo('Goal update required.')
        #     self._ac.cancel_goal()
        #     return 'update_required'

class UpdateGoalState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['updated', 'aborted', 'preempted'],
                             input_keys=['goal_pose'], output_keys=['goal_pose'])

    def execute(self, userdata):
        rospy.loginfo('Updating goal...')        
        rospy.sleep(1)  # Simulate some processing time
        return 'updated'


class NavigationStateMachine:
    def __init__(self):
        self.sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

        self.sis = smach_ros.IntrospectionServer('server_name', self.sm, '/SM_ROOT')
        self.sis.start()
        with self.sm:
          
            smach.StateMachine.add('WAIT_FOR_GOAL', WaitForGoalState(),
                                   transitions={'goal_received': 'NAVIGATE_TO_GOAL',
                                                'aborted': 'aborted',
                                                'preempted': 'preempted'})

            smach.StateMachine.add('NAVIGATE_TO_GOAL', NavigateToGoalState(),
                                   transitions={'succeeded': 'succeeded',
                                                'aborted': 'WAIT_FOR_GOAL',
                                                'preempted': 'preempted',
                                                'new_goal': 'NAVIGATE_TO_GOAL'})


    def execute(self):
        rospy.loginfo('Starting state machine...')
        outcome = self.sm.execute()
        rospy.loginfo('State machine finished with outcome: %s' % outcome)


if __name__ == '__main__':
    rospy.init_node('navigation_state_machine')
    nsm = NavigationStateMachine()
    nsm.execute()
    rospy.spin()
    nsm.sis.stop()

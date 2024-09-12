#!/usr/bin/env python
# Path: proactive_assistance/scripts/fake_task_navigation_goal.py
# This script publishes fake TaskNavigationGoal messages to the /proactive_assistance/task_navigation_goal topic. 
# It publishes an initial TaskNavigationGoal message with the target_pose set to a specific position and orientation.
# Also, it publishes the zone and object_name fields.

# TODO: For what I understand right now, the robot must be at the beginning of the demo looking towards the user, then once the
# user indicates the object, the robot will move to a position far from the objects table looking at the table, so that we can
# see all the objects. This script emulates that behavior. In the real demo, the robot will wait for the user command to get an
# specific object, then it will move to the table and look at the table to see all the objects.

import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from companion_msgs.msg import TaskNavigationGoal  # Make sure to replace 'your_package' with the actual package name

def publish_fake_goals():
    rospy.init_node('fake_task_navigation_goal_publisher')
    
    # Publisher for TaskNavigationGoal
    pub = rospy.Publisher('proactive_assistance/task_navigation_goal', TaskNavigationGoal, queue_size=10)
    rospy.sleep(1)  # Wait for the subscriber to connect

    rate = rospy.Rate(1)  # 1 Hz

    # Create a new TaskNavigationGoal message
    task_goal = TaskNavigationGoal()
    
    
    
    for i in range(2):
        # Define the task_pose
        task_goal.target_pose = PoseStamped()
        task_goal.target_pose.header.frame_id = "map"
        task_goal.target_pose.header.stamp = rospy.Time.now()
        



        task_goal.target_pose.pose.position = Point(-0.35, -0.1043135567279, 0.0)
        task_goal.target_pose.pose.orientation = Quaternion(0.0, 0.0, 0, 1)
        # Set the position and orientation far from the objects table looking at the table, so that we can see all the objects
        #task_goal.target_pose.pose.position = Point(-0.56,  0.336918167052, 0.0) 




        
        #task_goal.target_pose.pose.orientation = Quaternion(0.0, 0.0, -0.00103007814223, 0.999999469469)

        # Set the zone and object
        task_goal.zone = 'robot_table'
        task_goal.object_name = "appricot jam"

            # Publish the TaskNavigationGoal
        rospy.loginfo("Publishing fake TaskNavigationGoal")
        pub.publish(task_goal)
        rospy.loginfo("Published TaskNavigationGoal: %s", task_goal)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_fake_goals()
    except rospy.ROSInterruptException:
        pass

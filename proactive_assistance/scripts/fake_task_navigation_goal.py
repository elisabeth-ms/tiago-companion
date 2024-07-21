#!/usr/bin/env python
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
    
    
    
    for i in range(5):
        # Define the task_pose
        task_goal.target_pose = PoseStamped()
        task_goal.target_pose.header.frame_id = "map"
        task_goal.target_pose.header.stamp = rospy.Time.now()
        
        task_goal.target_pose.pose.position = Point( 0.2,  0.0, 0.0)  # Example position
        task_goal.target_pose.pose.orientation = Quaternion(0.0, 0.0, 0.68820891652, 0.725512568618)  # Example orientation

            # Set the zone and object
        task_goal.zone = 'robot_table'
        task_goal.object_name = 'biscuits'

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

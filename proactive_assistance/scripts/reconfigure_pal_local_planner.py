#!/usr/bin/env python

import rospy
from dynamic_reconfigure.client import Client

def reconfigure_pal_local_planner():
    # Initialize the ROS node
    rospy.init_node('pal_local_planner_reconfigure')

    # Create a dynamic reconfigure client to interact with the pal_local_planner
    client = Client('/move_base/PalLocalPlanner', timeout=30)

    # Define the parameters you want to change
    new_params = {
        'max_vel_trans': 0.25,  # Set the maximum translational velocity (m/s)
        'min_vel_trans': 0.05,  # Set the minimum translational velocity (m/s)
        'max_vel_x': 0.2,       # Set the maximum velocity in x-axis
        'min_vel_x': 0.0,       # Set the minimum velocity in x-axis
        'acc_lim_x': 0.5,       # Acceleration limit in the x-axis (m/s^2)
        'acc_lim_theta': 1.0,   # Acceleration limit in the rotational axis (rad/s^2)
        'yaw_goal_tolerance': 0.1,  # Tolerance for reaching the goal orientation (rad)
        'xy_goal_tolerance': 0.1,   # Tolerance for reaching the goal position (meters)
        'allow_backwards': True,    # Allow the robot to move backwards
        # Add other parameters as needed
    }

    # Set the parameters dynamically
    try:
        result = client.update_configuration(new_params)
        rospy.loginfo("Reconfigured parameters %s: ",result)
    except rospy.ServiceException as e:
        rospy.logerr("Failed to reconfigure pal_local_planner %s: ", e)

if __name__ == '__main__':
    try:
        reconfigure_pal_local_planner()
    except rospy.ROSInterruptException:
        pass

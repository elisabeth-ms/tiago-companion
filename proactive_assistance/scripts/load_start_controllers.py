#!/usr/bin/env python

import rospy
from controller_manager_msgs.srv import LoadController, SwitchController

def load_and_start_controllers():
    rospy.init_node('controller_loader')

    # Wait for necessary services
    rospy.wait_for_service('/controller_manager/load_controller')
    rospy.wait_for_service('/controller_manager/switch_controller')

    # Create service proxies
    load_controller = rospy.ServiceProxy('/controller_manager/load_controller', LoadController)
    switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)

    # List of controllers to load and start based on the screenshot
    controllers_to_load = [
            'arm_left_controller',
            'arm_left_current_limit_controller',
            'arm_right_controller',
            'arm_right_current_limit_controller',
            'arm_left_impedance_controller',
            'gripper_left_controller',
            'gripper_right_controller',
            'head_controller',
            'joint_state_controller',
            'mobile_base_controller',
            'torso_controller',
            'wheels_current_limit_controller',
            'cartesian_impedance_left_arm',
            'cartesian_impedance_right_arm',
            'force_torque_sensor_controller',
            'gravity_compensation',
            'gripper_left_current_limit_controller',
            'gripper_right_current_limit_controller',
            'pal_robot_info',
            'safe_shutdown_controller'
        ]

    loaded_controllers = []
    
    # Load all controllers
    for controller in controllers_to_load:
        try:
            load_controller(controller)
            loaded_controllers.append(controller)
            rospy.loginfo("Loaded controller: %s", controller)
        except rospy.ServiceException as e:
            rospy.logerr("Failed to load controller %s : %s",controller, str(e))

    # Start all loaded controllers
    try:
        switch_controller(start_controllers=loaded_controllers, stop_controllers=[], strictness=2)
        rospy.loginfo("Started controllers: %s", loaded_controllers)
    except rospy.ServiceException as e:
        rospy.logerr("Failed to start controllers: %s", str(e))

if __name__ == '__main__':
    try:
        load_and_start_controllers()
    except rospy.ROSInterruptException:
        pass

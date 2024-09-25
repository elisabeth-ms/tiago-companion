from messages import RobotMessage
from utils import send_UDP
import pickle
import socket

import time
import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Bool



IP_SEND = "192.168.149.172"
PORT_SEND = 5005

def callback(data):
    print("Received robot_executing_task: ", data)
    # Get the current time in seconds
    current_time_seconds = time.time()

    # Convert to nanoseconds
    current_time_nanoseconds = int(current_time_seconds * 1e9)
    if data.data == True:
        msg = RobotMessage("movement", "start", current_time_nanoseconds)
    else:
        msg = RobotMessage("movement", "finish", current_time_nanoseconds)
    # send the messages
    send_UDP(IP_SEND, PORT_SEND, msg)

rospy.init_node('comms_client_proactive_assistance')

rospy.Subscriber('proactive_assistance/robot_executing_task', Bool, callback)
rospy.spin()
from messages import RobotMessage
from utils import send_UDP
import pickle
import socket

import time
import rospy
from companion_msgs.msg import TaskNavigationGoal
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Bool, String



IP = "192.168.147.153"
PORT = 5005


rospy.init_node('comms_server_proactive_assistance')

# Create a socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
try:
    # Bind the socket to the port
    sock.bind((IP, PORT))
    print("Waiting for message...")

except Exception as e:
    print("Error:", e)

# Ros publisher
pub_task_navigation_goal= rospy.Publisher('proactive_assistance/task_navigation_goal', TaskNavigationGoal, queue_size=10)
pub_user_emotion_recognition = rospy.Publisher('proactive_assistance/user_emotion_recognition', String, queue_size=10)

while not rospy.is_shutdown():

    # Receive data and print the message
    data, addr = sock.recvfrom(1024)
    message = pickle.loads(data)
    print("message:", message)
    print("Received message type: ", message.type)
    
    if message.type == "microphone":

        print("Received message: ", message.message)
        task_goal = TaskNavigationGoal()
        # Define the task_pose
        task_goal.target_pose = PoseStamped()
        task_goal.target_pose.header.frame_id = "map"
        task_goal.target_pose.header.stamp = rospy.Time.now()
            
        task_goal.target_pose.pose.position = Point(-0.35, -0.1043135567279, 0.0)
        task_goal.target_pose.pose.orientation = Quaternion(0.0, 0.0, 0, 1)


        # Set the zone and object
        task_goal.zone = 'robot_table'
        task_goal.object_name = message.message

        # Publish the TaskNavigationGoal
        rospy.loginfo("Publishing fake TaskNavigationGoal")
        pub_task_navigation_goal.publish(task_goal)
        rospy.loginfo("Published TaskNavigationGoal: %s", task_goal)
        
        
    if message.type == "camera":
        print("Received message: ", message.message)
        pub_user_emotion_recognition.publish(message.message)

# Close the socket
sock.close()
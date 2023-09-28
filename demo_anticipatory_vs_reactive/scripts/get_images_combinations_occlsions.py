#!/usr/bin/env python
import pandas as pd
import os
import rospy
from sensor_msgs.msg import Image, PointCloud2
import cv2
from cv_bridge import CvBridge
import message_filters
import subprocess

# Initialize ROS node
rospy.init_node('data_saver_node')

# Create CvBridge object
bridge = CvBridge()

# Create directories for saving data
base_dir = "objects_partially_occluded"
if not os.path.exists(base_dir):
    os.makedirs(base_dir)

# Load the Excel file
df = pd.read_excel('object_combinations_with_occlusions.xlsx')

# Ask user for starting case number
start_case = int(input("Enter the starting case number: "))

# Check if the provided start_case is valid
if start_case not in df['Case_Num'].values:
    print("Invalid case number. Exiting.")
    exit()

def start_rosbag(case_num):
    bag_name = os.path.join(base_dir, "case_{}.bag".format(case_num))
    return subprocess.Popen(["rosbag", "record", "-O", bag_name, "/xtion/rgb/image_rect_color", "/xtion/depth/image_rect", "/xtion/depth_registered/points"])

def stop_rosbag(process):
    process.terminate()
    process.wait()

try:
    # Iterate through each row starting from the user-specified case
    for _, row in df[df['Case_Num'] >= start_case].iterrows():
        case_num = int(row['Case_Num'])
        if os.path.exists(os.path.join(base_dir, "case_{}.bag".format(case_num))):
            overwrite = raw_input("Rosbag for Case {} already exists. Do you want to overwrite it? (y/n): ".format(case_num))
            if overwrite.lower() != 'y':
                continue



        for col in ['Obj_A', 'Obj_B', 'Obj_C', 'Obj_D']:
            object_name = row[col]
            orientation = row[col + '_Ori']
            occlusion = row[col + '_Occ']
            occluded_by = row[col + '_Occ_By']
            
            if occlusion == "Visible":
                print(object_name+" is "+orientation+" and visible.")
            else:
                print(object_name+" is "+orientation+" and occluded by "+occluded_by+".")

        print("Starting recording for Case "+str(case_num)+":")
        user_input = raw_input("Press Enter to start recording\n")

        rosbag_process = start_rosbag(case_num)
        
        user_input = raw_input("Press Enter to stop recording and see the next case or 'e' to exit...\n")
        stop_rosbag(rosbag_process)

        if user_input.lower() == 'e':
            break
        
except KeyboardInterrupt:
    print("\nScript interrupted by user. Exiting...")
    if rosbag_process.poll() is None:  # Check if rosbag process is still running
        stop_rosbag(rosbag_process)
    exit()

rospy.spin()
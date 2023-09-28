#!/usr/bin/env python

import rospy
import rosbag
import os
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import glob
import numpy as np
# Initialize ROS node
rospy.init_node('rosbag_image_extractor')

# Create CvBridge object
bridge = CvBridge()

# Directory where your rosbags are stored
base_dir = "objects_partially_occluded"
bag_directory = os.path.join(base_dir, "bags")

# Create directories for saving images
color_dir = os.path.join(base_dir, "color")
depth_dir = os.path.join(base_dir, "depth")

for directory in [color_dir, depth_dir]:
    if not os.path.exists(directory):
        os.makedirs(directory)

# Get list of all rosbags in the directory
rosbags = glob.glob(os.path.join(bag_directory, 'case_*.bag'))

for bag_path in rosbags:
    # Extract case number from the rosbag filename
    case_num = os.path.basename(bag_path).split('_')[1].split('.')[0]

    # Open the rosbag
    with rosbag.Bag(bag_path, 'r') as bag:
        for topic, msg, t in bag.read_messages():
            if topic == '/xtion/rgb/image_rect_color':
                cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                img_name = os.path.join(color_dir, "color_case_" + case_num + ".png")
                cv2.imwrite(img_name, cv_img)
            elif topic == '/xtion/depth/image_rect':
                cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
                print(np.nanmin(cv_img))

                min_depth = np.nanmin(cv_img)
                max_depth = np.nanmax(cv_img)

                cv_img_normalized = ((cv_img - min_depth) / (max_depth - min_depth) * 255).astype(np.uint8)
                img_name = os.path.join(depth_dir, "depth_case_" + case_num + ".png")
                cv2.imwrite(img_name, cv_img_normalized)

print("Extraction complete!")

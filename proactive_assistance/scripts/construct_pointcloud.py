#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.point_cloud2 as pc2

class DepthToPointCloud:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('depth_to_pointcloud')

        # Create a CvBridge to convert ROS images to OpenCV format
        self.bridge = CvBridge()

        # Subscribe to the depth image and camera info topics
        self.image_sub = rospy.Subscriber('/xtion/depth/image_rect', Image, self.image_callback)
        self.info_sub = rospy.Subscriber('/xtion/depth/camera_info', CameraInfo, self.info_callback)

        # Publisher for the point cloud
        self.pc_pub = rospy.Publisher('/camera/depth/points', PointCloud2, queue_size=10)

        # Camera intrinsics will be stored here
        self.fx = 0
        self.fy = 0
        self.cx = 0
        self.cy = 0

    def info_callback(self, info_msg):
        # Extract the camera intrinsic parameters
        self.fx = info_msg.K[0]
        self.fy = info_msg.K[4]
        self.cx = info_msg.K[2]
        self.cy = info_msg.K[5]

    def image_callback(self, img_msg):
        rospy.loginfo('Received depth image')
        try:
            # Convert the ROS Image message to an OpenCV image
            depth_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            rospy.logerr('CvBridge Error: ', e)
            return

        # Convert depth image to a point cloud
        points = self.depth_image_to_point_cloud(depth_image)

        # Create a PointCloud2 message
        header = img_msg.header
        pc_msg = pc2.create_cloud_xyz32(header, points)

        # Publish the point cloud
        self.pc_pub.publish(pc_msg)

    def depth_image_to_point_cloud(self, depth_image):
        height, width = depth_image.shape
        points = []

        for v in range(height):
            for u in range(width):
                z = depth_image[v, u] / 1000.0  # Convert from mm to meters
                if z == 0:  # Skip invalid depth values
                    continue
                x = (u - self.cx) * z / self.fx
                y = (v - self.cy) * z / self.fy
                points.append((x, y, z))

        return points

if __name__ == '__main__':
    try:
        DepthToPointCloud()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

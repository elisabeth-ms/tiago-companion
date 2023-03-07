#! /usr/bin/env python

import os
import datetime
import numpy as np
import rospy
import cv2

from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
from numpy import nan


class DepthFilter(object):
    def __init__(self):
        
        rospy.loginfo("Initalizing add_depth_filter_node...")
        self.device_id = None
        self.activated = False
        self.max_distance = 1.2
        
        self.subscriber = rospy.Subscriber("/xtion/rgb/image_rect_color/compressed", CompressedImage, self.colorCallback,  queue_size = 5)
        # self.subscriber = rospy.Subscriber("/xtion/depth_registered/image", Image, self.depthCallback,  queue_size = 5)
        self.image_pub  = rospy.Publisher("/output/image", Image,queue_size=5)

        self.bridge = CvBridge()

        self.image_color = None
    def colorCallback(self, ros_data):
       

        #### direct conversion to CV2 ####
        self.image_color = self.bridge.compressed_imgmsg_to_cv2(ros_data,"rgb8")

        blank_image = self.image_color.copy()

        blank_image[0:220, :] = 0

        out_img = self.bridge.cv2_to_imgmsg(blank_image,"rgb8")
        out_img.header = ros_data.header
        self.image_pub.publish(out_img)		

        
        # cv2.imshow('cv_img', image_np)
        # cv2.waitKey(2)
    
    def depthCallback(self, ros_data):
        

        depth_image = None
        try:
            depth_image = self.bridge.imgmsg_to_cv2(ros_data, desired_encoding='32FC1')
            #self.image_pub.publish(self.bridge.cv2_to_imgmsg(image_np,"rgb8"))
            


            depth_image[np.isnan(depth_image)] = 1.1
            # for i in range(0,480):
            #     for j in range(0,640):
            #         # print(d)
            #         print(depth_image[i,j])
            cv2.imshow("depth",depth_image)
            cv2.waitKey(1)

            # print(depth_image)

            blank_image = self.image_color.copy()

            blank_image[depth_image >= 1.1] = 0

            # for i in range(0,480):
            #     for j in range(0,640):
            #         if(depth_image[i,j]/1000.0>self.max_distance):
            #            blank_image[i,j] = 255     

            # cv2.imshow('cv_img', depth_image)
            # ret,thresh1 = cv2.threshold(depth_image,1200.0,1200,cv2.THRESH_BINARY)
            out_img = self.bridge.cv2_to_imgmsg(blank_image,"rgb8")
            out_img.header = ros_data.header
            self.image_pub.publish(out_img)		

            # cv2.imshow("depth",thresh1)
            # cv2.waitKey(1)
        except CvBridgeError as e:
            print(e)
if __name__ == '__main__':
	rospy.init_node('add_depth_filter_node')
	depthFilter = DepthFilter()
	rospy.spin()


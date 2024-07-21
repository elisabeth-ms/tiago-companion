#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class BoundingBoxSelector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/xtion/rgb/image_raw', Image, self.image_callback)
        self.image = None
        self.selected_points = []

    def image_callback(self, data):
        # Convert the ROS Image message to OpenCV format
        self.image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

    def select_points(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.selected_points.append((x, y))
            rospy.loginfo("Point selected: {}".format((x, y)))
            if len(self.selected_points) == 2:
                # Draw bounding box on the image
                cv2.rectangle(self.image, self.selected_points[0], self.selected_points[1], (0, 255, 0), 2)
                cv2.imshow('Image', self.image)
                rospy.loginfo("Bounding box: {}".format(self.selected_points))

    def run(self):
        rospy.init_node('bounding_box_selector', anonymous=True)
        cv2.namedWindow('Image')
        cv2.setMouseCallback('Image', self.select_points)

        while not rospy.is_shutdown():
            if self.image is not None:
                cv2.imshow('Image', self.image)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        cv2.destroyAllWindows()

if __name__ == '__main__':
    selector = BoundingBoxSelector()
    selector.run()

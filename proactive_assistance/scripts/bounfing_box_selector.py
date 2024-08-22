#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from companion_msgs.msg import BoundingBoxesLabels, BoundingBox

class BoundingBoxSelector:
    def __init__(self):
        rospy.init_node('bounding_box_selector', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/xtion/rgb/image_raw', Image, self.image_callback)
        self.image = None
        self.selected_points = []
        self.publish_bboxes_cam = rospy.Publisher("/xtion/bounding_boxes", BoundingBoxesLabels, queue_size=20)
        self.bounding_box_msg = BoundingBoxesLabels()

        # Add a timer to publish the bounding box message at a regular interval
        rospy.Timer(rospy.Duration(0.1), self.publish_bounding_boxes)

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

                # Update the bounding box message
                boundingBox_msg = BoundingBox()
                boundingBox_msg.tlx = self.selected_points[0][0]
                boundingBox_msg.tly = self.selected_points[0][1]
                boundingBox_msg.brx = self.selected_points[1][0]
                boundingBox_msg.bry = self.selected_points[1][1]
                self.bounding_box_msg.bounding_boxes = [boundingBox_msg]
                self.bounding_box_msg.classes = ['jam']

                rospy.loginfo("Bounding box: {}".format(self.selected_points))
                self.selected_points = []

    def publish_bounding_boxes(self, event):
        if self.bounding_box_msg.bounding_boxes:
            self.publish_bboxes_cam.publish(self.bounding_box_msg)
            rospy.loginfo("Published bounding box: {}".format(self.bounding_box_msg))

    def run(self):
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
    rospy.sleep(1)  # Give time for the node to initialize
    selector.run()

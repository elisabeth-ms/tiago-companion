#!/usr/bin/env python
import rospy
import message_filters
from sensor_msgs.msg import Image
from companion_msgs.msg import BoundingBoxesLabels
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ImageBoundingBoxDrawer:
    def __init__(self):
        rospy.init_node('image_bounding_box_drawer', anonymous=True)

        self.bridge = CvBridge()

        # Create message filters
        image_sub = message_filters.Subscriber('/xtion/rgb/image_raw', Image)
        bboxes_sub = message_filters.Subscriber('/xtion/bounding_boxes', BoundingBoxesLabels)

        # Synchronize the topics by time
        ts = message_filters.ApproximateTimeSynchronizer([image_sub, bboxes_sub], 10, 0.1, allow_headerless=True)
        ts.registerCallback(self.callback)

        self.image_pub = rospy.Publisher('/xtion/annotated_image', Image, queue_size=10)

    def callback(self, image_msg, bboxes_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

        for bbox, label in zip(bboxes_msg.bounding_boxes, bboxes_msg.classes):
            pt1 = (int(bbox.tlx), int(bbox.tly))
            pt2 = (int(bbox.brx), int(bbox.bry))
            cv2.rectangle(cv_image, pt1, pt2, (0, 255, 0), 2)
            cv2.putText(cv_image, label, (pt1[0], pt1[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error while publishing: {0}".format(e))

if __name__ == '__main__':
    drawer = ImageBoundingBoxDrawer()
    rospy.spin()


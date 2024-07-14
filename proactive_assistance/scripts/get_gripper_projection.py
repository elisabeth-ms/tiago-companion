#!/usr/bin/env python
import rospy
import tf2_ros
import numpy as np
import cv2
from sensor_msgs.msg import CameraInfo,  Image
from cv_bridge import CvBridge, CvBridgeError

class GripperProjection:
    def __init__(self):
        rospy.init_node('gripper_projection_node')
        rospy.set_param('/use_sim_time', True)
        # Set up TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        # Initialize variables for camera matrix and distortion coefficients
        self.camera_matrix = None
        self.dist_coeffs = None
        self.ready_to_project = False  # Flag to check readiness

        # Subscribe to camera info
        camera_info_msg = rospy.wait_for_message("/xtion/rgb/camera_info", CameraInfo)
        self.camera_matrix = np.reshape(camera_info_msg.K, (3, 3))
        self.dist_coeffs = np.array(camera_info_msg.D)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/xtion/rgb/image_raw", Image, self.image_callback)
        self.image_pub = rospy.Publisher("/processed_image", Image, queue_size=1)

    def image_callback(self, data):
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                rospy.logerr("CvBridge Error: {0}".format(e))
                return

            point_2d = self.get_gripper_2d_projection((data.header.stamp))
            if point_2d is not None:
                # Draw a dot on the image at the projected point
                print("Drawing point on image")
                cv2.circle(cv_image, (int(point_2d[0]), int(point_2d[1])), 10, (0, 255, 0), -1)

            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            except CvBridgeError as e:
                rospy.logerr("CvBridge Error while publishing: {0}".format(e))        
    def get_gripper_2d_projection(self, stamp):
        try:
            print("Looking up transform...")
            trans = self.tf_buffer.lookup_transform('xtion_rgb_optical_frame','gripper_right_grasping_frame', stamp, rospy.Duration(1.0))
            print("Transform found.")
            translation = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
            point_3d = np.dot(self.camera_matrix, translation.reshape(3, 1))
            point_3d /= point_3d[2]  # Normalize the coordinates
            
             # Prepare the point in the correct shape: (N, 1, 2)
            points_2d = np.array([[point_3d[0, 0], point_3d[1, 0]]], dtype=np.float32).reshape(-1, 1, 2)

            # Undistort points
            point_2d_undistorted = cv2.undistortPoints(points_2d, self.camera_matrix, self.dist_coeffs, P=self.camera_matrix)
            rospy.loginfo("Projected 2D point: x=%.2f, y=%.2f", point_2d_undistorted[0][0][0], point_2d_undistorted[0][0][1])
            return point_2d_undistorted[0][0]
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            rospy.logwarn("Transform error: %s", ex)
            return None

    def run(self):
        rospy.loginfo("Starting projection calculations.")
        while not rospy.is_shutdown():
            point_2d = self.get_gripper_2d_projection()
            if point_2d is not None:
                rospy.sleep(1)  # Sleep to throttle the output or based on your application's needs


if __name__ == '__main__':
    projection = GripperProjection()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")



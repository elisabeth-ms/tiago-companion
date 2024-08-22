#!/usr/bin/env python

import rospy
import tf
import numpy as np
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseStamped, Point, PoseWithCovarianceStamped
import actionlib
import sensor_msgs.point_cloud2 as pc2
from companion_msgs.msg import BoundingBoxesLabels
from proactive_assistance.msg import NewWaypointAction, NewWaypointGoal, NewWaypointResult


class ObjectBasedNavigationServer:
    def __init__(self, name):
        self._action_name = name
        rospy.loginfo("Starting ObjectBasedNavigationServer")
        # Initialize TF listener
        self.tf_listener = tf.TransformListener()
        
        self._as = actionlib.SimpleActionServer(self._action_name, NewWaypointAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        
        self._enabled = True
        
        # Subscribe to object detection topic (for bounding box)
        self.object_detection_sub = rospy.Subscriber('/xtion/bounding_boxes', BoundingBoxesLabels, self.object_detection_callback)
        
        # Subscribe to point cloud topic
        self.point_cloud_sub = rospy.Subscriber('/xtion/depth_registered/points', PointCloud2, self.point_cloud_callback)
        
        self.amcl_pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_pose_callback)
        
        # Initialize bounding box and point cloud data
        self.bounding_box = None
        self.point_cloud = None
        self.desired_class = ""  # Desired object class
        # Offset to position the object near the robot's arm (adjust based on your robot's configuration)
        self.arm_offset = Point(0.0, 0.0, 0.0)
        
        self.closest_border_point = None
        self.previous_closest_border_point = None
        self.previous_class = ""
        self.amcl_pose = PoseStamped()
        
        # Define the table border points (example coordinates)
        self.table_border_points = [
            Point(x=-0.319864451885, y=1.15, z=0.0), 
            Point(x=1.34349691868, y=1.15, z=0.0)]


    def object_detection_callback(self, data):
        # Find the bounding box of the desired class
        for label, box in zip(data.classes, data.bounding_boxes):
            if label == self.desired_class:
                self.bounding_box = box
                break
    def amcl_pose_callback(self, data):
        # print("amcl_pose_callback")
        # print("data: ", data)
        
        self.amcl_pose.pose = data.pose.pose
        self.amcl_pose.header = data.header


    def point_cloud_callback(self, data):
        self.point_cloud = data

    def process_bounding_box(self, bounding_box):
        if self.point_cloud is None:
            rospy.logwarn("Point cloud data is not available")
            return
        
        print("len(self.point_cloud): ", len(self.point_cloud.data))
        
        center_x = int((bounding_box.tlx + bounding_box.brx) / 2)
        center_y = int((bounding_box.tly + bounding_box.bry) / 2)
        print("center_x: ", center_x)
        print("center_y: ", center_y)
        # Extract 3D point from point cloud at the center of the bounding box
        gen = pc2.read_points(self.point_cloud, field_names=("x", "y", "z"), skip_nans=True, uvs=[[center_x, center_y]])
        point_3d = next(gen, None)
        
        print("point_3d: ", point_3d)
        
        if point_3d is None:
            rospy.logwarn("No valid 3D point found at the center of the bounding box")
            return
        
        # Create PoseStamped for the 3D point in the camera frame
        object_pose_camera_frame = PoseStamped()
        object_pose_camera_frame.header.frame_id = self.point_cloud.header.frame_id
        object_pose_camera_frame.header.stamp = rospy.Time(0)
        object_pose_camera_frame.pose.position.x = point_3d[0]
        object_pose_camera_frame.pose.position.y = point_3d[1]
        object_pose_camera_frame.pose.position.z = point_3d[2]
        object_pose_camera_frame.pose.orientation.w = 1.0
        
        # Transform the point to the map frame
        object_pose_map_frame = self.transform_position(object_pose_camera_frame, "map")
        
        if object_pose_map_frame:
            # Calculate new goal position with offset
            goal_position = self.calculate_goal_position(object_pose_map_frame.pose.position)
            self.closest_border_point = self.find_closest_point_on_border(goal_position)


            
        
            
          

    def transform_position(self, pose, to_frame):
        try:
            self.tf_listener.waitForTransform(to_frame, pose.header.frame_id, rospy.Time(0), rospy.Duration(4.0))
            transformed_pose = self.tf_listener.transformPose(to_frame, pose)
            return transformed_pose
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Transformation failed: %s", str(e))
            return None

    def calculate_goal_position(self, object_position):
        goal_position = Point()
        goal_position.x = object_position.x - self.arm_offset.x
        goal_position.y = object_position.y - self.arm_offset.y
        goal_position.z = object_position.z - self.arm_offset.z
        return goal_position

    def execute_cb(self, request):
        
        rospy.loginfo("ObjectBasedNavigationServer: Received a goal update request")
        print("request: ", request)
        self.desired_class = request.object_name
        if self.previous_class!=self.desired_class:
            self.previous_class = self.desired_class
        
        if self.bounding_box is not None:
            self.process_bounding_box(self.bounding_box)
            print("bounding_box: ", self.bounding_box)
            
        print("closest_border_point: ", self.closest_border_point)
        
        if self.closest_border_point is None:
            rospy.logwarn("ObjectBasedNavigationServer: No valid goal position available")
            new_goal = NewWaypointResult()
            new_goal.current_pose = self.amcl_pose
            self._as.set_aborted()
        else:
            # Send the new goal
            new_goal = NewWaypointResult()
            new_goal.result = "new_goal"
            new_goal.target_pose = PoseStamped()
            new_goal.target_pose.header.stamp = rospy.Time.now()
            new_goal.target_pose.header.frame_id = "map"
            new_goal.target_pose.pose.position = self.closest_border_point
            new_goal.target_pose.pose.position.z = 0.0
            new_goal.target_pose.pose.orientation.x = 0.0
            new_goal.target_pose.pose.orientation.y = 0.0
            new_goal.target_pose.pose.orientation.z = 0.790255188966
            new_goal.target_pose.pose.orientation.w = 0.612777884974
            



            new_goal.current_pose = self.amcl_pose
            rospy.loginfo("ObjectBasedNavigationServer: New goal generated at x: %f, y: %f",
                              new_goal.target_pose.pose.position.x, new_goal.target_pose.pose.position.y)
            self.previous_closest_border_point = self.closest_border_point



            self._as.set_succeeded(new_goal)


    
    def find_closest_point_on_border(self, object_position):
        closest_point = None
        min_distance = float('inf')
        
        for i in range(len(self.table_border_points)):
            start_point = self.table_border_points[i]
            end_point = self.table_border_points[(i + 1) % len(self.table_border_points)]
            closest_point_on_segment = self.project_point_to_segment(object_position, start_point, end_point)
            distance = self.calculate_distance(object_position, closest_point_on_segment)
            if distance < min_distance:
                min_distance = distance
                closest_point = closest_point_on_segment
        
        return closest_point

    def project_point_to_segment(self, point, start, end):
        # Vector from start to end
        segment_vector = np.array([end.x - start.x, end.y - start.y, end.z - start.z])
        # Vector from start to the point
        point_vector = np.array([point.x - start.x, point.y - start.y, point.z - start.z])
        # Project point_vector onto segment_vector
        segment_length = np.dot(segment_vector, segment_vector)
        if segment_length == 0:
            return start
        projection = np.dot(point_vector, segment_vector) / segment_length
        projection = max(0, min(1, projection))
        # Calculate the closest point
        closest_point = Point(
            x=start.x + projection * segment_vector[0],
            y=start.y + projection * segment_vector[1],
            z=start.z + projection * segment_vector[2]
        )
        return closest_point

    def calculate_distance(self, point1, point2):
        return np.sqrt((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2 + (point1.z - point2.z) ** 2)


if __name__ == '__main__':
    rospy.init_node('object_based_navigation')
    obj_nav = ObjectBasedNavigationServer('goal_update')
    
    # Periodically process the bounding box and point cloud to send a new goal
    rate = rospy.Rate(5)  # Adjust the rate as needed

    rospy.spin()

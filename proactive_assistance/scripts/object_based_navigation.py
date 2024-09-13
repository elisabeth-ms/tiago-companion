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
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes

from sensor_msgs.msg import PointCloud2, CameraInfo
from sensor_msgs import point_cloud2
from image_geometry import PinholeCameraModel
from cv_bridge import CvBridge
from companion_msgs.srv import ObjectDetection, ObjectDetectionResponse

class ObjectBasedNavigationServer:
    def __init__(self, name):
        self._action_name = name
        rospy.loginfo("Starting ObjectBasedNavigationServer")
        # Initialize TF listener
        self.tf_listener = tf.TransformListener()
        
        self._as = actionlib.SimpleActionServer(self._action_name, NewWaypointAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        
        rospy.wait_for_service('object_detection/bounding_boxes')

        self._enabled = True
        
        camera_info_message = rospy.wait_for_message('/xtion/depth_registered/camera_info', CameraInfo)
        self.model_ = PinholeCameraModel()
        self.model_.fromCameraInfo(camera_info_message)
        
        print(self.model_)
        
        # Subscribe to object detection topic (for bounding box)
        # self.object_detection_sub = rospy.Subscriber('/object_detection/bounding_boxes', BoundingBoxes, self.object_detection_callback)
        
        depth_image_topic = '/xtion/depth_registered/image_raw'
        rospy.Subscriber(depth_image_topic, Image, self.depth_image_callback)
        
        self.bridge = CvBridge()

        
        # Subscribe to point cloud topic
        # self.point_cloud_sub = rospy.Subscriber('transformed_cloud', PointCloud2, self.point_cloud_callback)
        
        # # Lets wait for a message in the topic point_cloud
        # rospy.loginfo("Waiting for point cloud message")
        # msg = rospy.wait_for_message('transformed_cloud', PointCloud2)
        # rospy.loginfo("Point cloud message received")
        
        self.amcl_pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_pose_callback)
        
        # Initialize bounding box and point cloud data
        self.bounding_box = None
        self.point_cloud = None
        self.desired_class = ""  # Desired object class
        # Offset to position the object near the robot's arm (adjust based on your robot's configuration)
        self.arm_offset = Point(0.0, -0.15, 0.0)
        
        self.closest_border_point = None
        self.previous_closest_border_point = None
        self.previous_class = ""
        self.amcl_pose = PoseStamped()
        self.depth_image = None
        
        # Define the table border points (example coordinates)

        self.cloud_msg = PointCloud2()
        
        self.table_border_points = [
            Point(x=0.4, y= 0.677332573952, z=0.0), 
            Point(x=0.4, y= -0.481430256809, z=0.0)]



        
        rospy.loginfo("ObjectBasedNavigationServer: Initialized")

    # def object_detection_callback(self, data):
    #     # Find the bounding box of the desired class
    #     self.bounding_box = None
    #     for bbox in data.bounding_boxes:
    #         if bbox.Class == self.desired_class:
    #             self.bounding_box = bbox
    #             break
              
    def amcl_pose_callback(self, data):
        # print("amcl_pose_callback")
        # print("data: ", data)
        
        self.amcl_pose.pose = data.pose.pose
        self.amcl_pose.header = data.header


    def point_cloud_callback(self, data):
        self.point_cloud = data
    
    



    # Callback for depth image subscriber
    def depth_image_callback(self,depth_msg):
        # Convert depth image to OpenCV format using cv_bridge
        # print(depth_msg)
        self.depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')

        # print(self.depth_image)
        
        # for i in self.depth_image:
        #     print(i)
        # if not np.isnan(self.depth_image).any():
        #     print("Not nan")

        # Now you can process depth_image and convert it to a point cloud
        # self.cloud_msg = self.create_point_cloud(self.depth_image, depth_msg.header, self.model_)

        # Process or publish the cloud_msg as needed
        # You can add a publisher here to publish the point cloud:
        # point_cloud_pub.publish(cloud_msg)

    def create_point_cloud(self, depth_image, header, camera_model):
        # Get camera intrinsics from the model
        fx = camera_model.fx()
        fy = camera_model.fy()
        cx = camera_model.cx()
        cy = camera_model.cy()

        # Generate point cloud
        height, width = depth_image.shape
        points = []
        for v in range(height):
            for u in range(width):
                z = depth_image[v, u]
                if z == 0:
                    continue  # Skip zero depth points
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy
                points.append([x, y, z])

        # Create PointCloud2 message
        cloud_msg = point_cloud2.create_cloud_xyz32(header, points)
        return cloud_msg

    def process_bounding_box(self, bounding_box):
        
        if self.depth_image is None:
            rospy.logwarn("No depth image available")
            return
          
          
        # Bounding box coordinates
        xmin = int(bounding_box.xmin)
        xmax = int(bounding_box.xmax)
        ymin = int(bounding_box.ymin)
        ymax = int(bounding_box.ymax)
        
        rospy.loginfo("Bounding box coordinates: xmin: %d, xmax: %d, ymin: %d, ymax: %d", xmin, xmax, ymin, ymax)

        # Collect all points within the bounding box
        points = []
        for u in range(ymin, ymax + 1):
            for v in range(xmin, xmax + 1):
                depth = self.depth_image[u, v]
                if depth == 0 or np.isnan(depth):
                    continue
                point_3d = self.model_.projectPixelTo3dRay((v, u))
                if point_3d[0] != float('nan') and point_3d[1] != np.nan and point_3d[2] != np.nan:
                    point_3d = [coord*depth for coord in point_3d]
                    # print(point_3d)
                    points.append(point_3d)
                
              
                 


        if len(points) == 0:
            rospy.logwarn("No valid 3D points found within the bounding box")
            return

        # Calculate the average (mean) 3D point
        points = np.array(points)

        point_3d_mean = np.mean(points, axis=0)

        print("Mean 3D point within bounding box: ", point_3d_mean)
        
        # print("len(self.point_cloud): ", len(self.point_cloud.data))
        
        # center_x = int((bounding_box.xmin + bounding_box.xmax) / 2)
        # center_y = int((bounding_box.ymin + bounding_box.ymax) / 2)
        # print("center_x: ", center_x)
        # print("center_y: ", center_y)
        # # Extract 3D point from point cloud at the center of the bounding box
        # gen = pc2.read_points(self.point_cloud, field_names=("x", "y", "z"), skip_nans=True, uvs=[[center_x, center_y]])
        # point_3d = next(gen, None)
        
        # print("point_3d: ", point_3d)
        
        if point_3d_mean is None:
            rospy.logwarn("No valid 3D point found at the center of the bounding box")
            return
        
        # Create PoseStamped for the 3D point in the camera frame
        object_pose_camera_frame = PoseStamped()
        object_pose_camera_frame.header.frame_id = "xtion_rgb_optical_frame"
        object_pose_camera_frame.header.stamp = rospy.Time(0)
        object_pose_camera_frame.pose.position.x = point_3d_mean[0]
        object_pose_camera_frame.pose.position.y = point_3d_mean[1]
        object_pose_camera_frame.pose.position.z = point_3d_mean[2]
        object_pose_camera_frame.pose.orientation.w = 1.0
        
        object_pose_base_footprint_frame = self.transform_position(object_pose_camera_frame, "base_footprint")

        goal_pose = PoseStamped()
        if object_pose_base_footprint_frame:
            goal_pose = self.calculate_goal_position(object_pose_base_footprint_frame)
            goal_pose.header.stamp=rospy.Time(0)
        
        
        # Transform the point to the map frame
        object_pose_map_frame = self.transform_position(goal_pose, "map")
        
        print("object_pose_map_frame: ", object_pose_map_frame)
        
        other_object_pose_map_frame = self.transform_position(object_pose_camera_frame, "map")
        
        print("other: ", other_object_pose_map_frame)
        
        if object_pose_map_frame:

            goal_position= Point()
            goal_position.x = object_pose_map_frame.pose.position.x
            goal_position.y = object_pose_map_frame.pose.position.y

            
            # Calculate new goal position with offset
            self.closest_border_point = self.find_closest_point_on_border(goal_position)


            
        
            
          

    def transform_position(self, pose, to_frame):
        try:
            self.tf_listener.waitForTransform(to_frame, pose.header.frame_id, rospy.Time(0), rospy.Duration(4.0))
            transformed_pose = self.tf_listener.transformPose(to_frame, pose)
            return transformed_pose
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Transformation failed: %s", str(e))
            return None

    def calculate_goal_position(self, object_pose):
        goal_pose = object_pose
        print("object_pose: ", object_pose)
        goal_pose.pose.position.x = object_pose.pose.position.x - self.arm_offset.x
        goal_pose.pose.position.y = object_pose.pose.position.y - self.arm_offset.y
        print("goal_pose: ", goal_pose)
        return goal_pose

    def execute_cb(self, request):
        
        rospy.loginfo("ObjectBasedNavigationServer: Received a goal update request")
        print("request: ", request)
        self.bounding_box = None
        self.closest_border_point = None
        self.desired_class = request.object_name
        if self.previous_class!=self.desired_class:
            self.previous_class = self.desired_class
            
        
        if self.desired_class != 'none':
            # Lets call the service to get the bounding box
            response = None
            try:
                object_detection = rospy.ServiceProxy('object_detection/bounding_boxes', ObjectDetection)
                response = object_detection()
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s", e)
              
            # Find the bounding box of the desired class
            self.bounding_box = None
            print("response: ", response.boundingBoxes)
            for bbox in response.boundingBoxes.bounding_boxes:
                if bbox.Class == self.desired_class:
                    self.bounding_box = bbox
                    break
            
            
            print("Bounding box: ", self.bounding_box)
            if self.bounding_box is not None and self.depth_image is not None:
                print(self.bounding_box)
                print("we have depth image")
                self.process_bounding_box(self.bounding_box)
                print("bounding_box: ", self.bounding_box)
            else:
                print("No bounding box or depth image available")
                
            print("closest_border_point: ", self.closest_border_point)
        
        if self.closest_border_point is None or self.desired_class=='none':
            rospy.logwarn("ObjectBasedNavigationServer: No valid goal position available")
            new_goal = NewWaypointResult()
            new_goal.current_pose = self.amcl_pose
            new_goal.result = "no_goal"
            self._as.set_succeeded(new_goal)
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
            new_goal.target_pose.pose.orientation.z = 0.0367949234162
            new_goal.target_pose.pose.orientation.w = 0.999322837531



            new_goal.current_pose = self.amcl_pose
            rospy.loginfo("ObjectBasedNavigationServer: New goal generated at x: %f, y: %f",
                              new_goal.target_pose.pose.position.x, new_goal.target_pose.pose.position.y)
            self.previous_closest_border_point = self.closest_border_point
            self.closest_border_point = None
            self.bounding_box = None



            self._as.set_succeeded(new_goal)
            print("Sending new_goal: ", new_goal)


    
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
        # Vector from start to end (segment)
        segment_vector = np.array([end.x - start.x, end.y - start.y, end.z - start.z])
        
        # Vector from start to the external point
        point_vector = np.array([point.x - start.x, point.y - start.y, point.z - start.z])

        # Length of the segment squared (to avoid computing the square root)
        segment_length_squared = np.dot(segment_vector, segment_vector)
        
        # Handle case where the segment start and end are the same point
        if segment_length_squared == 0:
            return start  # The segment is a point

        # Projection factor: How far along the segment the projection lands
        projection_factor = np.dot(point_vector, segment_vector) / segment_length_squared
        
        # Clamp projection factor to ensure it stays on the segment (0 <= projection_factor <= 1)
        projection_factor = max(0, min(1, projection_factor))

        # Compute the closest point on the segment
        closest_point = Point(
            x=start.x + projection_factor * segment_vector[0],
            y=start.y + projection_factor * segment_vector[1],
            z=start.z + projection_factor * segment_vector[2]
        )
        
        return closest_point

    # def project_point_to_segment(self, point, start, end):
    #     # Vector from start to end
    #     segment_vector = np.array([end.x - start.x, end.y - start.y, end.z - start.z])
    #     # Vector from start to the point
    #     point_vector = np.array([point.x - start.x, point.y - start.y, point.z - start.z])
    #     # Project point_vector onto segment_vector
    #     segment_length = np.dot(segment_vector, segment_vector)
    #     if segment_length == 0:
    #         return start
    #     projection = np.dot(point_vector, segment_vector) / segment_length
    #     projection = max(0, min(1, projection))
    #     # Calculate the closest point
    #     closest_point = Point(
    #         x=start.x + projection * segment_vector[0],
    #         y=start.y + projection * segment_vector[1],
    #         z=start.z + projection * segment_vector[2]
    #     )
    #     return closest_point

    def calculate_distance(self, point1, point2):
        return np.sqrt((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2 + (point1.z - point2.z) ** 2)


if __name__ == '__main__':
    rospy.init_node('object_based_navigation')
    obj_nav = ObjectBasedNavigationServer('goal_update')
    
    # Periodically process the bounding box and point cloud to send a new goal
    rate = rospy.Rate(5)  # Adjust the rate as needed

    rospy.spin()

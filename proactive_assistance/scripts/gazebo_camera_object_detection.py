#! /usr/bin/env python
import numpy as np
import rospy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import DeleteModel, SpawnModel
import tf2_ros
import geometry_msgs.msg
from gazebo_ros_get_geometries_plugin.srv import GetGeometryOfModels, GetGeometryOfModelsResponse, GetGeometryOfModelsRequest
from visualization_msgs.msg import MarkerArray, Marker
from shape_msgs.msg import SolidPrimitive
import tf2_geometry_msgs
from sensor_msgs.msg import PointCloud, PointCloud2, CameraInfo, Image
from sensor_msgs import point_cloud2
from tf.transformations import quaternion_matrix
from std_msgs.msg import Header
import scipy
import PyKDL
import tf_conversions.posemath as pm
from gazebo_msgs.srv import SetModelState, SetModelStateRequest, SetModelStateResponse
import rospkg
import random
from companion_msgs.msg import BoundingBox, BoundingBoxesLabels

from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import csv
from std_srvs.srv import Empty
# from concaveHull import ConcaveHull
import time

class GetData(object):
    def __init__(self):


        rospy.loginfo("Initalizing gazebo_camera_object_detection node...")
        rospack = rospkg.RosPack()
        self.base_path = rospack.get_path('gaze_based_HRI_simulation')+'/data'
        self.object_path = rospack.get_path('companion_objects')+'/models'
        print(self.base_path)


        self.get_geometry_of_models = rospy.ServiceProxy(
            '/gazebo_get_geometries_plugin/get_geometries', GetGeometryOfModels)
        
        self.pause_physics = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause_physics = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)


        self.get_geometry_of_models.wait_for_service()
        print("Service get_geometries available")
        # self.set_model_state.wait_for_service()
        self.request_geometry_models = GetGeometryOfModelsRequest()
        self.request_geometry_models.not_wanted_models = [
            'tiago_dual', 'ground_plane', 'wall1', 'wall2', 'wall3', 'wall4', 'robot_table', 'user_table',
            'other_table', 'other_table2', 'cabinet', 'chair', 'closet1', 'closet1']
        self.camera_frame = 'xtion_rgb_optical_frame'
        #self.floating_camera_frame = 'floating_xtion_rgb_optical_frame'
        #self.floating_camera_model_name = 'xtion_pro_live'

        self.pc_wrt_camera_pub = rospy.Publisher(
            "point_cloud_wrt_robot_camera", PointCloud, queue_size=10)
        # self.pc_wrt_floatingcamera_pub = rospy.Publisher(
        #     "point_cloud_wrt_floating_camera", PointCloud, queue_size=10)

        self.unpause_physics.call()

        self.camera_info = rospy.wait_for_message(
            '/xtion/rgb/camera_info', CameraInfo)
        print(self.camera_info)
        self.models = rospy.wait_for_message('/gazebo/model_states', ModelStates)
        print(self.models)
        self.wanted_models = []
        for model in self.models.name:
            if model not in self.request_geometry_models.not_wanted_models:
                self.wanted_models.append(model)

        print(self.wanted_models)

        # cameras_parameters_csv_file = open(
        #     self.base_path+'/cameras_parameters.csv', 'w')
        # fieldnames = ['camera_name', 'focal_length_x',
        #               'focal_length_y', 'principal_point_x', 'principal_point_y']

        # writer = csv.DictWriter(
        #     cameras_parameters_csv_file, fieldnames=fieldnames)
        # writer.writeheader()
        # # writer.writerow({'camera_name': self.robots_camera_frame,
        # #                  'focal_length_x': self.robot_camera_info.P[0],
        # #                  'focal_length_y': self.robot_camera_info.P[5],
        # #                  'principal_point_x': self.robot_camera_info.P[2],
        # #                  'principal_point_y': self.robot_camera_info.P[6]})

        # writer.writerow({'camera_name': self.floating_camera_frame,
        #                  'focal_length_x': self.floating_camera_info.P[0],
        #                  'focal_length_y': self.floating_camera_info.P[5],
        #                  'principal_point_x': self.floating_camera_info.P[2],
        #                  'principal_point_y': self.floating_camera_info.P[6]})

        self.robot_camera_rgb_topic = '/xtion/rgb/image_raw'
        self.robot_camera_depth_topic = '/xtion/depth_registered/image_raw'
        # self.floating_camera_rgb_topic = '/floating_xtion/rgb/image_raw'
        # self.floating_camera_depth_topic = '/floating_xtion/depth_registered/image_raw'
        
        # self.available_models = ['apple_juice_box', 'can_sprite', 'cereal1', 'cereal2', 'cereal3', 'cocacola_can', 'nesquik', 'milk2', 'pringles_red', 'pringles_green', 'tea_box', 'tomato_sauce', 'biscuits']
        self.available_models = ['cocacola', 'pringles', 'milk', 'biscuits']
        self.bridge = CvBridge()
        # print(self.robot_camera_info)
        # print(self.floating_camera_info)
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.error = 0.005
        self.count_image = 1


        #self.image_sub = rospy.Subscriber('/xtion/rgb/image_raw', Image, self.image_callback)
        
        self.publish_bboxes_cam = rospy.Publisher("/xtion/bounding_boxes", BoundingBoxesLabels, queue_size=20)
        print("Initialization done")
        
        result = self.get_geometry_of_models(self.request_geometry_models)
        if not result.success:
            return False

        self.geometry_models = result.models_geometry_info
        
        self.pc_models_wrt_world = []
        self.classes = []
        for model_geometry_info in self.geometry_models:
            print("Model name: " + model_geometry_info.object_name)
            self.classes.append(model_geometry_info.object_name)
            
            if model_geometry_info.object_name  == 'cocacola':
                model_geometry_info.pose.position.z += model_geometry_info.shape.dimensions[0]/2
            elif model_geometry_info.object_name == 'pringles':
                model_geometry_info.pose.position.z += model_geometry_info.shape.dimensions[0]/2

            pc_model_wrt_world = self.createPointCloud(model_geometry_info.shape, 'world', model_geometry_info.pose, 3)
            self.pc_models_wrt_world.append(pc_model_wrt_world)

        while not rospy.is_shutdown():
            self.update(self.pc_models_wrt_world)
            rospy.sleep(0.01)

        
    def update(self, pc_models_wrt_world):
        boundingBoxes = BoundingBoxesLabels()
        nameObjects = []
        id = 0
        count = 0
        # start_time_transform = time.time()
        transform = self.get_transform(self.tf_buffer, 'world', self.camera_frame)
        if transform is None:
            return
        else:
            # start_time_all  = time.time()
            for pc_model_wrt_world in pc_models_wrt_world:
                # start_time = time.time()
                points_wrt_camera, _ = self.transform_point_cloud(pc_model_wrt_world, transform)
                # pc_model_wrt_camera = self.mat_to_point_cloud(points_wrt_camera, self.camera_frame)
                # ok, points_image, mean_distance = self.getPixelCoordinatesAllPointsAndMeanDistance(self.camera_info, '-XY', pc_model_wrt_camera)
                pixel_coords = self.convert_point_cloud_to_pixel_coordinates(self.camera_info, points_wrt_camera)
                if pixel_coords is not None:
                    # print("Pixel Coordinates:", pixel_coords)
                # else:
                #     print("Invalid or too few points.")
                # if ok:
                    bbox = self.computeBoundingBox(self.camera_info, pixel_coords)
                    # print(bbox)
                    boundingBox = BoundingBox()
                    boundingBox.id = id
                    boundingBox.tlx = bbox[0]
                    boundingBox.tly = bbox[1]
                    boundingBox.brx = bbox[2]
                    boundingBox.bry = bbox[3]
                    boundingBoxes.classes.append(self.classes[count])
                    boundingBoxes.bounding_boxes.append(boundingBox)
                    id+=1
                count+=1
                # update_time = time.time() - start_time
                # print("Time to update: ", update_time)
            # print("Time to update all: ", time.time() - start_time_all)
            boundingBoxes.header = Header()
            boundingBoxes.header.frame_id = self.camera_frame
            boundingBoxes.header.stamp = rospy.Time.now()
            self.publish_bboxes_cam.publish(boundingBoxes)

                    


    def setFloatingCameraPose(self,name, initial_floating_camera_rotation, max_pos, min_pos, max_RPY, min_RPY):
        # set floating camera pose
        x = random.uniform(min_pos[0], max_pos[0])
        y = random.uniform(min_pos[1], max_pos[1])
        z = random.uniform(min_pos[2], max_pos[2])

        print("Floating camera position: ", x,", ",y,", ",z)

        rotY = PyKDL.Rotation.RotY(random.uniform(min_RPY[1], max_RPY[1]))
        if y > 0:
            rotZ = PyKDL.Rotation.RotZ(random.uniform(min_RPY[2], 0))
        else:
            rotZ = PyKDL.Rotation.RotZ(random.uniform(0, max_RPY[2]))
        rot = initial_floating_camera_rotation*rotY
        rot = rot*rotZ
        q = rot.GetQuaternion()

        if not self.setModelPose(name, x, y, z, q[0], q[1], q[2], q[3]):
            return False

        else:
            return True


    

    def projectPointsInPlane(self, plane, pc):
        points = []
        if plane == 'XY':
            for point in pc.points:
                points.append([point.x, point.y])
        elif plane == 'YZ':
            for point in pc.points:
                points.append([point.y, point.z])
        elif plane == 'ZX':
            for point in pc.points:
                points.append([point.z, point.x])
        else:
            return False, None
            # points.append([0.0, 0.0])
            # points.append([0.0, 1.0])
            # points.append([1.0, 1.0])
            # points.append([1.0, 0.0])
        points = np.array(points)
        return True, points

    def compute2DConcaveHull(self, pts):
        ch = ConcaveHull()
        ch.loadpoints(pts)
        ch.calculatehull()
        boundary_points = np.vstack(ch.boundary.exterior.coords.xy).T
        print(boundary_points)
        return boundary_points

    def computeBoundingBox(self, camera_info, points_image):
        tlx = camera_info.width
        tly = camera_info.height
        brx = 0
        bry = 0
        for point in points_image:
            if point[0] < tlx:
                tlx = point[0]
            if point[0] > brx:
                brx = point[0]
            if point[1] < tly:
                tly = point[1]
            if point[1] > bry:
                bry = point[1]

        return (int(tlx), int(tly), int(brx), int(bry))

    def getPixelCoordinates(self, camera_info, plane, point3d):
        xpixel = 0
        ypixel = 0
        if point3d.z <= 0:
            return False, None, None
        if plane == '-XY':
            xpixel = point3d.x*camera_info.P[0]/point3d.z + camera_info.P[2]
            ypixel = point3d.y*camera_info.P[5]/point3d.z + camera_info.P[6]
        if xpixel > camera_info.width or xpixel < 0:
            xpixel = None
            return False, None, None
        if ypixel > camera_info.height or ypixel < 0:
            ypixel = None
            return False, None, None
        
        return True, xpixel, ypixel

    def getPixelCoordinatesAllPointsAndMeanDistance(self, camera_info, plane, pc):
        pointsImage = []
        mean = 0
        count_points = 0
        for point in pc.points:
            ok, xpixel, ypixel = self.getPixelCoordinates(
                camera_info, plane, point)
            print(ok, xpixel, ypixel)
            if ok:
                pointsImage.append([xpixel, ypixel])
        if len(pointsImage) > 4:
            pointsImage = np.array(pointsImage)
            return True, pointsImage, mean
        else:
            return False, None, None
    def get_pixel_coordinates_vectorized(self, camera_info, points):
        """
        Transform 3D points into pixel coordinates using camera intrinsic matrix.
        Points should be a NumPy array of shape (N, 3) where N is the number of points.
        """
        # Extract intrinsic parameters from camera_info
        fx, fy = camera_info.P[0], camera_info.P[5]
        cx, cy = camera_info.P[2], camera_info.P[6]

        # Filter out points that are behind the camera
        valid_indices = points[:, 2] > 0
        valid_points = points[valid_indices]

        # Compute pixel coordinates
        x_pixels = (valid_points[:, 0] * fx / valid_points[:, 2]) + cx
        y_pixels = (valid_points[:, 1] * fy / valid_points[:, 2]) + cy

        # Check bounds and validity
        valid_x = np.logical_and(x_pixels >= 0, x_pixels < camera_info.width)
        valid_y = np.logical_and(y_pixels >= 0, y_pixels < camera_info.height)
        overall_valid = np.logical_and(valid_x, valid_y)

        # Final valid pixel coordinates
        valid_x_pixels = x_pixels[overall_valid]
        valid_y_pixels = y_pixels[overall_valid]

        return np.column_stack((valid_x_pixels, valid_y_pixels))

    def convert_point_cloud_to_pixel_coordinates(self,camera_info, pc):
        """
        Convert a point cloud to pixel coordinates.
        pc should be a NumPy array of shape (N, 3).
        """
        # Convert point cloud to NumPy array if not already
        # points = np.array([(p.x, p.y, p.z) for p in pc.points])
        points = np.array(pc[:-1, :]).T  # Transpose to get x, y, z per row
        # Get pixel coordinates
        pixel_coordinates = self.get_pixel_coordinates_vectorized(camera_info, points)

        return pixel_coordinates if len(pixel_coordinates) > 4 else None
    # {
    #     ROS_INFO("p: %f %f %f ", p.x, p.y, p.z);
    #     // yInfo() << "intrinsics.focalLengthX:" << intrinsics.focalLengthX << " intrinsics.principalPointX: " << intrinsics.principalPointX;
    #     // yInfo() << "intrinsics.focalLengthY:" << intrinsics.focalLengthY << " intrinsics.principalPointY: " << intrinsics.principalPointY;

    #     ROS_INFO("focalLengthX_: %f principalPointX_: %f", focalLengthX_, principalPointX_);
    #     ROS_INFO("%f", p.x * focalLengthX_);
    #     xpixel = (int)(p.x * focalLengthX_ / p.z + principalPointX_);
    #     ypixel = (int)(p.y * focalLengthY_ / p.z + principalPointY_);
    #     ROS_INFO("xpixel: %d ypixel %d", xpixel, ypixel);

    #     // yInfo() << "xpixel: " << xpixel << "ypixel: " << ypixel;
    #     if (xpixel > width_)
    #         xpixel = width_;
    #     if (ypixel > height_)
    #         ypixel = height_;
    #     if (xpixel < 0)
    #         xpixel = 0;
    #     if (ypixel < 0)
    #         ypixel = 0;
    # }



    def createPointCloud(self, shape, frame, center_pose, number_of_points_per_axis):
        pc = PointCloud()
        pc.header.frame_id = frame
        matrix = quaternion_matrix([center_pose.orientation.x, center_pose.orientation.y,
                                   center_pose.orientation.z, center_pose.orientation.w])

        inv_matrix = np.linalg.inv(matrix)

        dimensions = [0, 0, 0]
        if shape.type == SolidPrimitive.BOX:
            dimensions[0] = shape.dimensions[0]
            dimensions[1] = shape.dimensions[1]
            dimensions[2] = shape.dimensions[2]
        if shape.type == SolidPrimitive.CYLINDER:
            dimensions[0] = 2*shape.dimensions[1]
            dimensions[1] = 2*shape.dimensions[1]
            dimensions[2] = shape.dimensions[0]
        elif shape.type == SolidPrimitive.SPHERE:
            dimensions[0] = 2*shape.dimensions[0]
            dimensions[1] = 2*shape.dimensions[0]
            dimensions[2] = shape.dimensions[0]

        for x in np.linspace(-dimensions[0]/2, dimensions[0]/2, number_of_points_per_axis, endpoint=True):
            for y in np.linspace(-dimensions[1]/2, dimensions[1]/2, number_of_points_per_axis, endpoint=True):
                for z in np.linspace(-dimensions[2]/2, dimensions[2]/2, number_of_points_per_axis, endpoint=True):
                    if shape.type == SolidPrimitive.CYLINDER:
                        if abs(np.sqrt(x**2 + y**2) - dimensions[0]/2) <= self.error:
                            vector = np.array([x, y, z, 1])
                            new_vector = matrix.dot(vector)
                            pc.points.append(geometry_msgs.msg.Point32(
                                new_vector[0]+center_pose.position.x, center_pose.position.y + new_vector[1],  center_pose.position.z + new_vector[2]))
                    else:
                        vector = np.array([x, y, z, 1])
                        new_vector = matrix.dot(vector)
                        pc.points.append(geometry_msgs.msg.Point32(
                            new_vector[0]+center_pose.position.x, center_pose.position.y + new_vector[1],  center_pose.position.z + new_vector[2]))
        return pc

    # convert a PointCloud or PointCloud2 to a 4xn scipy matrix (x y z 1)
    def point_cloud_to_mat(self, point_cloud):
        if type(point_cloud) == type(PointCloud()):
            points = [[p.x, p.y, p.z, 1] for p in point_cloud.points]
        elif type(point_cloud) == type(PointCloud2()):
            points = [[p[0], p[1], p[2], 1] for p in point_cloud2.read_points(
                point_cloud, field_names='xyz', skip_nans=True)]
        else:
            print("type not recognized:", type(point_cloud))
            return None
        points = scipy.matrix(points).T
        return points

    # convert a 4xn scipy matrix (x y z 1) to a PointCloud

    def mat_to_point_cloud(self, mat, frame_id):
        pc = PointCloud()
        pc.header.frame_id = frame_id
        for n in range(mat.shape[1]):
            column = mat[:, n]
            point = geometry_msgs.msg.Point32()
            point.x, point.y, point.z = column[0,
                                               0], column[1, 0], column[2, 0]
            pc.points.append(point)
        return pc

    # transform a PointCloud or PointCloud2 to be a 4xn scipy matrix (x y z 1) in a new frame
    def transform_point_cloud(self, point_cloud, transform):
        points = self.point_cloud_to_mat(point_cloud)
        points = transform * points
        return (points, transform)

    # get the 4x4 transformation matrix from frame1 to frame2 from TF

    def get_transform(self, tf_buffer, frame1, frame2):
      
          try:
              frame1_to_frame2 = tf_buffer.lookup_transform(frame2, frame1, rospy.Time(0), rospy.Duration(0.01))  # shorter timeout
              # use the transform as needed
              frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(frame1_to_frame2.transform.rotation.x, frame1_to_frame2.transform.rotation.y, frame1_to_frame2.transform.rotation.z, frame1_to_frame2.transform.rotation.w),
                            PyKDL.Vector(frame1_to_frame2.transform.translation.x, frame1_to_frame2.transform.translation.y, frame1_to_frame2.transform.translation.z))
              # print(pm.toMatrix(frame))
              return scipy.matrix(pm.toMatrix(frame))
          except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
              rospy.logwarn("Transform lookup failed: {}".format(e))
              return None



    def transfrom_point_cloud_from_pose(self, point_cloud, pose_frame2_wrt_frame1):
        points = self.point_cloud_to_mat(point_cloud)
        transform = self.get_transform_from_pose(pose_frame2_wrt_frame1)
        # if transform == None:
        #     return (None, None)
        points = transform * points
        return (points, transform)

    def get_transform_from_pose(self, pose_frame2_wrt_frame1):
        frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(pose_frame2_wrt_frame1.orientation.x, pose_frame2_wrt_frame1.orientation.y, pose_frame2_wrt_frame1.orientation.z, pose_frame2_wrt_frame1.orientation.w),
                            PyKDL.Vector(pose_frame2_wrt_frame1.position.x, pose_frame2_wrt_frame1.position.y, pose_frame2_wrt_frame1.position.z))
        # print(pm.toMatrix(frame))
        return scipy.matrix(pm.toMatrix(frame))

    def init(self):
        result = self.get_geometry_of_models(self.request_geometry_models)
        if not result.success:
            return False

        self.geometry_models = result.models_geometry_info

        pc_models_wrt_world = []
        
        for model_geometry_info in self.geometry_models:
            print("Model name: " + model_geometry_info.object_name)

            dimensions = [0,0,0]
            if len(model_geometry_info.shape.dimensions) == 3:
                dimensions = model_geometry_info.shape.dimensions
            elif len(model_geometry_info.shape.dimensions) == 2:
                dimensions = [model_geometry_info.shape.dimensions[0], model_geometry_info.shape.dimensions[1], 0]
            elif len(model_geometry_info.shape.dimensions) == 1:
                dimensions = [model_geometry_info.shape.dimensions[0], 0, 0]
            # writer_models_geometry.writerow({'object_name': model_geometry_info.object_name,
            #                                     'base_frame': 'base_footprint', 'x': model_geometry_info.pose.position.x,
            #                                     'y': model_geometry_info.pose.position.y,
            #                                     'z': model_geometry_info.pose.position.z,
            #                                     'qx': model_geometry_info.pose.orientation.x,
            #                                     'qy': model_geometry_info.pose.orientation.y,
            #                                     'qz': model_geometry_info.pose.orientation.z,
            #                                     'qw': model_geometry_info.pose.orientation.w,
            #                                     'shape_type': model_geometry_info.shape.type,
            #                                     'dimension0': dimensions[0],
            #                                     'dimension1': dimensions[1],
                                                # 'dimension2': dimensions[2]})
            
            pc_model_wrt_world = self.createPointCloud(
                model_geometry_info.shape, 'world', model_geometry_info.pose, 3)
            pc_models_wrt_world.append(pc_model_wrt_world)
        return pc_models_wrt_world
            
            
        
        

if __name__ == '__main__':
    rospy.init_node('get_data_from_simulation')
    getData = GetData()
    # count = 0

        # count+=1
        # if count == 1:
        #     break
    # rospy.spin()
#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import ros_numpy
import numpy as np
from sensor_msgs import point_cloud2
import plyfile

def callback(data):
    print("Received a pointcloud")
    data_out = list(point_cloud2.read_points(data))
    print(data_out)
    
    labels = []
    index_label = -1
    data_array = np.array(data_out)
    x = []
    for i in range(data_array.shape[0]):
        if index_label == -1:
            index_label += 1
            labels.append(data_array[i][3])
            x = np.array([(data_array[0][0], data_array[0][1], data_array[0][2])], dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])

        if data_array[i][3] != labels[index_label]:
            print("Label changed")
            index_label += 1
            labels.append(data_array[i][3])
            name_file = "pointcloud_" + str(int(labels[index_label-1])) + ".ply"
            plyfile.PlyData([plyfile.PlyElement.describe(x, 'vertex')], text=True).write(name_file)
            x = np.array([(data_array[i][0], data_array[i][1], data_array[i][2])], dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])

        x = np.append(x, np.array([(data_array[i][0], data_array[i][1], data_array[i][2])], dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')]))
        if i == data_array.shape[0]-1:
            name_file = "pointcloud_" + str(int(labels[index_label])) + ".ply"
            plyfile.PlyData([plyfile.PlyElement.describe(x, 'vertex')], text=True).write(name_file)

if __name__ == '__main__':
	rospy.init_node('store_ros_pointcloud')
	rospy.Subscriber("/constrained_superquadrics/pointcloud", PointCloud2, callback)

	rospy.spin()

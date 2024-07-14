#! /usr/bin/env python

import numpy as np
import rospy
from gazebo_msgs.msg import LinkStates
import tf2_ros
import geometry_msgs.msg
import tf

class LinkStateToTF(object):
    def __init__(self):

        rospy.loginfo("Initalizing link_state_to_tf_node...")
        self.device_id = None
        self.activated = False
        self.max_distance = 1.2
        self.floating_camera_link_str = 'tiago_dual::base_footprint'
        
        self.tf_frame_str = 'base_footprint'
        # self.base_link_str = 'tiago_dual::base_footprint2'
        # self.tf_frame_base_link_str = 'base_link'
        self.subscriber = rospy.Subscriber(
            "/gazebo/link_states", LinkStates, self.linkStateCallback,  queue_size=10)
        self.transformations = [
            geometry_msgs.msg.TransformStamped()]

        self.image_color = None



    def linkStateCallback(self, data):
        linkIndexes = [-1]
        linkIndexes = [data.name.index(
            self.floating_camera_link_str)]
        br = tf2_ros.TransformBroadcaster()

        for i in range(0, len(linkIndexes)):
            if linkIndexes[i] > -1:
                link_pose = data.pose[linkIndexes[i]]
                t = geometry_msgs.msg.TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = "world"

                t.child_frame_id = self.tf_frame_str
                t.transform.translation.x = link_pose.position.x 
                t.transform.translation.y = link_pose.position.y
                t.transform.translation.z = link_pose.position.z 
                t.transform.rotation.x = link_pose.orientation.x
                t.transform.rotation.y = link_pose.orientation.y
                t.transform.rotation.z = link_pose.orientation.z
                t.transform.rotation.w = link_pose.orientation.w
                
                transform = tf.transformations.concatenate_matrices(
                tf.transformations.translation_matrix([t.transform.translation.x,
                                               t.transform.translation.y,
                                               t.transform.translation.z]),
                tf.transformations.quaternion_matrix([t.transform.rotation.x,
                                              t.transform.rotation.y,
                                              t.transform.rotation.z,
                                              t.transform.rotation.w])
                )

                # Compute inverse
                inverse_transform = tf.transformations.inverse_matrix(transform)

                # Extract translation and rotation from the inverse matrix
                inv_translation = tf.transformations.translation_from_matrix(inverse_transform)
                inv_quaternion = tf.transformations.quaternion_from_matrix(inverse_transform)

                # Create a new TransformStamped for the inverse transform
                t_inv = geometry_msgs.msg.TransformStamped()
                t_inv.transform.translation.x = inv_translation[0]
                t_inv.transform.translation.y = inv_translation[1]
                t_inv.transform.translation.z = inv_translation[2]
                t_inv.transform.rotation.x = inv_quaternion[0]
                t_inv.transform.rotation.y = inv_quaternion[1]
                t_inv.transform.rotation.z = inv_quaternion[2]
                t_inv.transform.rotation.w = inv_quaternion[3]
                t_inv.header.stamp = rospy.Time.now()
                t_inv.header.frame_id = self.tf_frame_str
                t_inv.child_frame_id = "world"
                self.transformations[i] = t_inv
        br.sendTransform(self.transformations[0])
        # br.sendTransform(self.transformations[1])
if __name__ == '__main__':
    rospy.init_node('link_state_to_tf_node')
    linkStateToTF = LinkStateToTF()
    rospy.spin()

    while rospy.is_shutdown() == False:
        linkStateToTF.update()
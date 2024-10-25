#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
import tf2_ros
import geometry_msgs.msg
import tf.transformations

class OptiTrackToTF:
    def __init__(self):
        # Get parameters from the ROS parameter server
        self.offset_x = rospy.get_param('~offset_x', -0.07)
        self.offset_y = rospy.get_param('~offset_y', 0.0)
        self.offset_z = rospy.get_param('~offset_z', 0.0)
        self.optitrack_topic = rospy.get_param('~optitrack_topic', '/vrpn_client_node/L1/pose')
        self.world_frame = rospy.get_param('~world_frame', 'world')
        self.robot_frame = rospy.get_param('~robot_frame', 'base_link')

        rospy.init_node('optitrack_to_tf_with_offset')

        self.br = tf2_ros.TransformBroadcaster()
        # ROS_INFO("OptiTrack to TF with offset node started")

        rospy.Subscriber(self.optitrack_topic, PoseStamped, self.pose_callback)

    def apply_offset(self, pose_msg):
        orientation = pose_msg.pose.orientation
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)

        rotation_matrix = tf.transformations.quaternion_matrix(quaternion)

        offset_in_robot_frame = [self.offset_x, self.offset_y, self.offset_z, 1.0]  # Homogeneous coordinates

        offset_in_world_frame = rotation_matrix.dot(offset_in_robot_frame)

        corrected_x = pose_msg.pose.position.x + offset_in_world_frame[0]
        corrected_y = pose_msg.pose.position.y + offset_in_world_frame[1]
        corrected_z = pose_msg.pose.position.z + offset_in_world_frame[2]

        return corrected_x, corrected_y, corrected_z

    def pose_callback(self, pose_msg):
        corrected_x, corrected_y, corrected_z = self.apply_offset(pose_msg)

        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.world_frame 
        t.child_frame_id = self.robot_frame

        t.transform.translation.x = corrected_x
        t.transform.translation.y = corrected_y
        t.transform.translation.z = corrected_z

        t.transform.rotation.x = pose_msg.pose.orientation.x
        t.transform.rotation.y = pose_msg.pose.orientation.y
        t.transform.rotation.z = pose_msg.pose.orientation.z
        t.transform.rotation.w = pose_msg.pose.orientation.w

        self.br.sendTransform(t)

    def start(self):
        rospy.spin()

# Example usage:
if __name__ == '__main__':
    optitrack_to_tf = OptiTrackToTF()
    optitrack_to_tf.start()
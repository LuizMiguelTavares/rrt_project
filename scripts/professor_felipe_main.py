#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import tf.transformations as tf

class PoseAdjuster:
    def __init__(self):
        rospy.init_node("vrpn_pose_adjuster")

        # Parameters
        self.odom_topic = rospy.get_param("~odom_topic", "/L1/odom")
        self.pose_topic = rospy.get_param("~pose_topic", "/vrpn_client_node/L1/pose")
        self.adjusted_pose_topic = rospy.get_param("~adjusted_pose_topic", "/vrpn_adjusted_pose")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")

        # Subscribers
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
        rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_callback)

        # Publisher
        self.adjusted_pose_pub = rospy.Publisher(self.adjusted_pose_topic, PoseStamped, queue_size=10)

        # Data storage
        self.odom_initial = None
        self.pose_initial = None
        self.orientation_offset = None

    def odom_callback(self, msg):
        if self.odom_initial is None:
            # Store initial odom pose
            self.odom_initial = msg.pose.pose
        self.odom = msg

    def pose_callback(self, msg):
        if self.pose_initial is None:
            # Store initial vrpn pose
            self.pose_initial = msg.pose

            # Compute initial orientation offset
            odom_quat = [self.odom_initial.orientation.x, self.odom_initial.orientation.y,
                         self.odom_initial.orientation.z, self.odom_initial.orientation.w]
            pose_quat = [self.pose_initial.orientation.x, self.pose_initial.orientation.y,
                         self.pose_initial.orientation.z, self.pose_initial.orientation.w]
            self.orientation_offset = tf.quaternion_multiply(
                tf.quaternion_inverse(pose_quat),
                odom_quat
            )

        if self.odom_initial is not None and self.pose_initial is not None and self.orientation_offset is not None:
            # Adjust pose by subtracting initial offset
            adjusted_pose = PoseStamped()
            adjusted_pose.header = msg.header
            adjusted_pose.header.frame_id = self.odom_frame  # Publish in the odom frame

            # Compute adjusted position
            adjusted_pose.pose.position.x = msg.pose.position.x - self.pose_initial.position.x + self.odom_initial.position.x
            adjusted_pose.pose.position.y = msg.pose.position.y - self.pose_initial.position.y + self.odom_initial.position.y
            adjusted_pose.pose.position.z = msg.pose.position.z - self.pose_initial.position.z + self.odom_initial.position.z

            # Compute adjusted orientation
            pose_quat = [msg.pose.orientation.x, msg.pose.orientation.y,
                         msg.pose.orientation.z, msg.pose.orientation.w]
            adjusted_quat = tf.quaternion_multiply(self.orientation_offset, pose_quat)

            adjusted_pose.pose.orientation.x = adjusted_quat[0]
            adjusted_pose.pose.orientation.y = adjusted_quat[1]
            adjusted_pose.pose.orientation.z = adjusted_quat[2]
            adjusted_pose.pose.orientation.w = adjusted_quat[3]

            # Publish adjusted pose
            self.adjusted_pose_pub.publish(adjusted_pose)

if __name__ == "__main__":
    try:
        PoseAdjuster()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
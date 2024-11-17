#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped

class PathPublisher:
    def __init__(self):
        rospy.init_node("path_publisher")

        # Parameters
        self.odom_topic = rospy.get_param("~odom_topic", "/L1/odom")
        self.adjusted_pose_topic = rospy.get_param("~adjusted_pose_topic", "/vrpn_adjusted_pose")
        self.odom_path_topic = rospy.get_param("~odom_path_topic", "/odom_path")
        self.adjusted_path_topic = rospy.get_param("~adjusted_path_topic", "/adjusted_path")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")

        # Publishers
        self.odom_path_pub = rospy.Publisher(self.odom_path_topic, Path, queue_size=10)
        self.adjusted_path_pub = rospy.Publisher(self.adjusted_path_topic, Path, queue_size=10)

        # Paths
        self.odom_path = Path()
        self.adjusted_path = Path()
        self.odom_path.header.frame_id = self.odom_frame
        self.adjusted_path.header.frame_id = self.odom_frame

        # Subscribers
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
        rospy.Subscriber(self.adjusted_pose_topic, PoseStamped, self.adjusted_pose_callback)

    def odom_callback(self, msg):
        # Add the current odom pose to the path
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        self.odom_path.header.stamp = rospy.Time.now()
        self.odom_path.poses.append(pose)

        # Publish the odom path
        self.odom_path_pub.publish(self.odom_path)

    def adjusted_pose_callback(self, msg):
        # Add the current adjusted pose to the path
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose

        self.adjusted_path.header.stamp = rospy.Time.now()
        self.adjusted_path.poses.append(pose)

        # Publish the adjusted path
        self.adjusted_path_pub.publish(self.adjusted_path)

if __name__ == "__main__":
    try:
        PathPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

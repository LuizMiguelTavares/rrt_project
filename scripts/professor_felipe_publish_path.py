#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class PathPublisher:
    def __init__(self):
        rospy.init_node("path_publisher")

        # Parameters
        self.odom_topic = rospy.get_param("~odom_topic", "/L1/odom")
        self.vrpn_topic = rospy.get_param("~vrpn_topic", "/vrpn_adjusted_pose")
        self.odom_path_topic = rospy.get_param("~odom_path_topic", "/odom_path")
        self.vrpn_path_topic = rospy.get_param("~vrpn_path_topic", "/vrpn_path")
        self.frame_id = rospy.get_param("~frame_id", "odom")
        self.publish_rate = rospy.get_param("~publish_rate", 5.0)  # Frequency in Hz

        self.odom_pose_true = False
        self.vrpn_pose_true = False

        # Subscribers
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
        rospy.Subscriber(self.vrpn_topic, PoseStamped, self.vrpn_callback)

        # Publishers
        self.odom_path_pub = rospy.Publisher(self.odom_path_topic, Path, queue_size=10)
        self.vrpn_path_pub = rospy.Publisher(self.vrpn_path_topic, Path, queue_size=10)

        # Path messages
        self.odom_path = Path()
        self.odom_path.header.frame_id = self.frame_id

        self.vrpn_path = Path()
        self.vrpn_path.header.frame_id = self.frame_id

        # Timer for periodic publishing
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.publish_paths)

    def odom_callback(self, msg):
        # Extract the pose from Odometry and mark it as received
        self.odom_pose = msg.pose.pose
        self.odom_pose_true = True

    def vrpn_callback(self, msg):
        # Extract the pose from PoseStamped and mark it as received
        self.vrpn_pose = msg.pose
        self.vrpn_pose_true = True

    def publish_paths(self, event):
        if not self.odom_pose_true or not self.vrpn_pose_true:
            return
        
        current_time = rospy.Time.now()

        # Create PoseStamped for odom_pose
        odom_pose_stamped = PoseStamped()
        odom_pose_stamped.header.stamp = current_time
        odom_pose_stamped.header.frame_id = self.frame_id
        odom_pose_stamped.pose = self.odom_pose
        self.odom_path.poses.append(odom_pose_stamped)

        # Create PoseStamped for vrpn_pose
        vrpn_pose_stamped = PoseStamped()
        vrpn_pose_stamped.header.stamp = current_time
        vrpn_pose_stamped.header.frame_id = self.frame_id
        vrpn_pose_stamped.pose = self.vrpn_pose
        self.vrpn_path.poses.append(vrpn_pose_stamped)

        # Update the header timestamps for the paths
        self.odom_path.header.stamp = current_time
        self.vrpn_path.header.stamp = current_time

        # Publish the paths
        self.odom_path_pub.publish(self.odom_path)
        self.vrpn_path_pub.publish(self.vrpn_path)

if __name__ == "__main__":
    try:
        PathPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
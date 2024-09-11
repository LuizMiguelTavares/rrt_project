#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class PathPublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('path_publisher')

        self.is_path_on = False
        self.pose_topic = rospy.get_param('~pose_topic', '/vrpn_client_node/P1/pose')
        self.path_topic = rospy.get_param('~path_topic', '/traveled_path')
        self.rate = rospy.get_param('~rate', 10)

        self.pose_sub = rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_callback)
        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=10)

        self.path_msg = Path()
        self.pose = PoseStamped()
        self.pose_old = PoseStamped()
        self.path_msg.header.frame_id = "map"  # Set the frame ID according to your setup

        self.rate_obj = rospy.Rate(self.rate)

        # Main loop
        self.run()

    def pose_callback(self, pose_msg):
        self.pose = pose_msg

        if not self.is_path_on:
            self.path_msg.poses.append(self.pose)
            self.pose_old = self.pose
            self.is_path_on = True

    def run(self):
        while not rospy.is_shutdown():
            if not self.is_path_on:
                self.rate_obj.sleep()
                continue

            if not (self.pose == self.pose_old):
                self.path_msg.header.stamp = rospy.Time.now()
                self.path_msg.poses.append(self.pose)
                self.pose_old = self.pose

            self.path_pub.publish(self.path_msg)

            self.rate_obj.sleep()

if __name__ == '__main__':
    try:
        # Instantiate the PathPublisher and keep the node alive
        PathPublisher()
    except rospy.ROSInterruptException:
        pass

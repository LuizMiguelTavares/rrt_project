#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import math

def generate_circular_path(radius, num_points):
    path = Path()
    path.header.frame_id = "map"

    for i in range(num_points):
        angle = 2 * math.pi * i / num_points
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0
        path.poses.append(pose)

    return path

def publish_path():
    rospy.init_node('circular_path_publisher', anonymous=True)
    path_pub = rospy.Publisher('/circular_path', Path, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    radius = rospy.get_param('~radius', 1.0)
    num_points = rospy.get_param('~num_points', 100)

    path = generate_circular_path(radius, num_points)

    while not rospy.is_shutdown():
        path.header.stamp = rospy.Time.now()
        for pose in path.poses:
            pose.header.stamp = rospy.Time.now()
        path_pub.publish(path)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_path()
    except rospy.ROSInterruptException:
        pass
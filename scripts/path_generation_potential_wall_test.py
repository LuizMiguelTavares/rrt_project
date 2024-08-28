#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def generate_discretized_path(num_points):
    path = Path()
    path.header.frame_id = "odom"

    for i in range(num_points + 1):
        x = 5.0 * (i / float(num_points))

        pose = PoseStamped()
        pose.header.frame_id = "odom"
        pose.pose.position.x = x
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0

        path.poses.append(pose)

    return path

def main():
    rospy.init_node('path_generator_node', anonymous=True)

    # Get parameters for discretization and publishing frequency
    frequency = rospy.get_param('~frequency', 1.0)  # Default frequency: 1 Hz
    num_points = rospy.get_param('~num_points', 100)  # Default: 100 points

    path_pub = rospy.Publisher('/rrt_path', Path, queue_size=10)

    path = generate_discretized_path(num_points)

    rate = rospy.Rate(frequency)

    while not rospy.is_shutdown():
        path.header.stamp = rospy.Time.now()

        for pose in path.poses:
            pose.header.stamp = rospy.Time.now()

        path_pub.publish(path)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
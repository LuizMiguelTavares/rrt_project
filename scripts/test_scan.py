#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
import tf

class LidarBallMarker:
    def __init__(self):
        rospy.init_node('lidar_ball_marker', anonymous=True)

        self.lidar_sub = rospy.Subscriber('/laser/scan', LaserScan, self.lidar_callback)
        self.marker_pub = rospy.Publisher('/ball_marker', Marker, queue_size=10)
        self.listener = tf.TransformListener()

        self.marker = Marker()
        self.marker.ns = "lidar"
        self.marker.id = 0
        self.marker.type = Marker.SPHERE
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.1
        self.marker.scale.z = 0.1
        self.marker.color.a = 1.0
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0

    def lidar_callback(self, data):
        angle_index = 0  # 0 degrees
        distance = data.ranges[angle_index]
        
        print(data.angle_min)
        print(data.angle_max)
        print(data.angle_increment)
        print("/n")
        if distance == float('inf') or distance != distance:  # Handle NaN and inf values
            rospy.logwarn("Invalid laser scan range at 0 degrees")
            return

        # Convert polar to Cartesian coordinates
        x = distance
        y = 0.0

        self.marker.header.stamp = rospy.Time.now()
        self.marker.header.frame_id = data.header.frame_id
        self.marker.pose.position.x = -x
        self.marker.pose.position.y = y
        self.marker.pose.position.z = 0.0
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0

        self.marker_pub.publish(self.marker)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = LidarBallMarker()
        node.run()
    except rospy.ROSInterruptException:
        pass
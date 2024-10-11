#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
import math
import tf.transformations as tf_trans

class ArrowPublisher:
    def __init__(self):
        rospy.init_node('arrow_publisher', anonymous=True)

        self.pose_sub = rospy.Subscriber('/vrpn_client_node/L1/pose', PoseStamped, self.pose_callback)
        self.point_sub = rospy.Subscriber('/potential', Point, self.point_callback)

        self.marker_pub = rospy.Publisher('/arrow_marker_test', Marker, queue_size=10)

        self.latest_pose = None
        self.latest_point = None

        self.rate = rospy.Rate(10)

    def pose_callback(self, msg):
        self.latest_pose = msg
        self.publish_arrow()

    def point_callback(self, msg):
        self.latest_point = msg
        self.publish_arrow()

    def publish_arrow(self):
        if self.latest_pose is None or self.latest_point is None:
            return

        pose = self.latest_pose
        point = self.latest_point

        start_x = pose.pose.position.x
        start_y = pose.pose.position.y
        start_z = pose.pose.position.z

        orientation = pose.pose.orientation
        quaternion = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        )
        euler = tf_trans.euler_from_quaternion(quaternion)
        yaw = euler[2]

        rotated_x = point.x * math.cos(yaw) - point.y * math.sin(yaw)
        rotated_y = point.x * math.sin(yaw) + point.y * math.cos(yaw)
        rotated_z = point.z 

        end_x = start_x + rotated_x
        end_y = start_y + rotated_y
        end_z = start_z + rotated_z

        marker = Marker()
        marker.header.frame_id = pose.header.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "arrows"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.points = [
            Point(start_x, start_y, start_z),
            Point(end_x, end_y, end_z)
        ]

        marker.scale.x = 0.02  # Diâmetro do corpo da seta (shaft)
        marker.scale.y = 0.04  # Diâmetro da ponta da seta (head)
        marker.scale.z = 0.02  # Comprimento da ponta da seta (head)

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Define a cor da seta (RGBA)
        marker.color.a = 1.0  # Opacidade
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        # Publica o Marker
        self.marker_pub.publish(marker)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        arrow_publisher = ArrowPublisher()
        arrow_publisher.run()
    except rospy.ROSInterruptException:
        pass
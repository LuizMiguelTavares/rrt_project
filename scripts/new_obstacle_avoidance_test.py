#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
import math

class ObstacleAvoidance:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('obstacle_avoidance')

        # Subscribe to the LiDAR topic
        self.lidar_sub = rospy.Subscriber('/laser/scan', LaserScan, self.lidar_callback)

        # Publishers for visualization
        self.quadrants_pub = rospy.Publisher('/quadrants', MarkerArray, queue_size=10)
        self.closest_points_pub = rospy.Publisher('/closest_points', MarkerArray, queue_size=10)

        # Initialize a list to store closest points in Cartesian coordinates
        self.closest_points = []

    def lidar_callback(self, msg):
        # Number of LiDAR points
        n_points = len(msg.ranges)
        
        # Number of points per quadrant
        points_per_quadrant = n_points // 6
        
        # Reset the closest points list
        self.closest_points = []

        # Marker array for quadrants and closest points
        quadrants_marker_array = MarkerArray()
        closest_points_marker_array = MarkerArray()

        # Colors for quadrants
        colors = [
            (1.0, 0.0, 0.0),  # Red
            (0.0, 1.0, 0.0),  # Green
            (0.0, 0.0, 1.0),  # Blue
            (1.0, 1.0, 0.0),  # Yellow
            (1.0, 0.0, 1.0),  # Magenta
            (0.0, 1.0, 1.0)   # Cyan
        ]

        # Process each quadrant
        for i in range(6):
            start_index = i * points_per_quadrant
            end_index = (i + 1) * points_per_quadrant if i < 5 else n_points

            # Get the points in the current quadrant
            quadrant_points = msg.ranges[start_index:end_index]

            # Find the closest point in the current quadrant
            closest_point = min(quadrant_points)
            closest_index = quadrant_points.index(closest_point) + start_index

            # Convert polar coordinates (range, angle) to Cartesian coordinates (x, y)
            angle = msg.angle_min + closest_index * msg.angle_increment
            x = closest_point * math.cos(angle)
            y = closest_point * math.sin(angle)

            # Store the closest point in Cartesian coordinates
            self.closest_points.append((x, y))

            # Create a marker for the closest point
            closest_point_marker = Marker()
            closest_point_marker.header.frame_id = msg.header.frame_id
            closest_point_marker.header.stamp = rospy.Time.now()
            closest_point_marker.ns = "closest_points"
            closest_point_marker.id = i
            closest_point_marker.type = Marker.SPHERE
            closest_point_marker.action = Marker.ADD
            closest_point_marker.pose.position.x = x
            closest_point_marker.pose.position.y = y
            closest_point_marker.pose.position.z = 0
            closest_point_marker.pose.orientation.x = 0.0
            closest_point_marker.pose.orientation.y = 0.0
            closest_point_marker.pose.orientation.z = 0.0
            closest_point_marker.pose.orientation.w = 1.0
            closest_point_marker.scale.x = 0.1
            closest_point_marker.scale.y = 0.1
            closest_point_marker.scale.z = 0.1
            closest_point_marker.color.a = 1.0  # Alpha
            closest_point_marker.color.r = 1.0
            closest_point_marker.color.g = 1.0
            closest_point_marker.color.b = 1.0

            closest_points_marker_array.markers.append(closest_point_marker)

            # Create markers for the quadrant points
            for j in range(start_index, end_index):
                point_range = msg.ranges[j]
                if point_range < msg.range_max and point_range > msg.range_min:
                    angle = msg.angle_min + j * msg.angle_increment
                    x = point_range * math.cos(angle)
                    y = point_range * math.sin(angle)
                    
                    quadrant_point_marker = Marker()
                    quadrant_point_marker.header.frame_id = msg.header.frame_id
                    quadrant_point_marker.header.stamp = rospy.Time.now()
                    quadrant_point_marker.ns = "quadrant_points"
                    quadrant_point_marker.id = j + i * points_per_quadrant
                    quadrant_point_marker.type = Marker.SPHERE
                    quadrant_point_marker.action = Marker.ADD
                    quadrant_point_marker.pose.position.x = x
                    quadrant_point_marker.pose.position.y = y
                    quadrant_point_marker.pose.position.z = 0
                    quadrant_point_marker.pose.orientation.x = 0.0
                    quadrant_point_marker.pose.orientation.y = 0.0
                    quadrant_point_marker.pose.orientation.z = 0.0
                    quadrant_point_marker.pose.orientation.w = 1.0
                    quadrant_point_marker.scale.x = 0.02
                    quadrant_point_marker.scale.y = 0.02
                    quadrant_point_marker.scale.z = 0.02
                    quadrant_point_marker.color.a = 1.0  # Alpha
                    quadrant_point_marker.color.r = colors[i][0]
                    quadrant_point_marker.color.g = colors[i][1]
                    quadrant_point_marker.color.b = colors[i][2]

                    quadrants_marker_array.markers.append(quadrant_point_marker)

        # Publish the markers
        self.quadrants_pub.publish(quadrants_marker_array)
        self.closest_points_pub.publish(closest_points_marker_array)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = ObstacleAvoidance()
    node.run()
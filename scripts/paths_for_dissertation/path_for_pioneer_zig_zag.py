#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import math

class PathGenerator:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('path_generator', anonymous=True)
        
        self.path_topic = rospy.get_param("~path_topic", "/generated_path")
        self.publish_rate = rospy.get_param("~publish_rate", 10)
        self.world_frame = rospy.get_param("~world_frame", "map")

        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=10)

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.world_frame

        self.rate = rospy.Rate(self.publish_rate)

    def generate_and_publish_path(self):
        x_offset = 1.4
        y_offset = 0.0
        
        x_radius = 1.2
        y_radius = 0.8
        
        steps = np.linspace(0, 2*np.pi, 300)
        
        deg = 45
        rotation_matrix = np.array([[math.cos(math.radians(deg)), -math.sin(math.radians(deg))],
                                    [math.sin(math.radians(deg)), math.cos(math.radians(deg))]])
        
        for step in steps:
            pose = PoseStamped()
            pose.header.frame_id = self.world_frame
            x = x_radius * math.cos(step)
            y = y_radius * math.sin(step)
            
            x_y = np.array([[x],
                            [y]])
            
            x_y_rotated = rotation_matrix @ x_y
            
            pose.pose.position.x = x_y_rotated[0][0] + x_offset
            pose.pose.position.y = x_y_rotated[1][0] + y_offset
            pose.pose.position.z = 0
            pose.pose.orientation.w = 1.0

            self.path_msg.poses.append(pose)

            self.path_msg.header.stamp = rospy.Time.now()

        while not rospy.is_shutdown():
            self.path_pub.publish(self.path_msg)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        path_generator = PathGenerator() 
        path_generator.generate_and_publish_path()
    except rospy.ROSInterruptException:
        pass
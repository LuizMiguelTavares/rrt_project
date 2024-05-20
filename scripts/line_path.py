#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import numpy as np

class LinePublisher:
    def __init__(self):
        rospy.init_node('line_publisher_node')
        
        self.listener = tf.TransformListener()
        
        self.x_goal = rospy.get_param('~x_goal', 2.5)
        self.y_goal = rospy.get_param('~y_goal', -2)
        self.num_points = rospy.get_param('~num_points', 100)
        
        self.path_pub = rospy.Publisher('/rrt_path', Path, queue_size=10)
        
        self.rate = rospy.Rate(10) # 10 Hz
        
        self.run()
    
    def run(self):
        while not rospy.is_shutdown():
            try:
                self.listener.waitForTransform("map", "P1", rospy.Time(0), rospy.Duration(4.0))
                (trans, rot) = self.listener.lookupTransform("map", "P1", rospy.Time(0))
                
                start_point = np.array([trans[0], trans[1], trans[2]])
                end_point = np.array([self.x_goal, self.y_goal, 0])
                
                self.publish_path(start_point, end_point)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logerr("TF Exception")
            
            self.rate.sleep()
    
    def publish_path(self, start, end):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()
        
        for i in np.linspace(0, 1, self.num_points):
            point = start + i * (end - start)
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = point[2]
            pose.pose.orientation.w = 1.0
            
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)

if __name__ == '__main__':
    try:
        LinePublisher()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python3

import rospy
from scipy.cluster.hierarchy import linkage, fcluster
from sensor_msgs.msg import PointCloud
from std_srvs.srv import Empty
import numpy as np
from std_msgs.msg import Int32MultiArray
from rrt_project.srv import ClusterObstacles, ClusterObstaclesResponse

class ObstacleClusterizer:
    def __init__(self):
        rospy.init_node('obstacle_clusterizer')
        
        # Service for clustering obstacles
        self.service = rospy.Service('cluster_obstacles', ClusterObstacles, self.handle_cluster_obstacles)
        rospy.loginfo("Obstacle clusterization service is ready.")

    def clusterize_obstacles(self, obstacle_points, max_obstacle_distance):
        """Clusterize the given points based on the specified max distance."""
        Z = linkage(obstacle_points, 'single')
        return fcluster(Z, max_obstacle_distance, criterion='distance')

    def flat_list_to_np_array(self, flat_list):
        """Converts a flat list of coordinates into a 2D NumPy array."""
        np_array = np.array(flat_list)
        return np_array.reshape(-1, 2) 

    def handle_cluster_obstacles(self, req):
        """Service handler for clustering obstacles."""
        try:
            obstacle_points = self.flat_list_to_np_array(req.obstacle_points)
            clusters = self.clusterize_obstacles(obstacle_points, req.max_obstacle_distance)
            response = ClusterObstaclesResponse()
            response.clusters = clusters.tolist()
            rospy.logerr("Cluster_size: %s", len(response.clusters))
            return response
        except Exception as e:
            rospy.logerr("Failed to cluster obstacles: %s", e)
            return None
            

if __name__ == '__main__':
    try:
        node = ObstacleClusterizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
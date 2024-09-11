#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import logging
from nav_msgs.srv import GetMap, GetMapResponse, SetMap, SetMapResponse
from nav_msgs.msg import OccupancyGrid

class MapServerNode:
    def __init__(self):
        # Set the logging level for tf2 to ERROR to suppress warnings
        logging.getLogger('tf2_ros').setLevel(logging.ERROR)
        
        self.map = None
        rospy.init_node('map_server_node')

        # Create a service to provide the updated map
        self.map_service = rospy.Service('/updated_map', GetMap, self.handle_get_updated_map)

        # Create a publisher for the updated map
        self.map_pub = rospy.Publisher('/updated_map', OccupancyGrid, queue_size=10, latch=True)
        
        # Create a service to receive and update the global map with a local map
        self.update_map_service = rospy.Service('/update_map', SetMap, self.handle_update_map)
        
        # Initialize tf2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Subscribe to the map service
        self.get_static_map()
        
        # Publish the map if there are subscribers
        rospy.Timer(rospy.Duration(1), self.publish_map_if_subscribers)

    def get_static_map(self):
        rospy.wait_for_service('/static_map')
        
        try:
            static_map_service = rospy.ServiceProxy('/static_map', GetMap)
            response = static_map_service()
            self.map = response.map
            rospy.loginfo("Received map with width: %d, height: %d", self.map.info.width, self.map.info.height)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            self.map = None

    def handle_get_updated_map(self, req):
        if self.map is not None:
            return GetMapResponse(self.map)
        else:
            rospy.logwarn("Map is not available.")
            return GetMapResponse()

    def transform_map(self, local_map):
        try:
            # Get the transform from the local map frame to the global map frame
            transform = self.tf_buffer.lookup_transform(self.map.header.frame_id, 
                                                         local_map.header.frame_id, 
                                                         rospy.Time(0), 
                                                         rospy.Duration(1.0))
            
            global_map_data = np.copy(self.map.data)
            
            for i in range(local_map.info.width):
                for j in range(local_map.info.height):
                    local_index = j * local_map.info.width + i
                    if local_map.data[local_index] >= 0:
                        local_x = i * local_map.info.resolution + local_map.info.origin.position.x
                        local_y = j * local_map.info.resolution + local_map.info.origin.position.y
                        
                        # Transform the local coordinates to global coordinates
                        local_point = tf2_geometry_msgs.PointStamped()
                        local_point.header.frame_id = local_map.header.frame_id
                        local_point.header.stamp = rospy.Time(0)
                        local_point.point.x = local_x
                        local_point.point.y = local_y
                        local_point.point.z = 0
                        
                        global_point = tf2_geometry_msgs.do_transform_point(local_point, transform)
                        
                        global_x = int((global_point.point.x - self.map.info.origin.position.x) / self.map.info.resolution)
                        global_y = int((global_point.point.y - self.map.info.origin.position.y) / self.map.info.resolution)
                        global_index = global_y * self.map.info.width + global_x
                        
                        if 0 <= global_x < self.map.info.width and 0 <= global_y < self.map.info.height:
                            global_map_data[global_index] = local_map.data[local_index]
            
            return global_map_data
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr("Failed to transform map: %s", e)
            return self.map.data

    def handle_update_map(self, req):
        local_map = req.map
        
        # Transform the local map's coordinates
        updated_map_data = self.transform_map(local_map)
        
        # Update the global map with the transformed local map
        self.map.data = updated_map_data
        self.map_pub.publish(self.map)
        rospy.loginfo("Global map updated with local map")
        
        return SetMapResponse(success=True)

    def publish_map_if_subscribers(self, event):
        if self.map_pub.get_num_connections() > 0:
            self.map_pub.publish(self.map)

if __name__ == "__main__":
    node = MapServerNode()
    rospy.spin()
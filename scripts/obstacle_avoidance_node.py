#!/usr/bin/env python3

import rospy
import tf
import tf2_ros

from geometry_msgs.msg import TwistStamped, Point
from sensor_msgs.msg import LaserScan, PointCloud, ChannelFloat32
from visualization_msgs.msg import Marker

from aurora_py.obstacle_avoidance_2d import ObstacleAvoidance
from scipy.cluster.hierarchy import linkage, fcluster
from scipy.spatial import KDTree
import numpy as np

class ObstacleAvoidanceScan:
    def __init__(self):
        rospy.init_node('obstacle_avoidance_node')
        self.scan_data = None
        self.robot_points = None
        self.linear_velocity = None
        self.angular_velocity = None

        self.max_obstacle_distance = rospy.get_param('~max_obstacle_distance', None)   
        self.density_gain = rospy.get_param('~density_gain', None)
        self.min_observation_radius = rospy.get_param('~min_observation_radius', None)
        self.observation_radius_gain = rospy.get_param('~observation_radius_gain', None)
        self.min_threshold = rospy.get_param('~min_threshold', None)
        self.cumulative_distance = rospy.get_param('~cumulative_distance', None)
        self.scan_topic = rospy.get_param('~scan_topic', None)
        velocity_topic = rospy.get_param('~velocity_topic', None)
        
        rospy.loginfo("max_obstacle_distance: {}".format(self.max_obstacle_distance))
        rospy.loginfo("density_gain: {}".format(self.density_gain))
        rospy.loginfo("min_observation_radius: {}".format(self.min_observation_radius))
        rospy.loginfo("observation_radius_gain: {}".format(self.observation_radius_gain))
        rospy.loginfo("min_threshold: {}".format(self.min_threshold))
        rospy.loginfo("cumulative_distance: {}".format(self.cumulative_distance))
        rospy.loginfo("scan_topic: {}".format(self.scan_topic))
        rospy.loginfo("velocity_topic: {}".format(velocity_topic))

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.world_frame = rospy.get_param('~world_frame', None)
        self.robot_frame = rospy.get_param('~robot_frame', None)
        self.laser_frame = None

        obstacle_avoidance = rospy.get_param('~obstacle_avoidance', None)
        n = obstacle_avoidance['n']
        a = obstacle_avoidance['a']
        b = obstacle_avoidance['b']
        k = obstacle_avoidance['k']
        self.obs_avoidance = ObstacleAvoidance(n=n, a=a, b=b, k=k)

        self.namespace = rospy.get_namespace()
        
        self.velocity_sub = rospy.Subscriber(velocity_topic, 
                                            TwistStamped,
                                            self.velocity_callback,
                                            queue_size=10)

        self.scan_sub = rospy.Subscriber(self.scan_topic, 
                                            LaserScan, 
                                            self.scan_callback, 
                                            queue_size=10)
        
        self.subscribe_robot_points = rospy.Subscriber(f"{self.namespace}points",
                                                    PointCloud,
                                                    self.sub_robot_points,
                                                    queue_size=10)
        
        self.potential_publisher = rospy.Publisher(f"{self.namespace}potential",
                                                    Point,
                                                    queue_size=10)

        self.publish_markers = rospy.Publisher('potential_marker', Marker, queue_size=10)
        self.publish_observation_radius = rospy.Publisher('observation_radius_marker', Marker, queue_size=10)
        self.publish_bezier = rospy.Publisher('Bezier_points', PointCloud, queue_size=10)
        
        self.rate = rospy.Rate(30)

        rospy.loginfo(f"{self.namespace.strip('/')} obstacle avoidance node started")
        
    def velocity_callback(self, velocity_data):
        self.linear_velocity = velocity_data.twist.linear.x
        self.angular_velocity = velocity_data.twist.angular.z

    def scan_callback(self, scan_data):
        self.laser_frame = scan_data.header.frame_id
        self.scan_data = scan_data
        self.num_laser_points = len(scan_data.ranges)
    
    def sub_robot_points(self, robot_points_data):
        self.robot_points = np.array([[p.x, p.y] for p in robot_points_data.points], dtype=float)

    def calculate_cartesian_coordinates(self, scan_data):
        points_world = []
        index = []
        try:
            trans = self.tf_buffer.lookup_transform(self.world_frame, self.laser_frame, rospy.Time(0))
            translation = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z], dtype=float)
            quaternion = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
            rotation_matrix = tf.transformations.quaternion_matrix(quaternion).astype(float)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr('Failed to get transform: {}'.format(e))
            return None, None

        current_angle = scan_data.angle_min
        for i, distance in enumerate(scan_data.ranges):
            if scan_data.range_min < distance < scan_data.range_max:
                local_x = distance * np.cos(current_angle)
                local_y = distance * np.sin(current_angle)
                local_point = np.array([local_x, local_y, 0, 1], dtype=float)

                world_point = np.dot(rotation_matrix, local_point)[:3] + translation

                points_world.append((world_point[0], world_point[1]))
                index.append(i)
                
            current_angle += scan_data.angle_increment
        
        if len(points_world) == 0:
            return np.array([]), np.array([]) 

        return np.array(points_world), index

    def select_obstacle_points(self, robot_points, room_points, index, observation_radius, min_threshold):
        robot_center = np.mean(robot_points, axis=0)
        
        filtered_points = [(idx, room_point) for idx, room_point in zip(index, room_points)
                        if min_threshold < np.linalg.norm(robot_center - room_point) < observation_radius]
        
        if len(filtered_points) == 0:
            return np.array([]), np.array([])
            
        selected_indices, selected_points = zip(*filtered_points)
        
        return np.array(selected_points), np.array(selected_indices)

    def clusterize_obstacles(self, obstacle_points, max_obstacle_distance):
        Z = linkage(obstacle_points, 'single')
        return fcluster(Z, max_obstacle_distance, criterion='distance')

    def calculate_potential(self, robot_points, obstacle_points, clusters, obstacle_idx, cumulative_distance, density_gain):
        if clusters[0] == clusters[-1] and len(set(clusters))>1:
            clusters = list(clusters)
            obstacle_points = list(obstacle_points)
            while clusters[0] == clusters[-1]:
                aux, obs_aux = clusters.pop(-1), obstacle_points.pop(-1)
                clusters.insert(0, aux)
                obstacle_points.insert(0, obs_aux)
            obstacle_points = np.array(obstacle_points, dtype=float)
        elif np.any(obstacle_idx > (self.num_laser_points-self.num_laser_points/4)) and np.any(obstacle_idx < self.num_laser_points/4) and len(set(clusters))==1:
            max_diff = 0
            index_of_max_diff = -1
            
            for i in range(1, len(obstacle_idx)):
                diff = obstacle_idx[i] - obstacle_idx[i-1]
                if diff > max_diff:
                    max_diff = diff
                    index_of_max_diff = i - 1

            obstacle_points = np.concatenate((obstacle_points[index_of_max_diff + 1:], obstacle_points[:index_of_max_diff + 1]), axis=0)
            clusters = np.concatenate((clusters[index_of_max_diff + 1:], clusters[:index_of_max_diff + 1]))

        cluster_dict = {cluster: obstacle_points[np.where(np.array(clusters) == cluster)] for cluster in set(clusters)}
        
        bezier_pt_cloud = []
        bezier_clusters = []
        
        x_dot, y_dot = 0, 0
        for cluster, points in cluster_dict.items():
            kdtree = KDTree(points)
            smallest_distance = np.inf
            for robot_point in robot_points:
                dist, index = kdtree.query(robot_point)
                if dist < smallest_distance:
                    smallest_distance = dist
                    closest_index = index
                    closest_robot_point = robot_point

            x_p = np.array([points[0][0], points[closest_index][0], points[-1][0]], dtype=float) 
            y_p = np.array([points[0][1], points[closest_index][1], points[-1][1]], dtype=float) 

            control_points = self._calculate_control_points(points[closest_index], points, cumulative_distance=cumulative_distance)
            density = int(density_gain*len(control_points)/smallest_distance**2) + 4
            bezier_points = self._bezier_curve(control_points, density=density)
            
            aux_bezier_pt_cloud = [Point(x=x, y=y) for x, y in bezier_points]
            bezier_pt_cloud.extend(aux_bezier_pt_cloud)
            aux_cluster = [float(cluster)]*len(bezier_points)
            bezier_clusters.extend(aux_cluster)
            
            # Pensar na possibilidade de colocar mais pontos do robô e não só o mais próximo
            x_dot_partial, y_dot_partial = self.obs_avoidance.obstacle_avoidance(closest_robot_point, bezier_points)
            x_dot += x_dot_partial
            y_dot += y_dot_partial
        
        bezier_points_pc = PointCloud()
        bezier_points_pc.header.frame_id = self.world_frame
        bezier_points_pc.header.stamp = rospy.Time.now()
        bezier_points_pc.points = bezier_pt_cloud

        bezier_clusters_channel = ChannelFloat32()
        bezier_clusters_channel.name = "cluster_ids"
        bezier_clusters_channel.values = bezier_clusters
        bezier_points_pc.channels.append(bezier_clusters_channel)
        
        self.publish_bezier.publish(bezier_points_pc)
        return x_dot, y_dot

    def _calculate_control_points(self, closest_point, points, cumulative_distance=0.15):
        cum_dist = 0
        control_points = []
        
        for idx, point in enumerate(points):
            if idx>0:
                cum_dist += np.linalg.norm(np.array(point)-np.array(prev_point))
            prev_point = point

            if idx == 0 or idx == len(points)-1 or cum_dist >= cumulative_distance or np.all(point==closest_point):
                control_points.append(point)
                
                cum_dist = 0
        control_points = np.array(control_points, dtype=float)
        return control_points
    
    def _bezier_curve(self, control_points, density):
        control_points = np.array(control_points, dtype=float)  # Ensure control points are float type
        t_values = np.linspace(0, 1, density)

        curve_points = np.zeros((density, 2), dtype=float)  # Initialize curve_points as float type
        n = len(control_points) - 1
        for i, point in enumerate(control_points):
            curve_points += np.outer(np.power(1 - t_values, n - i) * np.power(t_values, i) * 
                                    np.math.comb(n, i), point)
        
        bezier_points = PointCloud()
        bezier_points.header.frame_id = self.world_frame
        bezier_points.header.stamp = rospy.Time.now()
        for point in curve_points:
            p = Point()
            p.x, p.y = point
            bezier_points.points.append(p)
        
        return curve_points

    
    def loop(self):
        while not rospy.is_shutdown():
            if self.robot_points is None or self.scan_data is None:
                # rospy.logwarn('Obstacle_avoidance_scan_node: No points or scan_data found!')
                self.rate.sleep()
                continue
            
            scan_data = self.scan_data
            room_points, index = self.calculate_cartesian_coordinates(scan_data)
            
            if room_points is None and index is None:
                self.rate.sleep()
                continue
            
            if len(room_points) == 0:
                potential_msg = Point(x=0.0, y=0.0)
                self.potential_publisher.publish(potential_msg)
                self.rate.sleep()
                continue
            
            robot_points = self.robot_points
            observation_radius_gain = self.observation_radius_gain
            
            observation_radius = observation_radius_gain * np.sqrt(self.linear_velocity**2 + self.angular_velocity**2)
            
            if observation_radius < self.min_observation_radius:
                observation_radius = self.min_observation_radius
            
            self.observation_radius_cylinder_marker(observation_radius)
            
            min_threshold = self.min_threshold
            obstacle_points, obstacle_idx = self.select_obstacle_points(robot_points, room_points, index, observation_radius, min_threshold)

            if len(obstacle_points) < 2:
                potential_msg = Point(x=0.0, y=0.0)
                self.potential_publisher.publish(potential_msg)
                self.rate.sleep()
                continue

            max_obstacle_distance = self.max_obstacle_distance
            clusters = self.clusterize_obstacles(obstacle_points, max_obstacle_distance)

            cumulative_distance = self.cumulative_distance
            density_gain = self.density_gain
            x_dot, y_dot = self.calculate_potential(robot_points, obstacle_points, clusters, obstacle_idx, cumulative_distance, density_gain)
            
            self.pub_marker(x_dot, y_dot)

            potential_msg = Point(x=x_dot, y=y_dot)
            self.potential_publisher.publish(potential_msg)
            self.rate.sleep()
    
    def pub_marker(self, x_dot, y_dot):
        
        # Angle of the arrow in quaternion
        theta = np.arctan2(y_dot, x_dot)
        q = tf.transformations.quaternion_from_euler(0, 0, theta)
        
        # Intensity of the arrow in logarithmic scale
        intensity = np.sqrt(x_dot**2 + y_dot**2)

        marker = Marker()
        marker.header.frame_id = self.robot_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "Potential field"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Set the pose of the marker
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]

        # Set the scale of the marker
        marker.scale.x = intensity 
        marker.scale.y = 0.01  # arrow width
        marker.scale.z = 0.01  # arrow height

        # Set the color
        marker.color.a = 1.0 
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        # Publish the marker
        self.publish_markers.publish(marker)
        
    def observation_radius_cylinder_marker(self, observation_radius):
        marker = Marker()
        marker.header.frame_id = self.robot_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "Potential field observation cylinder"
        
        marker.id = 1
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        
        marker.scale.x = observation_radius*2
        marker.scale.y = observation_radius*2
        marker.scale.z = 0.01
        
        marker.color.a = 0.5
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        
        self.publish_observation_radius.publish(marker)

def main():
    obs_avoidance = ObstacleAvoidanceScan()
    obs_avoidance.loop()

if __name__ == '__main__':
    main()
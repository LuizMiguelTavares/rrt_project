#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
import tf
from geometry_msgs.msg import Twist, Point, TwistStamped, PoseStamped, PointStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import ChannelFloat32
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np

class DifferentialController:
    def __init__(self):
        rospy.init_node('differential_controller')

        # Variáveis de controle
        self.x_dot_obs, self.y_dot_obs = 0.0, 0.0
        self.x_dot_world, self.y_dot_world = None, None
        self.btn_emergencia = False
        self.btn_emergencia_is_on = False
        self.robot_path_is_on = False
        self.robot_pose_is_on = False
        self.path_index = 0
        self.route = []
        self.end_of_path = False
        self.jacobian_true = False
        self.v_true = False
        
        self.old_route = None

        self.prev_pose = None
        self.prev_potential = None
        self.prev_time = rospy.Time.now()
        
        Ku = np.array([[11.6307,     0  ],
                            [    0   , 8.1689]])
        
        self.Kv = np.array([[12.5165,      0  ],
                       [    0   , 11.9415]])
        
        self.Ku_inv = np.linalg.inv(Ku)
        
        self.kd = np.array([[20, 0],
                            [0, 20]])

        # Tempo e período da última atualização do LIDAR
        self.potential_update_time = None
        self.potential_period = None  # Período entre as atualizações de potencial

        # Configuração do transform buffer e listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Obtendo parâmetros
        gains = rospy.get_param('~gains', None)
        self.angular_velocity_priority_gain = rospy.get_param('~angular_velocity_priority_gain', 0.0)
        self.distance_to_change_path_index = rospy.get_param('~distance_to_change_path_index', None)
        self.reference_velocity = rospy.get_param('~reference_velocity', 0.1)
        self.max_linear_velocity = rospy.get_param('~max_linear_velocity', 0.2)
        self.max_angular_velocity = rospy.get_param('~max_angular_velocity', 0.5)
        self.world_frame = rospy.get_param('~world_frame', None)
        self.robot_frame = rospy.get_param('~robot_frame', None)
        self.robot_type = rospy.get_param('~robot_type', None)
        robot_control_topic = rospy.get_param('~robot_control_topic', None)
        self.rate = rospy.Rate(rospy.get_param('~control_frequency', 30))
        self.path_topic = rospy.get_param('~path_topic', "path")
        self.goal_threshold = rospy.get_param('~goal_threshold', 0.25)
        self.apply_filter = rospy.get_param('~apply_filter', True)
        self.linear_filter_gain = rospy.get_param('~linear_filter_gain', 0.8)
        self.angular_filter_gain = rospy.get_param('~angular_filter_gain', 0.8)
        self.pose_topic = rospy.get_param('~pose_topic', "/vrpn_client_node/L1/pose")

        self.last_linear_velocity_x = False
        self.last_linear_velocity_y = False
        self.last_angular_velocity = False

        self.pgains = [gains['linear'], gains['angular']]

        print(f"Gains - Linear: {self.pgains[0]}, Angular: {self.pgains[1]}")
        self.a = rospy.get_param('~a', 0.15)

        self.publisher = rospy.Publisher(robot_control_topic, Twist, queue_size=10)
        self.publisher_no_limits = rospy.Publisher(robot_control_topic + '_no_limits', Twist, queue_size=10)
        self.publisher_no_limits_no_potential = rospy.Publisher(robot_control_topic + '_no_limits_no_potential', Twist, queue_size=10)
        self.publish_which_route_point = rospy.Publisher('which_route_point', PointStamped, queue_size=10)
        self.publish_control_point = rospy.Publisher('control_point', PointStamped, queue_size=10)

        rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_callback, queue_size=10)
        
        self.path_subscriber = rospy.Subscriber(self.path_topic,
                                                Path,
                                                self.route_callback)

        self.potential_subscriber = rospy.Subscriber(f"potential",
                                                     Point,
                                                     self.potential_callback)

        self.emergency_flag_subscriber = rospy.Subscriber('/emergency_flag',
                                                          Bool,
                                                          self.emergency_button_callback,
                                                          queue_size=10)

        self.stop_msg = Twist()
        self.stop_msg.linear.x = 0.0
        self.stop_msg.linear.y = 0.0
        self.stop_msg.linear.z = 0.0
        self.stop_msg.angular.x = 0.0
        self.stop_msg.angular.y = 0.0
        self.stop_msg.angular.z = 0.0

        rospy.loginfo(f'{rospy.get_name()} started!')

    def potential_callback(self, X_dot_obs):
        # Agora estamos apenas pegando os valores diretos sem filtragem ou interpolação
        self.x_dot_obs = X_dot_obs.x
        self.y_dot_obs = X_dot_obs.y

    def pose_callback(self, pose):
        self.robot_pose = pose
        if not self.robot_pose_is_on:
            self.robot_pose_is_on = True

    def route_callback(self, route_data):
        if self.old_route and route_data.header.stamp == self.old_route.header.stamp:
            return

        try:
            transform = self.tf_buffer.lookup_transform(self.world_frame,
                                                        route_data.header.frame_id,
                                                        rospy.Time(0),
                                                        rospy.Duration(1.0))

            translation = np.array([transform.transform.translation.x,
                                    transform.transform.translation.y,
                                    transform.transform.translation.z])
            quaternion = [transform.transform.rotation.x,
                        transform.transform.rotation.y,
                        transform.transform.rotation.z,
                        transform.transform.rotation.w]
            rotation_matrix = tf.transformations.quaternion_matrix(quaternion)[:3, :3]

            num_poses = len(route_data.poses)
            transformed_route = np.zeros((num_poses, 2))
            transformed_route_dx = np.zeros((num_poses, 2))
            transformed_route_ddx = np.zeros((num_poses, 2))

            pose_positions = np.array([[pose.pose.position.x, pose.pose.position.y, 1] for pose in route_data.poses])

            for i in range(num_poses):
                transformed_position = np.dot(rotation_matrix[:2, :2], pose_positions[i, :2]) + translation[:2]
                transformed_route[i] = transformed_position

                if i > 0:
                    dx_vector = transformed_route[i] - transformed_route[i - 1]
                    unitary_dx_vector = dx_vector / np.linalg.norm(dx_vector)
                    transformed_route_dx[i - 1] = unitary_dx_vector * self.reference_velocity

                    if i > 1:  # Calcula a aceleração somente a partir do segundo ponto
                        ddx_vector = transformed_route_dx[i - 1] - transformed_route_dx[i - 2]
                        transformed_route_ddx[i - 2] = ddx_vector

            transformed_route_dx[-1] = np.array([0, 0])
            transformed_route_ddx[-1] = np.array([0, 0])
            transformed_route_ddx[-2] = np.array([0, 0])

            self.z_route = transform.transform.translation.z

            self.robot_path_is_on = True
            self.route = transformed_route
            self.route_dx = transformed_route_dx
            self.route_ddx = transformed_route_ddx

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Failed to fetch or apply transform: {e}")

        self.old_route = route_data

    def find_closest_point(self, robot_pose, route):
        min_dist = float('inf')
        closest_point = None
        for idx, point in enumerate(route):
            dist = np.linalg.norm(np.array(point) - np.array(robot_pose[:2]))
            if dist < min_dist:
                min_dist = dist
                closest_point = point
                closest_idx = idx
        return closest_point, closest_idx

    def emergency_button_callback(self, emergency):
        self.btn_emergencia_is_on = True
        if emergency.data:
            self.btn_emergencia = True
            rospy.loginfo('Robot stopping by Emergency')
            rospy.loginfo('Sending emergency stop command')

            for _ in range(10):
                stop_cmd = Twist()
                stop_cmd.linear.x = 0.0
                stop_cmd.linear.y = 0.0
                stop_cmd.linear.z = 0.0
                stop_cmd.angular.x = 0.0
                stop_cmd.angular.y = 0.0
                stop_cmd.angular.z = 0.0
                # Publish the Twist message to stop the robot
                self.publisher.publish(stop_cmd)

            rospy.signal_shutdown("Limo Emergency stop")

    def control_loop(self):
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            delta_t = (current_time - self.prev_time).to_sec()

            if not self.robot_path_is_on or not self.robot_pose_is_on or self.btn_emergencia:
                self.rate.sleep()
                continue

            translation = np.array([self.robot_pose.pose.position.x, self.robot_pose.pose.position.y])
            quaternion = [self.robot_pose.pose.orientation.x, self.robot_pose.pose.orientation.y, self.robot_pose.pose.orientation.z, self.robot_pose.pose.orientation.w]
            yaw = tf.transformations.euler_from_quaternion(quaternion)[2]
            robot_pose_center = np.array([translation[0], translation[1], yaw])

            if self.prev_pose is not None and delta_t > 0:
                velocity = (robot_pose_center[:2] - self.prev_pose[:2]) / delta_t
                # rospy.loginfo(f"Robot Velocity: {velocity}")
            else:
                velocity = np.array([0.0, 0.0])

            self.prev_pose = robot_pose_center

            if self.prev_potential is not None and delta_t > 0:
                potential_derivative_x = (self.x_dot_obs - self.prev_potential[0]) / delta_t
                potential_derivative_y = (self.y_dot_obs - self.prev_potential[1]) / delta_t
                # rospy.loginfo(f"Potential Derivative - X: {potential_derivative_x}, Y: {potential_derivative_y}")
            else:
                potential_derivative_x, potential_derivative_y = 0.0, 0.0

            self.prev_potential = [self.x_dot_obs, self.y_dot_obs]
            
            # print(self.x_dot_obs, self.y_dot_obs, "\n")

            # Lógica de controle
            robot_pose_control = robot_pose_center + np.array([np.cos(yaw)*self.a, np.sin(yaw)*self.a, 0.0])

            point = PointStamped()
            point.header.frame_id = self.world_frame
            point.header.stamp = rospy.Time.now()
            point.point.x = robot_pose_control[0]
            point.point.y = robot_pose_control[1]
            point.point.z = 0.0
            self.publish_control_point.publish(point)
            
            route = self.route
            route_dx = self.route_dx
            route_ddx = self.route_ddx
            
            closest_point, closest_idx = self.find_closest_point(robot_pose_control, route)
            
            if self.path_index > len(route) - 1:
                self.path_index = len(route) - 1
                
            x_desired = closest_point[0]
            y_desired = closest_point[1]

            x_dot_desired = route_dx[closest_idx][0]
            y_dot_desired = route_dx[closest_idx][1]
            
            x_ddot_desired = route_ddx[closest_idx][0]
            y_ddot_desired = route_ddx[closest_idx][1]
            
            point = PointStamped()
            point.header.frame_id = self.world_frame
            point.header.stamp = rospy.Time.now()
            point.point.x = x_desired
            point.point.y = y_desired
            point.point.z = self.z_route
            self.publish_which_route_point.publish(point)
            
            x_d_goal = route[-1][0] - robot_pose_control[0]
            y_d_goal = route[-1][1] - robot_pose_control[1]
            distance_to_goal = np.sqrt((x_d_goal)**2 + (y_d_goal)**2)
            
            if (self.path_index >= len(route) - 1) and (distance_to_goal <= self.goal_threshold):
                rospy.loginfo('Path completed')
                self.publisher.publish(self.stop_msg)
                self.rate.sleep()
                continue
            
            A_o = np.array([[np.cos(yaw), -np.sin(yaw)],
                            [np.sin(yaw), np.cos(yaw)]])
            
            A_o_inv = np.linalg.inv(A_o)
            
            X_dot_obs_ref = np.array([[self.x_dot_obs],
                                      [self.y_dot_obs]])
            
            X_ddot_obs_ref = np.array([[potential_derivative_x],
                                       [potential_derivative_y]])
            
            ### X_obs_ref to world
            X_dot_obs_ref_w = A_o @ X_dot_obs_ref
            
            #### Calculate X_dot_ref_path ####
            X_dot_desired_w = np.array([[x_dot_desired],
                                        [y_dot_desired]])

            X_til_w = np.array([[x_desired - robot_pose_control[0]],
                                [y_desired - robot_pose_control[1]]])

            gains = np.array([[self.pgains[0], 0],
                                [0, self.pgains[1]]])

            X_dot_ref_path_w = X_dot_desired_w + gains @ X_til_w
            
            X_dot_ref_w = X_dot_ref_path_w + X_dot_obs_ref_w
            
            ## Acceleration
            X_ddot_obs_ref_w = A_o @ X_ddot_obs_ref
            
            X_dot = np.array([[velocity[0]],
                              [velocity[1]]])
            
            X_dot_til = X_dot_ref_w - X_dot
            
            X_ddot_desired_w = np.array([[x_ddot_desired],
                                        [y_ddot_desired]])
            
            X_ddot_ref_path_w = X_ddot_desired_w + self.kd @ X_dot_til
            
            X_ddot_ref_w = X_ddot_ref_path_w + X_ddot_obs_ref_w
            
            uv = (self.Ku_inv @ A_o_inv) @ (X_ddot_ref_w + A_o @ self.Kv @ A_o_inv @ X_dot) 
            
            ## Psi control
            psi_desired = np.arctan2(X_dot_desired_w[1][0], X_dot_desired_w[0][0])

            psi_til = psi_desired - yaw

            if psi_til > np.pi:
                psi_til -= 2*np.pi
            if psi_til < -np.pi:
                psi_til += 2*np.pi

            reference_angular_velocity = 5*psi_til
            
            reference_linear_velocity_x = uv[0][0]
            reference_linear_velocity_y = uv[1][0]

            ctrl_msg_no_limits = Twist()
            ctrl_msg_no_limits.linear.x = reference_linear_velocity_x
            ctrl_msg_no_limits.linear.y = reference_linear_velocity_y
            ctrl_msg_no_limits.linear.z = 0.0
            ctrl_msg_no_limits.angular.x = 0.0
            ctrl_msg_no_limits.angular.y = 0.0
            ctrl_msg_no_limits.angular.z = reference_angular_velocity
            
            self.publisher_no_limits.publish(ctrl_msg_no_limits)

            if np.abs(reference_linear_velocity_x) > self.max_linear_velocity:
                reference_linear_velocity_x = np.sign(reference_linear_velocity_x)*self.max_linear_velocity
            if np.abs(reference_linear_velocity_y) > self.max_linear_velocity:
                reference_linear_velocity_y = np.sign(reference_linear_velocity_y)*self.max_linear_velocity
            if np.abs(reference_angular_velocity) > self.max_angular_velocity:
                reference_angular_velocity = np.sign(reference_angular_velocity)*self.max_angular_velocity
                
            
            if self.apply_filter:
                if self.last_linear_velocity_x:
                    reference_linear_velocity_x = self.linear_filter_gain * reference_linear_velocity_x + (1 - self.linear_filter_gain) * self.last_linear_velocity_x
                    self.last_linear_velocity_x = reference_linear_velocity_x
                else:
                    self.last_linear_velocity_x = reference_linear_velocity_x
                    
                if self.last_linear_velocity_y:
                    reference_linear_velocity_y = self.linear_filter_gain * reference_linear_velocity_y + (1 - self.linear_filter_gain) * self.last_linear_velocity_y
                    self.last_linear_velocity_y = reference_linear_velocity_y
                else:
                    self.last_linear_velocity_y = reference_linear_velocity_y

                if self.last_angular_velocity:
                    reference_angular_velocity = self.angular_filter_gain * reference_angular_velocity + (1 - self.angular_filter_gain) * self.last_angular_velocity
                    self.last_angular_velocity = reference_angular_velocity
                else:
                    self.last_angular_velocity = reference_angular_velocity

            ctrl_msg = Twist()
            ctrl_msg.linear.x = reference_linear_velocity_x
            ctrl_msg.linear.y = reference_linear_velocity_y
            ctrl_msg.linear.z = 0.0
            ctrl_msg.angular.x = 0.0
            ctrl_msg.angular.y = 0.0
            ctrl_msg.angular.z = reference_angular_velocity
            self.publisher.publish(ctrl_msg)
            
            self.prev_time = current_time
            self.rate.sleep()

def main():
    try:
        robot = DifferentialController()
        robot.control_loop()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
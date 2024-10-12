#!/usr/bin/env python3
#!/usr/bin/env python3

import rospy
import tf2_ros
import tf
from geometry_msgs.msg import Twist, PoseStamped, PointStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import ChannelFloat32

import numpy as np

class DifferentialController:
    def __init__(self):
        rospy.init_node('differential_controller')

        # Variables initialization
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

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Getting Params
        gains = rospy.get_param('~gains', None)
        self.angular_velocity_priority_gain = rospy.get_param('~angular_velocity_priority_gain', 0.0)
        self.distance_to_change_path_index = rospy.get_param('~distance_to_change_path_index', None)
        self.reference_velocity = rospy.get_param('~reference_velocity', 0.1)
        self.max_linear_velocity = rospy.get_param('~max_linear_velocity', 0.2)
        self.max_angular_velocity = rospy.get_param('~max_angular_velocity', 0.5)
        self.world_frame = rospy.get_param('~world_frame', None)
        self.robot_type = rospy.get_param('~robot_type', None)
        robot_control_topic = rospy.get_param('~robot_control_topic', None)
        robot_control_message = rospy.get_param('~robot_control_message', None)
        self.rate = rospy.Rate(rospy.get_param('~control_frequency', 30))
        self.path_topic = rospy.get_param('~path_topic', "path")
        self.goal_threshold = rospy.get_param('~goal_threshold', 0.05)
        self.apply_filter = rospy.get_param('~apply_filter', True)
        self.linear_filter_gain = rospy.get_param('~linear_filter_gain', 0.8)
        self.angular_filter_gain = rospy.get_param('~angular_filter_gain', 0.8)
        self.pose_topic = rospy.get_param('~pose_topic', "/vrpn_client_node/L1/pose")
        self.obs_filter_gain = rospy.get_param('~obs_filter_gain', 0.7)

        self.last_linear_velocity = False
        self.last_X_obs_dot = False
        self.last_angular_velocity = False

        # velocity_topic = rospy.get_param('~velocity_topic', None)

        self.pgains = [gains['linear'], gains['angular']]

        print(f"Gains - Linear: {self.pgains[0]}, Angular: {self.pgains[1]}")
        self.a = rospy.get_param('~a', 0.15)

        message_types = {
            'Twist': Twist,
            'ChannelFloat32': ChannelFloat32,
        }

        self.publisher = rospy.Publisher(robot_control_topic, message_types[robot_control_message], queue_size=10)
        self.publish_which_route_point = rospy.Publisher('which_route_point', PointStamped, queue_size=10)
        self.publish_control_point = rospy.Publisher('control_point', PointStamped, queue_size=10)
        
        rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_callback, queue_size=10)
        
        self.path_subscriber = rospy.Subscriber(self.path_topic,
                            Path,
                            self.route_callback)

        ### Stop message
        if self.robot_type == 'Solverbot':
            self.stop_msg = ChannelFloat32(values=[0.0, 0.0])

        if self.robot_type == 'Pioneer':
            self.stop_msg = Twist()
            self.stop_msg.linear.x = 0.0
            self.stop_msg.linear.y = 0.0
            self.stop_msg.linear.z = 0.0
            self.stop_msg.angular.x = 0.0
            self.stop_msg.angular.y = 0.0
            self.stop_msg.angular.z = 0.0

        rospy.loginfo(f'{rospy.get_name()} started!')

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

            pose_positions = np.array([[pose.pose.position.x, pose.pose.position.y, 1] for pose in route_data.poses])

            for i in range(num_poses):
                transformed_position = np.dot(rotation_matrix[:2, :2], pose_positions[i, :2]) + translation[:2]
                transformed_route[i] = transformed_position

                if i > 0:
                    dx_vector = transformed_route[i] - transformed_route[i - 1]
                    unitary_dx_vector = dx_vector / np.linalg.norm(dx_vector)
                    transformed_route_dx[i - 1] = unitary_dx_vector * self.reference_velocity
            
            dx_vector = transformed_route[0] - transformed_route[-1]
            unitary_dx_vector = dx_vector / np.linalg.norm(dx_vector)
            transformed_route_dx[-1] = unitary_dx_vector * self.reference_velocity
            # transformed_route_dx[-1] = np.array([0, 0])

            self.z_route = transform.transform.translation.z

            self.robot_path_is_on = True
            self.route = transformed_route
            self.route_dx = transformed_route_dx

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

    def control_loop(self):
        while not rospy.is_shutdown():
            if self.robot_path_is_on == False or self.robot_pose_is_on == False:
                # rospy.loginfo(f'{rospy.get_name()}: No path or emergency button found')
                self.rate.sleep()
                continue

            # if self.btn_emergencia:
            #     self.rate.sleep()
            #     continue

            translation = np.array([self.robot_pose.pose.position.x, self.robot_pose.pose.position.y, self.robot_pose.pose.position.z])
            quaternion = [self.robot_pose.pose.orientation.x, self.robot_pose.pose.orientation.y, self.robot_pose.pose.orientation.z, self.robot_pose.pose.orientation.w]

            yaw = tf.transformations.euler_from_quaternion(quaternion)[2]
            robot_pose_center = np.array([translation[0], translation[1], yaw])

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

            closest_point, closest_idx = self.find_closest_point(robot_pose_control, route)

            if self.path_index > len(route) - 1:
                self.path_index = len(route) - 1

            x_desired = closest_point[0]
            y_desired = closest_point[1]

            x_dot_desired = route_dx[closest_idx][0]
            y_dot_desired = route_dx[closest_idx][1]

            x_d_goal = route[-1][0] - robot_pose_control[0]
            y_d_goal = route[-1][1] - robot_pose_control[1]
            distance_to_goal = np.sqrt((x_d_goal)**2 + (y_d_goal)**2)
            
            # print(f"Distance to goal: {distance_to_goal}")

            point = PointStamped()
            point.header.frame_id = self.world_frame
            point.header.stamp = rospy.Time.now()
            point.point.x = x_desired
            point.point.y = y_desired
            point.point.z = self.z_route
            self.publish_which_route_point.publish(point)

            ######## Control ########
            rotation_matrix_bw = np.array([[np.cos(yaw), -np.sin(yaw)],
                                           [np.sin(yaw), np.cos(yaw)]])

            ### H_inv is the inverse of the kinematic model
            H_inv = np.array([[      np.cos(yaw)      ,       np.sin(yaw)],
                            [-(1/self.a)*np.sin(yaw), (1/self.a)*np.cos(yaw)]])

            #### Calculate X_dot_ref_path ####
            X_dot_desired_w = np.array([[x_dot_desired],
                                        [y_dot_desired]])

            X_til_w = np.array([[x_desired - robot_pose_control[0]],
                                [y_desired - robot_pose_control[1]]])

            gains = np.array([[self.pgains[0], 0],
                                [0, self.pgains[1]]])

            X_dot_ref_path_w = X_dot_desired_w + gains @ X_til_w

            #### Calculate X_dot_ref_obs ####
            
            if self.last_X_obs_dot:
                self.x_dot_obs = self.obs_filter_gain * self.x_dot_obs + (1 - self.obs_filter_gain) * self.last_X_obs_dot[0]
                self.y_dot_obs = self.obs_filter_gain * self.y_dot_obs + (1 - self.obs_filter_gain) * self.last_X_obs_dot[1]
                
                self.last_X_obs_dot = [self.x_dot_obs, self.y_dot_obs]
            else:
                self.last_X_obs_dot = [self.x_dot_obs, self.y_dot_obs]
            
            X_dot_obs_ref = np.array([[self.x_dot_obs],
                                        [self.y_dot_obs]])

            ### X_obs_ref to world
            X_dot_obs_ref_w = rotation_matrix_bw @ X_dot_obs_ref

            #### Full reference ####
            X_dot_ref_w = X_dot_obs_ref_w + X_dot_ref_path_w

            uw = H_inv @ X_dot_ref_w

            reference_linear_velocity = uw[0][0]
            reference_angular_velocity = uw[1][0]

            if np.abs(reference_linear_velocity) > self.max_linear_velocity:
                reference_linear_velocity = np.sign(reference_linear_velocity)*self.max_linear_velocity
            if np.abs(reference_angular_velocity) > self.max_angular_velocity:
                reference_angular_velocity = np.sign(reference_angular_velocity)*self.max_angular_velocity

            if self.apply_filter:
                if self.last_linear_velocity:
                    reference_linear_velocity = self.linear_filter_gain * reference_linear_velocity + (1 - self.linear_filter_gain) * self.last_linear_velocity
                    self.last_linear_velocity = reference_linear_velocity
                else:
                    self.last_linear_velocity = reference_linear_velocity

                if self.last_angular_velocity:
                    reference_angular_velocity = self.angular_filter_gain * reference_angular_velocity + (1 - self.angular_filter_gain) * self.last_angular_velocity
                    self.last_angular_velocity = reference_angular_velocity
                else:
                    self.last_angular_velocity = reference_angular_velocity

            ctrl_msg = Twist()
            ctrl_msg.linear.x = reference_linear_velocity
            ctrl_msg.linear.y = 0.0
            ctrl_msg.linear.z = 0.0
            ctrl_msg.angular.x = 0.0
            ctrl_msg.angular.y = 0.0
            ctrl_msg.angular.z = reference_angular_velocity
            self.publisher.publish(ctrl_msg)

            self.rate.sleep()

def main():
    try:
        robot = DifferentialController()
        robot.control_loop()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
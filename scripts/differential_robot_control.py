#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
import tf
from geometry_msgs.msg import Twist, Point, TwistStamped, PoseStamped, PointStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from sensor_msgs.msg import ChannelFloat32

from aurora_py.differential_robot_controller import solver_bot_controller, pioneer_controller

import numpy as np

class DifferentialController:
    def __init__(self):
        rospy.init_node('differential_controller')

        # Variables initialization
        self.x_dot, self.y_dot = 0.0, 0.0
        self.x_dot_world, self.y_dot_world = None, None
        self.btn_emergencia = False
        self.btn_emergencia_is_on = False
        self.robot_path_is_on = False
        self.path_index = 0
        self.route = []

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Getting Params
        gains = rospy.get_param('~gains', None)
        self.angular_velocity_priority_gain = rospy.get_param('~angular_velocity_priority_gain', None)
        self.distance_to_change_path_index = rospy.get_param('~distance_to_change_path_index', None)
        self.min_velocity = rospy.get_param('~min_velocity', None)
        self.max_linear_velocity = rospy.get_param('~max_linear_velocity', None)
        self.world_frame = rospy.get_param('~world_frame', None)
        self.robot_frame = rospy.get_param('~robot_frame', None)
        self.robot_type = rospy.get_param('~robot_type', None)
        robot_control_topic = rospy.get_param('~robot_control_topic', None)
        robot_control_message = rospy.get_param('~robot_control_message', None)
        self.rate = rospy.Rate(rospy.get_param('~control_frequency', 30))
        self.path_topic = rospy.get_param('~path_topic', "path")
        self.goal_threshold = rospy.get_param('~goal_threshold', 0.25)
        # velocity_topic = rospy.get_param('~velocity_topic', None)
        
        self.pgains = [gains['linear'], gains['angular']]
        self.a = rospy.get_param('~a', 0.15)

        if self.robot_type == 'Solverbot':
            self.differential_robot_controller = solver_bot_controller

        if self.robot_type == 'Pioneer':
            self.differential_robot_controller = pioneer_controller

        message_types = {
            'Twist': Twist,
            'ChannelFloat32': ChannelFloat32,
        }

        self.publisher = rospy.Publisher(robot_control_topic, message_types[robot_control_message], queue_size=10)
        self.publish_which_route_point = rospy.Publisher('which_route_point', PointStamped, queue_size=10)

        self.path_subscriber = rospy.Subscriber(self.path_topic,
                                                Path,
                                                self.route_callback)

        self.potential_subscriber = rospy.Subscriber(f"potential",
                                                    Point,
                                                    self.potential_callback)

        # self.velocity_sub = rospy.Subscriber(velocity_topic,
        #                                     TwistStamped,
        #                                     self.velocity_callback,
        #                                     queue_size=10)

        self.emergency_flag_subscriber = rospy.Subscriber('/emergency_flag',
                                                        Bool,
                                                        self.emergency_button_callback,
                                                        queue_size=10)

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

    def potential_callback(self, potential_data):
        self.x_dot = potential_data.x
        self.y_dot = potential_data.y

    def route_callback(self, route_data):
        transformed_route = []

        try:
            transform = self.tf_buffer.lookup_transform(self.world_frame,
                                                        route_data.header.frame_id,
                                                        rospy.Time(0),
                                                        rospy.Duration(1.0))
            
            for pose in route_data.poses:
                pose_stamped = PoseStamped()
                pose_stamped.pose = pose.pose
                pose_stamped.header.frame_id = route_data.header.frame_id
                pose_stamped.header.stamp = pose.header.stamp

                transformed_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)

                transformed_route.append([transformed_pose.pose.position.x, transformed_pose.pose.position.y])
                self.z_route = transformed_pose.pose.position.z
            self.robot_path_is_on = True
            self.route = transformed_route

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr("Failed to fetch or apply transform: %s" % e)

        # Update the internal route representation with the transformed route
        self.route = transformed_route

    def emergency_button_callback(self, emergency):
        self.btn_emergencia_is_on = True
        if emergency.data:
            self.btn_emergencia = True
            rospy.loginfo('Robot stopping by Emergency')
            rospy.loginfo('Sending emergency stop command')

            if self.robot_type == 'Pioneer':
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

                rospy.signal_shutdown("Pioneer Emergency stop")

            if self.robot_type == 'Solverbot':
                for _ in range(10):
                    message = [0.0, 0.0]
                    stop_cmd = ChannelFloat32(values=message)
                    # Publish the Twist message to stop the robot
                    self.publisher.publish(stop_cmd)

                rospy.signal_shutdown("SolverBot Emergency stop")

    def control_loop(self):
        while not rospy.is_shutdown():
            try:
                trans = self.tf_buffer.lookup_transform(self.world_frame, self.robot_frame, rospy.Time(0))
                translation = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
                quaternion = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logerr(f'{rospy.get_name()} failed to get transform: {e}')
                self.rate.sleep()
                continue

            if self.robot_path_is_on == False or self.btn_emergencia_is_on == False:
                rospy.loginfo(f'{rospy.get_name()}: No path or emergency button found')
                self.rate.sleep()
                continue

            if self.btn_emergencia:
                self.rate.sleep()
                continue

            yaw = tf.transformations.euler_from_quaternion(quaternion)[2]
            robot_pose = np.array([translation[0], translation[1], yaw])

            route = self.route

            if self.path_index >= len(route):
                self.path_index = len(route) -1

            x_desired = route[self.path_index][0]
            y_desired = route[self.path_index][1]

            x_dot_route = route[self.path_index][0] - robot_pose[0]
            y_dot_route = route[self.path_index][1] - robot_pose[1]
            
            distance = np.sqrt((x_dot_route)**2 + (y_dot_route)**2)
            x_dot_route = x_dot_route/distance
            y_dot_route = y_dot_route/distance

            x_d_goal = route[-1][0] - robot_pose[0]
            y_d_goal = route[-1][1] - robot_pose[1]
            distance_to_goal = np.sqrt((x_d_goal)**2 + (y_d_goal)**2)
            
            point = PointStamped()
            point.header.frame_id = self.world_frame
            point.header.stamp = rospy.Time.now()
            point.point.x = x_desired
            point.point.y = y_desired
            point.point.z = self.z_route
            self.publish_which_route_point.publish(point)

            # print(distance_to_goal)

            if (self.path_index >= len(route) - 1) and (distance_to_goal <= self.goal_threshold):
                rospy.loginfo('Path completed')
                self.publisher.publish(self.stop_msg)
                self.rate.sleep()
                continue

            if distance < self.distance_to_change_path_index:
                if self.path_index >= len(route) - 1:
                    self.path_index = len(route) - 1
                else:
                    self.path_index += 1

            X_potential = np.array([self.x_dot, self.y_dot])
            x_dot_desired = np.array([x_dot_route, y_dot_route]) * self.min_velocity
            X_dot_desired = X_potential + x_dot_desired

            desired = [x_desired, y_desired, X_dot_desired[0], X_dot_desired[1]]

            gains = self.pgains
            a = self.a

            if self.robot_type == 'Solverbot':
                right_wheel, left_wheel = self.differential_robot_controller(robot_pose, desired, gains=gains, a=a)
                message = [right_wheel, left_wheel]
                ctrl_msg = ChannelFloat32(values=message)

            if self.robot_type == 'Pioneer':
                reference_linear_velocity, reference_angular_velocity = self.differential_robot_controller(robot_pose, desired, gains=gains, limits=[0.5, 0.5], a=a)
                reference_linear_velocity = reference_linear_velocity/(1+ self.angular_velocity_priority_gain * np.abs(reference_angular_velocity))
                if np.abs(reference_linear_velocity) > self.max_linear_velocity:
                    reference_linear_velocity = np.sign(reference_linear_velocity)*self.max_linear_velocity

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
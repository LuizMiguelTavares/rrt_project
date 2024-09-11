#!/usr/bin/env python3

import rospy
import tf
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import tf.transformations as tft

class GroundTruthPublisher:
    def __init__(self):
        rospy.init_node('ground_truth_publisher', anonymous=True)
        robot_vrpn_topic = rospy.get_param('~robot_vrpn_topic', '/vrpn_client_node/P1/pose')
        self.pub = rospy.Publisher(robot_vrpn_topic, PoseStamped, queue_size=10)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
        rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.robot_frame = rospy.get_param('~robot_frame', 'odom')
        print(f"Robot frame: {self.robot_frame}")
        self.robot_name = rospy.get_param('~robot_name', 'p3dx')
        print(f"Robot name: {self.robot_name}")
        self.frame_id = rospy.get_param('~frame_id', 'map')
        print(f"Frame ID: {self.frame_id}")
        self.rate = rospy.Rate(50)  # 50 Hz
        self.br = tf.TransformBroadcaster()
        self.ground_truth_pose = None
        self.odom_pose = None

    def model_states_callback(self, data):
        try:
            index = data.name.index(self.robot_name)
            self.ground_truth_pose = data.pose[index]
        except ValueError:
            rospy.logwarn("Robot name not found in model states")

    def odom_callback(self, data):
        self.odom_pose = data.pose.pose

    def compute_transform(self):
        if self.ground_truth_pose and self.odom_pose:
            # Compute the difference in position
            delta_x = self.ground_truth_pose.position.x - self.odom_pose.position.x
            delta_y = self.ground_truth_pose.position.y - self.odom_pose.position.y
            delta_z = self.ground_truth_pose.position.z - self.odom_pose.position.z

            # Compute the difference in orientation
            odom_orientation = [
                self.odom_pose.orientation.x,
                self.odom_pose.orientation.y,
                self.odom_pose.orientation.z,
                self.odom_pose.orientation.w
            ]

            ground_truth_orientation = [
                self.ground_truth_pose.orientation.x,
                self.ground_truth_pose.orientation.y,
                self.ground_truth_pose.orientation.z,
                self.ground_truth_pose.orientation.w
            ]

            odom_matrix = tft.quaternion_matrix(odom_orientation)
            ground_truth_matrix = tft.quaternion_matrix(ground_truth_orientation)

            delta_matrix = tft.concatenate_matrices(tft.inverse_matrix(odom_matrix), ground_truth_matrix)
            delta_orientation = tft.quaternion_from_matrix(delta_matrix)

            return (delta_x, delta_y, delta_z), delta_orientation
        else:
            return None, None

    def publish_transform(self):
        delta_position, delta_orientation = self.compute_transform()
        if delta_position is not None and delta_orientation is not None:
            self.br.sendTransform(
                delta_position,
                delta_orientation,
                rospy.Time.now(),
                self.robot_frame,  # Child frame
                self.frame_id  # Parent frame
            )

            # Publish the corrected pose to the VRPN topic
            corrected_pose_stamped = PoseStamped()
            corrected_pose_stamped.header.stamp = rospy.Time.now()
            corrected_pose_stamped.header.frame_id = self.frame_id
            corrected_pose_stamped.pose.position.x = self.ground_truth_pose.position.x 
            corrected_pose_stamped.pose.position.y = self.ground_truth_pose.position.y
            corrected_pose_stamped.pose.position.z = self.ground_truth_pose.position.z 
            corrected_pose_stamped.pose.orientation.x = self.ground_truth_pose.orientation.x
            corrected_pose_stamped.pose.orientation.y = self.ground_truth_pose.orientation.y
            corrected_pose_stamped.pose.orientation.z = self.ground_truth_pose.orientation.z
            corrected_pose_stamped.pose.orientation.w = self.ground_truth_pose.orientation.w
            self.pub.publish(corrected_pose_stamped)

    def run(self):
        while not rospy.is_shutdown():
            self.publish_transform()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        ground_truth_publisher = GroundTruthPublisher()
        ground_truth_publisher.run()
    except rospy.ROSInterruptException:
        pass

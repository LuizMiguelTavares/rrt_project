#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Dense>
#include <sensor_msgs/ChannelFloat32.h>
#include <cmath>

class DifferentialController {
public:
    DifferentialController() : tfListener(tfBuffer) {
        ros::NodeHandle nh("~");
        ros::NodeHandle nh_;

        // Variables initialization
        x_dot_obs = y_dot_obs = 0.0;
        robot_path_is_on = robot_pose_is_on = btn_emergencia_is_on = false;
        path_index = 0;
        a = 0.15;

        // Getting parameters from the server
        nh.param("reference_velocity", reference_velocity, 0.1);
        nh.param("control_frequency", control_frequency, 30.0);
        nh.param("obs_filter_gain", obs_filter_gain, 0.7);
        nh.param("world_frame", world_frame, std::string("world_frame"));
        nh.param("path_topic", path_topic, std::string("path"));
        nh.param("pose_topic", pose_topic, std::string("/vrpn_client_node/L1/pose"));
        nh.param("angular_gain", angular_gain, 2.0);
        nh.param("angular_filter_gain", angular_filter_gain, 0.6);
        nh.param("use_X_obs_on_psi_control", use_X_obs_on_psi_control, false);


        nh.param("a", a, 0.15);

        nh.getParam("robot_control_topic", robot_control_topic);

        // Gain parameters
        nh.getParam("gains/x", pgains_x);
        nh.getParam("gains/y", pgains_y);
        pgains = Eigen::Vector2d(pgains_x, pgains_y);

        // Publishers and subscribers
        control_pub = nh_.advertise<geometry_msgs::Twist>(robot_control_topic, 10);
        control_pub_no_potential = nh_.advertise<geometry_msgs::Twist>(robot_control_topic + "_no_potential", 10);
        control_point_pub = nh_.advertise<geometry_msgs::PointStamped>("control_point", 10);
        route_point_pub = nh_.advertise<geometry_msgs::PointStamped>("which_route_point", 10);

        pose_sub = nh_.subscribe(pose_topic, 10, &DifferentialController::poseCallback, this);
        path_sub = nh_.subscribe(path_topic, 10, &DifferentialController::routeCallback, this);
        potential_sub = nh_.subscribe("/potential", 10, &DifferentialController::potentialCallback, this);
        emergency_sub = nh_.subscribe("/emergency_flag", 10, &DifferentialController::emergencyCallback, this);

        // Initialize stop messages
        stop_msg.linear.x = 0.0;
        stop_msg.linear.y = 0.0;
        stop_msg.linear.z = 0.0;
        stop_msg.angular.x = 0.0;
        stop_msg.angular.y = 0.0;
        stop_msg.angular.z = 0.0;

        last_X_obs_dot = Eigen::Vector2d::Zero();
        last_x_dot_ref_psi = 0.0;
        last_x_dot_ref_psi = 0.0;
        rate = new ros::Rate(control_frequency);
        ROS_INFO("Differential controller started.");
    }

    ~DifferentialController() {
        delete rate;
    }

    void controlLoop() {
        while (ros::ok()) {
            ros::spinOnce();

            if (!robot_pose_is_on || !robot_path_is_on || !btn_emergencia_is_on) {
                rate->sleep();
                continue;
            }

            if (btn_emergencia) {
                control_pub.publish(stop_msg);
                ROS_WARN("Emergency stop activated.");
                rate->sleep();
                continue;
            }

            // Get the robot's current pose and yaw
            Eigen::Vector3d robot_position(robot_pose.pose.position.x, robot_pose.pose.position.y, robot_pose.pose.position.z);
            tf2::Quaternion q(
                robot_pose.pose.orientation.x,
                robot_pose.pose.orientation.y,
                robot_pose.pose.orientation.z,
                robot_pose.pose.orientation.w
            );

            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

            // Compute robot control pose
            Eigen::Vector3d robot_pose_control = robot_position + Eigen::Vector3d(std::cos(yaw) * a, std::sin(yaw) * a, 0.0);

            // Find the closest point on the route
            auto [closest_point, closest_idx] = findClosestPoint(robot_pose_control, route);

            geometry_msgs::PointStamped route_point;
            route_point.header.stamp = ros::Time::now();
            route_point.header.frame_id = robot_pose.header.frame_id;
            route_point.point.x = closest_point(0);
            route_point.point.y = closest_point(1);

            route_point_pub.publish(route_point);


            if (path_index > route.size() - 1) {
                path_index = route.size() - 1;
            }

            Eigen::Vector2d X_dot_desired = route_dx[closest_idx];
            Eigen::Vector2d X_til = (closest_point - robot_pose_control.head<2>());

            // Compute the reference velocity based on the control gains
            Eigen::Matrix2d gains;
            gains << pgains(0), 0, 0, pgains(1);
            Eigen::Vector2d X_dot_ref = X_dot_desired + gains * X_til;

            // If potential field is active, modify velocities
            Eigen::Vector2d X_dot_obs(x_dot_obs, y_dot_obs);
            X_dot_obs = obs_filter_gain * X_dot_obs + (1 - obs_filter_gain) * last_X_obs_dot;
            last_X_obs_dot = X_dot_obs;

            // Compute control
            Eigen::Matrix2d rotation_matrix_bw;
            rotation_matrix_bw << std::cos(yaw), -std::sin(yaw),
                                std::sin(yaw), std::cos(yaw);

            Eigen::Matrix2d rotation_matrix_wb;
            rotation_matrix_wb << std::cos(yaw), std::sin(yaw),
                                -std::sin(yaw), std::cos(yaw);

            Eigen::Vector2d X_dot_ref_path_w = X_dot_ref;
            Eigen::Vector2d X_dot_obs_ref_w = rotation_matrix_bw * X_dot_obs;

            Eigen::Vector2d X_dot_ref_w = X_dot_obs_ref_w + X_dot_ref_path_w;
            Eigen::Vector2d uw = rotation_matrix_wb * X_dot_ref_w;
            
            // Eigen::Vector2d uw = computeControl(yaw, X_dot_ref, X_dot_obs);
            
            double ref_linear_velocity_x = uw(0);
            double ref_linear_velocity_y = uw(1);
            double psi_desired;

            if (use_X_obs_on_psi_control){
                double x_dot_ref_psi = X_dot_ref_w(0) * angular_filter_gain + (1 - angular_filter_gain) * last_x_dot_ref_psi;
                double y_dot_ref_psi = X_dot_ref_w(1) * angular_filter_gain + (1 - angular_filter_gain) * last_y_dot_ref_psi;
                psi_desired = std::atan2(y_dot_ref_psi, x_dot_ref_psi);
                last_x_dot_ref_psi = x_dot_ref_psi;
                last_y_dot_ref_psi = y_dot_ref_psi;
            } else {
                psi_desired = std::atan2(X_dot_ref_path_w(1), X_dot_ref_path_w(0));
            }
            
            double psi_til = psi_desired - yaw;

            if (psi_til > M_PI) {
                psi_til -= 2 * M_PI;
            }
            if (psi_til < -M_PI) {
                psi_til += 2 * M_PI;
            }

            double ref_angular_velocity = angular_gain*psi_til;

            geometry_msgs::Twist ctrl_msg;
            ctrl_msg.linear.x = ref_linear_velocity_x;
            ctrl_msg.linear.y = ref_linear_velocity_y;
            ctrl_msg.angular.z = ref_angular_velocity;
            control_pub.publish(ctrl_msg);

            // Eigen::Vector2d uw_no_potential = computeControl(yaw, X_dot_ref, {0.0, 0.0});
            // double ref_linear_velocity_no_potential = uw_no_potential(0);
            // double ref_angular_velocity_no_potential = uw_no_potential(1);

            // geometry_msgs::Twist ctrl_msg_no_potential;
            // ctrl_msg_no_potential.linear.x = ref_linear_velocity_no_potential;
            // ctrl_msg_no_potential.angular.z = ref_angular_velocity_no_potential;
            // control_pub_no_potential.publish(ctrl_msg_no_potential);

            rate->sleep();
        }
    }

private:
    // ROS handles
    ros::Publisher control_pub, control_pub_no_potential, control_point_pub, route_point_pub;
    ros::Subscriber pose_sub, path_sub, potential_sub, emergency_sub;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    ros::Rate *rate;
    geometry_msgs::Twist stop_msg;
    geometry_msgs::PoseStamped robot_pose;

    // Variables
    double x_dot_obs, y_dot_obs;
    double x_dot_world, y_dot_world;
    double reference_velocity;
    double obs_filter_gain, angular_gain, angular_filter_gain;
    double a;
    Eigen::Vector2d last_X_obs_dot;
    double last_x_dot_ref_psi, last_y_dot_ref_psi;

    bool btn_emergencia, robot_path_is_on, robot_pose_is_on, btn_emergencia_is_on, use_X_obs_on_psi_control;
    std::vector<Eigen::Vector2d> route, route_dx;
    int path_index = 0;

    double control_frequency;
    std::string world_frame, path_topic, pose_topic, robot_control_topic;
    Eigen::Vector2d pgains;
    double pgains_x, pgains_y;

    void potentialCallback(const geometry_msgs::Point::ConstPtr& msg) {
        x_dot_obs = msg->x;
        y_dot_obs = msg->y;
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg) {
        robot_pose = *pose_msg;
        robot_pose_is_on = true;
    }

    void emergencyCallback(const std_msgs::Bool::ConstPtr& emergency_msg) {
        btn_emergencia_is_on = true;
        btn_emergencia = emergency_msg->data;
        if (btn_emergencia) {
            ROS_WARN("Emergency stop triggered");
        }
    }

    void routeCallback(const nav_msgs::Path::ConstPtr& route_data) {
        if (!route_data->poses.empty()) {
            try {
                geometry_msgs::TransformStamped transform = tfBuffer.lookupTransform(world_frame, route_data->header.frame_id, ros::Time(0), ros::Duration(1.0));

                Eigen::Vector3d translation(transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);
                tf2::Quaternion q(transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w);
                tf2::Matrix3x3 rotation_matrix(q);

                // Convert tf2::Matrix3x3 to Eigen::Matrix2d for compatibility with Eigen::Vector2d
                Eigen::Matrix2d rotation_matrix_eigen;
                rotation_matrix_eigen << rotation_matrix[0][0], rotation_matrix[0][1],
                                        rotation_matrix[1][0], rotation_matrix[1][1];

                route.clear();
                route_dx.clear();

                for (const auto& pose : route_data->poses) {
                    Eigen::Vector2d position(pose.pose.position.x, pose.pose.position.y);
                    Eigen::Vector2d transformed_position = rotation_matrix_eigen * position.head<2>() + translation.head<2>();

                    route.push_back(transformed_position);

                    if (route.size() > 1) {
                        Eigen::Vector2d dx_vector = route.back() - *(route.end() - 2);
                        Eigen::Vector2d unitary_dx_vector = dx_vector.normalized();
                        route_dx.push_back(unitary_dx_vector * reference_velocity);
                    }
                }
                route_dx.push_back(Eigen::Vector2d::Zero());

                robot_path_is_on = true;
            } catch (tf2::TransformException& ex) {
                ROS_WARN("Could not transform: %s", ex.what());
            }
        }
    }

    std::pair<Eigen::Vector2d, int> findClosestPoint(const Eigen::Vector3d& robot_pose, const std::vector<Eigen::Vector2d>& route) {
        double min_dist_sqr = std::numeric_limits<double>::max();
        Eigen::Vector2d closest_point;
        int closest_idx = 0;

        double prev_dist_sqr = std::numeric_limits<double>::max();
        for (int i = 0; i < route.size(); ++i) {
            double dist_sqr = (route[i] - robot_pose.head<2>()).squaredNorm();
            if (dist_sqr < min_dist_sqr) {
                min_dist_sqr = dist_sqr;
                closest_point = route[i];
                closest_idx = i;
            } else if (dist_sqr > prev_dist_sqr) {
                break;
            }
            prev_dist_sqr = dist_sqr;
        }

        return std::make_pair(closest_point, closest_idx);
    }

    // Eigen::Vector2d computeControl(double yaw, const Eigen::Vector2d& X_dot_ref, const Eigen::Vector2d& X_dot_obs) {
    //     Eigen::Matrix2d rotation_matrix_bw;
    //     rotation_matrix_bw << std::cos(yaw), -std::sin(yaw),
    //                           std::sin(yaw), std::cos(yaw);

    //     Eigen::Matrix2d rotation_matrix_wb;
    //     rotation_matrix_wb << std::cos(yaw), std::sin(yaw),
    //                           -std::sin(yaw), std::cos(yaw);

    //     Eigen::Vector2d X_dot_ref_path_w = X_dot_ref;
    //     Eigen::Vector2d X_dot_obs_ref_w = rotation_matrix_bw * X_dot_obs;

    //     Eigen::Vector2d X_dot_ref_w = X_dot_obs_ref_w + X_dot_ref_path_w;
    //     return rotation_matrix_wb * X_dot_ref_w;
    // }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "differential_controller");

    DifferentialController controller;
    controller.controlLoop();

    return 0;
}
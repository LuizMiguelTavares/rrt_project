#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/core.hpp>
#include <cmath>
#include <vector>
#include <mutex>

#include "rrt_simple_ptr.hpp"

class MapPathSubscriber {
public:
    MapPathSubscriber() : tf_listener_(tf_buffer_), found_path_(false) {
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");

        getParamOrThrow(private_nh, "path_topic", path_topic_);
        getParamOrThrow(private_nh, "rate", rate_);
        getParamOrThrow(private_nh, "num_nodes", num_nodes_);
        getParamOrThrow(private_nh, "step_size", step_size_);
        getParamOrThrow(private_nh, "goal_threshold", goal_threshold_);
        getParamOrThrow(private_nh, "bias_probability", bias_probability_);
        getParamOrThrow(private_nh, "radius", radius_);
        getParamOrThrow(private_nh, "world_frame", world_frame_);
        getParamOrThrow(private_nh, "robot_frame", robot_frame_);
        getParamOrThrow(private_nh, "x_goal", x_goal_);
        getParamOrThrow(private_nh, "y_goal", y_goal_);

        path_pub_ = nh.advertise<nav_msgs::Path>(path_topic_, 10);
        ros::service::waitForService("/static_map");
        static_map_client_ = nh.serviceClient<nav_msgs::GetMap>("/static_map");

        merged_map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/merged_map", 10);
    }

    bool fetchStaticMap() {
        nav_msgs::GetMap srv;
        if (static_map_client_.call(srv)) {
            map_ = srv.response.map;
            radius_pixel_ = std::round(radius_ / map_.info.resolution);
            return true;
        } else {
            ROS_ERROR("Failed to call /static_map service");
            return false;
        }
    }

    void run() {
        ros::Rate rate(rate_);
        while (ros::ok()) {
            ros::spinOnce();
            if (!map_.data.empty()) {
                if (!found_path_) {
                    generateRRT();
                } else {
                    path_pub_.publish(ros_path_);
                }
            } else {
                if (fetchStaticMap()) {
                    ROS_INFO("Successfully fetched the static map.");
                }
            }
            rate.sleep();
        }
    }

private:
    template<typename T>
    void getParamOrThrow(ros::NodeHandle& nh, const std::string& param_name, T& param_value) {
        if (!nh.getParam(param_name, param_value)) {
            std::string error_msg = ros::this_node::getName() + ": Required parameter '" + param_name + "' not set.";
            ROS_ERROR_STREAM(error_msg);
            throw std::runtime_error(error_msg);
        }
    }

    cv::Mat occupancyGridToCvMat(const nav_msgs::OccupancyGrid& grid) {
        std::lock_guard<std::mutex> lock(local_map_mutex);
        cv::Mat mat(grid.info.width, grid.info.height, CV_8UC1);

        for (unsigned int y = 0; y < grid.info.height; y++) {
            for (unsigned int x = 0; x < grid.info.width; x++) {
                int i = x + y * grid.info.width;
                if (grid.data[i] == -1) {
                    mat.at<uchar>(x, grid.info.height - 1 - y) = 127;
                } else if (grid.data[i] == 0) {
                    mat.at<uchar>(x, grid.info.height - 1 - y) = 255;
                } else if (grid.data[i] == 100) {
                    mat.at<uchar>(x, grid.info.height - 1 - y) = 0;
                }
            }
        }

        cv::Mat rotated;
        cv::rotate(mat, rotated, cv::ROTATE_90_CLOCKWISE);
        cv::flip(rotated, rotated, 1);

        return rotated;
    }

    void generateRRT() {
        geometry_msgs::PoseStamped robot_pose;
        robot_pose.header.frame_id = robot_frame_;
        robot_pose.header.stamp = ros::Time::now();
        robot_pose.pose.position.x = 0.0;
        robot_pose.pose.position.y = 0.0;
        robot_pose.pose.position.z = 0.0;
        robot_pose.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, 0, 1));

        geometry_msgs::PoseStamped goal_pose_world;
        goal_pose_world.header.frame_id = world_frame_;
        goal_pose_world.header.stamp = ros::Time::now();
        goal_pose_world.pose.position.x = x_goal_;
        goal_pose_world.pose.position.y = y_goal_;
        goal_pose_world.pose.position.z = 0.0;
        goal_pose_world.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, 0, 1));

        geometry_msgs::PoseStamped start_pose_map;
        geometry_msgs::PoseStamped goal_pose_map;

        try {
            geometry_msgs::TransformStamped transformRobotToMap = tf_buffer_.lookupTransform(map_.header.frame_id, robot_frame_, ros::Time(0), ros::Duration(3.0));
            tf2::doTransform(robot_pose, start_pose_map, transformRobotToMap);
            ROS_INFO("Transformed start pose in map frame: x: %f, y: %f, z: %f", start_pose_map.pose.position.x, start_pose_map.pose.position.y, start_pose_map.pose.position.z);

            geometry_msgs::TransformStamped transformWorldToMap = tf_buffer_.lookupTransform(map_.header.frame_id, world_frame_, ros::Time(0), ros::Duration(3.0));
            tf2::doTransform(goal_pose_world, goal_pose_map, transformWorldToMap);
            ROS_INFO("Transformed goal pose in map frame: x: %f, y: %f, z: %f", goal_pose_map.pose.position.x, goal_pose_map.pose.position.y, goal_pose_map.pose.position.z);
        } catch (tf2::TransformException &ex) {
            ROS_WARN("Transform failed: %s", ex.what());
            return;
        }

        cv::Mat grid_map = occupancyGridToCvMat(map_);

        double x_start = std::round((start_pose_map.pose.position.x - map_.info.origin.position.x) / map_.info.resolution);
        double y_start = std::round((start_pose_map.pose.position.y - map_.info.origin.position.y) / map_.info.resolution);
        y_start = map_.info.height - y_start;

        double x_goal = std::round((goal_pose_map.pose.position.x - map_.info.origin.position.x) / map_.info.resolution);
        double y_goal = std::round((goal_pose_map.pose.position.y - map_.info.origin.position.y) / map_.info.resolution);
        y_goal = map_.info.height - y_goal;

        ROS_INFO("Start point in local map: x: %f, y: %f", x_start, y_start);
        ROS_INFO("Goal point in local map: x: %f, y: %f", x_goal, y_goal);

        if (x_start < 0 || x_start >= map_.info.width || y_start < 0 || y_start >= map_.info.height) {
            ROS_ERROR("%s: Start point outside the local map.", ros::this_node::getName().c_str());
            return;
        }

        if (x_goal < 0 || x_goal >= map_.info.width || y_goal < 0 || y_goal >= map_.info.height) {
            ROS_ERROR("%s: Goal point outside the local map.", ros::this_node::getName().c_str());
            return;
        }

        std::shared_ptr<motion_planning::Node> start_node = std::make_shared<motion_planning::Node>(x_start, y_start, nullptr);
        std::shared_ptr<motion_planning::Node> goal_node = std::make_shared<motion_planning::Node>(x_goal, y_goal, nullptr);
        std::vector<std::shared_ptr<motion_planning::Node>> nodes;
        nodes = motion_planning::rrt(grid_map, start_node, goal_node, num_nodes_, step_size_, goal_threshold_, bias_probability_, radius_pixel_);

        if (nodes.empty()) {
            ROS_ERROR("No path found from RRT within the local map.");
            return;
        }

        std::vector<std::shared_ptr<motion_planning::Node>> goal_path;
        if (nodes.back()->x != goal_node->x || nodes.back()->y != goal_node->y) {
            ROS_ERROR("No local path found!");
            return;
        } else {
            goal_path = motion_planning::trace_goal_path(nodes.back());
            found_path_ = true;
        }

        ros_path_ = convertNodesToPath(goal_path, map_);
    }

    nav_msgs::Path convertNodesToPath(const std::vector<std::shared_ptr<motion_planning::Node>> nodes, const nav_msgs::OccupancyGrid map) {
        nav_msgs::Path path;
        path.header.frame_id = map.header.frame_id;
        path.header.stamp = ros::Time::now();

        for (auto node : nodes) {
            if (node == nullptr) continue;
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = map.header.frame_id;
            pose_stamped.header.stamp = ros::Time::now();

            pose_stamped.pose.position.x = node->x * map.info.resolution + map.info.origin.position.x;
            pose_stamped.pose.position.y = (map.info.height - node->y) * map.info.resolution + map.info.origin.position.y;
            pose_stamped.pose.position.z = 0;

            pose_stamped.pose.orientation.x = 0.0;
            pose_stamped.pose.orientation.y = 0.0;
            pose_stamped.pose.orientation.z = 0.0;
            pose_stamped.pose.orientation.w = 1.0;

            path.poses.push_back(pose_stamped);
        }

        return path;
    }

    bool found_path_;
    nav_msgs::Path ros_path_;
    std::mutex local_map_mutex;
    int rate_;
    std::string path_topic_;
    int num_nodes_;
    double step_size_;
    double goal_threshold_;
    double bias_probability_;
    double radius_;
    int radius_pixel_;
    std::string world_frame_;
    std::string robot_frame_;
    double x_goal_;
    double y_goal_;

    ros::Publisher path_pub_;
    ros::ServiceClient static_map_client_;

    nav_msgs::OccupancyGrid map_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    ros::Publisher merged_map_pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_path_subscriber");
    MapPathSubscriber mps;
    mps.run();
    return 0;
}
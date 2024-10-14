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

#include "rrt_star.hpp"

class MapPathSubscriber {
public:
    MapPathSubscriber() : tf_listener_(tf_buffer_), found_path_(false), rrtstar_(), best_path_cost_(-1.0), start_and_goal_set(false), first_path_found_(false) {
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

        rrtstar_.setMaxIterations(num_nodes_);
        rrtstar_.setStepSize(step_size_);
        rrtstar_.setDestinationThreshold(goal_threshold_);
    }

    bool fetchStaticMap() {
        nav_msgs::GetMap srv;
        if (static_map_client_.call(srv)) {
            map_ = srv.response.map;
            radius_pixel_ = std::round(radius_ / map_.info.resolution);
            rrtstar_.setRadius(radius_pixel_);
            cv::Mat grid_map = occupancyGridToCvMat(map_);
            rrtstar_.setMap(grid_map);

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
                } 
                
                if (rrtstar_.reached()) {
                    if (ros_path_.poses.size() > 0) {
                        path_pub_.publish(ros_path_);
                    }
                    
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
            // ROS_INFO("Transformed start pose in map frame: x: %f, y: %f, z: %f", start_pose_map.pose.position.x, start_pose_map.pose.position.y, start_pose_map.pose.position.z);

            geometry_msgs::TransformStamped transformWorldToMap = tf_buffer_.lookupTransform(map_.header.frame_id, world_frame_, ros::Time(0), ros::Duration(3.0));
            tf2::doTransform(goal_pose_world, goal_pose_map, transformWorldToMap);
            // ROS_INFO("Transformed goal pose in map frame: x: %f, y: %f, z: %f", goal_pose_map.pose.position.x, goal_pose_map.pose.position.y, goal_pose_map.pose.position.z);
        } catch (tf2::TransformException &ex) {
            ROS_WARN("Transform failed: %s", ex.what());
            return;
        }

        double x_start = std::round((start_pose_map.pose.position.x - map_.info.origin.position.x) / map_.info.resolution);
        double y_start = std::round((start_pose_map.pose.position.y - map_.info.origin.position.y) / map_.info.resolution);
        y_start = map_.info.height - y_start;

        double x_goal = std::round((goal_pose_map.pose.position.x - map_.info.origin.position.x) / map_.info.resolution);
        double y_goal = std::round((goal_pose_map.pose.position.y - map_.info.origin.position.y) / map_.info.resolution);
        y_goal = map_.info.height - y_goal;

        // ROS_INFO("Start point in local map: x: %f, y: %f", x_start, y_start);
        // ROS_INFO("Goal point in local map: x: %f, y: %f", x_goal, y_goal);

        if (x_start < 0 || x_start >= map_.info.width || y_start < 0 || y_start >= map_.info.height) {
            ROS_ERROR("%s: Start point outside the local map.", ros::this_node::getName().c_str());
            return;
        }

        if (x_goal < 0 || x_goal >= map_.info.width || y_goal < 0 || y_goal >= map_.info.height) {
            ROS_ERROR("%s: Goal point outside the local map.", ros::this_node::getName().c_str());
            return;
        }

        if (!start_and_goal_set){
            rrtstar_.setStartPoint(x_start, y_start);
            rrtstar_.setDestination(x_goal, y_goal);
            start_and_goal_set = true;
        }

        if (first_path_found_) {
            ROS_INFO("rrt_star_path: Reached the goal point.");
            int iter = rrtstar_.getCurrentIterations();
            
            rrtstar_.setMaxIterations(iter + 10000);
            std::vector<rrt_star::Point> new_path;
            new_path = rrtstar_.planner();

            if (rrtstar_.getBestPathCost() < best_path_cost_){
                ROS_INFO("Found a better path in %i iterations", rrtstar_.getCurrentIterations() - iter );
                ROS_INFO("Old best path cost: %f", best_path_cost_);
                goal_path = new_path;
                best_path_cost_ = rrtstar_.getBestPathCost();
                ROS_INFO("New best path cost: %f", best_path_cost_);
            } else {
                found_path_ = true;
                return;
            }
        } else {
            ROS_INFO("Generating a new path.");
            goal_path = rrtstar_.planner();
            if (rrtstar_.reached()) {
                best_path_cost_ = rrtstar_.getBestPathCost();
                first_path_found_ = true;
            } else {
                ROS_ERROR("rrt_star_path: Failed to find a rrt star path.");
            }
        }

        ROS_INFO("rrtstar_.reached(): %d", rrtstar_.reached());

        ros_path_ = convertPointToPath(goal_path, map_);
        ROS_INFO("Number of nodes in the RRT path before: %d", ros_path_.poses.size());

        double max_distance = 0.02;
        ros_path_ = interpolatePath(ros_path_, max_distance);

        // Plot the number of nodes in the RRT path
        ROS_INFO("Number of nodes in the RRT path after: %d", ros_path_.poses.size());

        if (ros_path_.poses.size() > 0){
            path_pub_.publish(ros_path_);
            }
    }

    nav_msgs::Path convertPointToPath(const std::vector<rrt_star::Point> points, const nav_msgs::OccupancyGrid map) {
        nav_msgs::Path path;
        path.header.frame_id = map.header.frame_id;
        path.header.stamp = ros::Time::now();

        std::vector<rrt_star::Point> reversed_points = points;
        std::reverse(reversed_points.begin(), reversed_points.end());

        for (auto point : reversed_points) {
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = map.header.frame_id;
            pose_stamped.header.stamp = ros::Time::now();

            pose_stamped.pose.position.x = point.m_x * map.info.resolution + map.info.origin.position.x;
            pose_stamped.pose.position.y = (map.info.height - point.m_y) * map.info.resolution + map.info.origin.position.y;
            pose_stamped.pose.position.z = 0;

            pose_stamped.pose.orientation.x = 0.0;
            pose_stamped.pose.orientation.y = 0.0;
            pose_stamped.pose.orientation.z = 0.0;
            pose_stamped.pose.orientation.w = 1.0;

            path.poses.push_back(pose_stamped);
        }

        return path;
    }

    nav_msgs::Path interpolatePath(const nav_msgs::Path& path, double max_distance) {
        nav_msgs::Path interpolated_path;
        interpolated_path.header = path.header;

        for (size_t i = 0; i < path.poses.size() - 1; ++i) {
            const geometry_msgs::PoseStamped& pose1 = path.poses[i];
            const geometry_msgs::PoseStamped& pose2 = path.poses[i + 1];

            interpolated_path.poses.push_back(pose1);

            double dx = pose2.pose.position.x - pose1.pose.position.x;
            double dy = pose2.pose.position.y - pose1.pose.position.y;
            double dz = pose2.pose.position.z - pose1.pose.position.z;
            double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

            if (distance > max_distance) {
                int num_intermediate_points = static_cast<int>(std::ceil(distance / max_distance)) - 1;
                double step_x = dx / (num_intermediate_points + 1);
                double step_y = dy / (num_intermediate_points + 1);
                double step_z = dz / (num_intermediate_points + 1);

                for (int j = 1; j <= num_intermediate_points; ++j) {
                    geometry_msgs::PoseStamped intermediate_pose;
                    intermediate_pose.header = interpolated_path.header;
                    intermediate_pose.pose.position.x = pose1.pose.position.x + step_x * j;
                    intermediate_pose.pose.position.y = pose1.pose.position.y + step_y * j;
                    intermediate_pose.pose.position.z = pose1.pose.position.z + step_z * j;
                    // Interpolate orientation if needed
                    intermediate_pose.pose.orientation = pose1.pose.orientation;
                    interpolated_path.poses.push_back(intermediate_pose);
                }
            }
        }

        // Add the last pose
        interpolated_path.poses.push_back(path.poses.back());
        return interpolated_path;
    }


    std::vector<rrt_star::Point> goal_path;
    bool first_path_found_;
    bool start_and_goal_set;
    float best_path_cost_;
    rrt_star::RRTStar rrtstar_;
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
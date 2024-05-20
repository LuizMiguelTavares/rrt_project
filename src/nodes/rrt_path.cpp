#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "rrt_simple_source.hpp"
#include <cmath>
#include <optional>
#include <vector>
#include <memory>
#include <opencv2/core.hpp>
#include <thread>

class RoutePublisher
{
public:
    RoutePublisher() : last_map_pruning_subscriber_count(0), rrtsuccess(false){
        ros::NodeHandle nh("~");
        ros::NodeHandle nh_global;

        double tmp_step_size, tmp_goal_threshold, tmp_bias_probability;
        float tmp_radius;

        tf_listener = std::make_unique<tf2_ros::TransformListener>(tf_buffer);
        nh.param<std::string>("path_topic", path_topic, "path");

        if (nh.getParam("num_nodes", num_nodes) && nh.getParam("step_size", tmp_step_size) && nh.getParam("goal_threshold", tmp_goal_threshold) && nh.getParam("bias_probability", tmp_bias_probability) && nh.getParam("radius", tmp_radius)) {
            step_size = tmp_step_size;
            goal_threshold = tmp_goal_threshold;
            bias_probability = tmp_bias_probability;
            radius_meter = tmp_radius;
        } else {
            ROS_ERROR("Parameters not provided!");
            ros::shutdown();
            return;
        }

        if (nh.getParam("robot_frame", robot_frame)) {
            ROS_INFO("Robot frame: %s", robot_frame.c_str());
        } else {
            ROS_ERROR("Robot frame not provided!");
            ros::shutdown();
            return;
        }

        double tmp_goal_x, tmp_goal_y;
        if (nh.getParam("x_goal", tmp_goal_x) && nh.getParam("y_goal", tmp_goal_y)) {
            goal_position = {tmp_goal_x, tmp_goal_y};
        } else {
            ROS_ERROR("Goal position not provided!");
            ros::shutdown();
            return;
        }

        rate = std::make_unique<ros::Rate>(10);

        std::string ns = ros::this_node::getNamespace();
        nav_msgs_path_pub = nh_global.advertise<nav_msgs::Path>(path_topic, 10);
        pruned_map_pub = nh_global.advertise<nav_msgs::OccupancyGrid>("pruned_map", 10);
        map_subscriber = nh_global.subscribe("/map", 10, &RoutePublisher::map_callback, this);

        ROS_INFO_STREAM(ns.substr(1) << " route publisher node started!");
    }

    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(map_mutex);
        if (msg->data.empty()) {
            ROS_ERROR("Empty map received!");
            return;
        }

        if (map.data.empty()){
            radius_pixel = std::round(radius_meter / msg->info.resolution);
            ROS_INFO("Radius in pixels: %d", radius_pixel);
        }

        ROS_INFO("Map received!");
        ROS_INFO("Map Metadata: \nResolution: %f\nWidth: %d\nHeight: %d\nOrigin: (%f, %f, %f)", 
                 msg->info.resolution, msg->info.width, msg->info.height, 
                 msg->info.origin.position.x, msg->info.origin.position.y, msg->info.origin.position.z);

        map = *msg;
        // pruned_map = pruneMap(map, -3, 3, -3, 3);
        pruned_map = map;

        ROS_INFO("Pruned Map Metadata: \nResolution: %f\nWidth: %d\nHeight: %d\nOrigin: (%f, %f, %f)", 
                 pruned_map.info.resolution, pruned_map.info.width, pruned_map.info.height, 
                 pruned_map.info.origin.position.x, pruned_map.info.origin.position.y, pruned_map.info.origin.position.z);
        
        pruned_map_pub.publish(pruned_map);
    }

    void updateStartPositionFromTF(const std::string& map_frame, const std::string& pose_frame) {
        geometry_msgs::TransformStamped transform_stamped;
        try {
            transform_stamped = tf_buffer.lookupTransform(map_frame, pose_frame, ros::Time(0), ros::Duration(1.0));
            start_position = {transform_stamped.transform.translation.x, transform_stamped.transform.translation.y};
            ROS_INFO("Updated start position from TF (x: %f, y: %f)", start_position[0], start_position[1]);
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }
    }

    void rrt_route() {
        while (ros::ok()) {
            check_pruning_map_subscribers();

            if (map.data.empty()) {
                ROS_INFO("RRT_node: Waiting for map...");
                rate->sleep();
                ros::spinOnce();
                continue;
            }

            if (start_position.empty()) {
                updateStartPositionFromTF(map.header.frame_id, robot_frame);
                if (start_position.empty()) {
                    ROS_INFO("RRT_node: Waiting for initial position...");
                    rate->sleep();
                    ros::spinOnce();
                    continue;
                }
            }

            // if(!rrtsuccess)
            if(!rrtsuccess){
                std::lock_guard<std::mutex> lock(map_mutex);
                cv_map = occupancyGridToCvMat(pruned_map);

                // See if the start and goal positions are valid on the pruned map
                if (start_position[0] < pruned_map.info.origin.position.x || start_position[0] > pruned_map.info.origin.position.x + pruned_map.info.width * pruned_map.info.resolution ||
                    start_position[1] < pruned_map.info.origin.position.y || start_position[1] > pruned_map.info.origin.position.y + pruned_map.info.height * pruned_map.info.resolution) {
                    ROS_ERROR("Start position is outside the map!");
                    continue;
                }

                if (goal_position[0] < pruned_map.info.origin.position.x || goal_position[0] > pruned_map.info.origin.position.x + pruned_map.info.width * pruned_map.info.resolution ||
                    goal_position[1] < pruned_map.info.origin.position.y || goal_position[1] > pruned_map.info.origin.position.y + pruned_map.info.height * pruned_map.info.resolution) {
                    ROS_ERROR("Goal position is outside the map!");
                    continue;
                }

                // Choose real position on the map given the start and goal positions and the resolution
                std::vector<double> start_position_grid = {std::round((start_position[0] - pruned_map.info.origin.position.x) / pruned_map.info.resolution), 
                                                        std::round(pruned_map.info.height - ((start_position[1] - pruned_map.info.origin.position.y) / pruned_map.info.resolution))};


                std::vector<double> goal_position_grid = {std::round((goal_position[0] - pruned_map.info.origin.position.x) / pruned_map.info.resolution),
                                                        std::round(pruned_map.info.height - ((goal_position[1] - pruned_map.info.origin.position.y) / pruned_map.info.resolution))};

                ROS_INFO("Start position: (%f, %f)", start_position_grid[0], start_position_grid[1]);
                ROS_INFO("Goal position: (%f, %f)", goal_position_grid[0], goal_position_grid[1]);

                std::unique_ptr<motion_planning::Node> start = std::make_unique<motion_planning::Node>(start_position_grid, nullptr);
                std::unique_ptr<motion_planning::Node> goal = std::make_unique<motion_planning::Node>(goal_position_grid, nullptr);

                std::vector<motion_planning::Node*> nodes = motion_planning::rrt(cv_map, start.get(), goal.get(), num_nodes, step_size, goal_threshold, bias_probability, radius_pixel);

                std::vector<motion_planning::Node*> goal_path;
                // motion_planning::plot_rrt(cv_map, start.get(), goal.get(), false, nodes);
                if (motion_planning::distance(*nodes.back(), *goal) < goal_threshold) {
                    goal_path = motion_planning::trace_goal_path(nodes[nodes.size() - 2]);
                    this->nav_msgs_points = discretizePath(goal_path, 10);

                    // motion_planning::plot_rrt(cv_map, start.get(), goal.get(), true, goal_path);
                    rrtsuccess = true;
                } else {
                    ROS_ERROR("No path found!");
                    continue;
                }

                // for (auto const& node : nodes) {
                //     delete node;
                // }

                // for (auto node : goal_path) {
                //     geometry_msgs::PoseStamped point;
                //     point.pose.position.x = node->position[0] * pruned_map.info.resolution + pruned_map.info.origin.position.x;
                //     point.pose.position.y = (pruned_map.info.height - node->position[1]) * pruned_map.info.resolution + pruned_map.info.origin.position.y;
                //     point.pose.position.z = 0;
                //     nav_msgs_points.push_back(point);
                // }
                // geometry_msgs::PoseStamped end;
                // end.pose.position.x = goal_position[0];
                // end.pose.position.y = goal_position[1];
                // end.pose.position.z = 0;
                // nav_msgs_points.push_back(end);
                nav_msgs_path.poses = nav_msgs_points;
            }

            // std::vector<motion_planning::Node*> nodes = motion_planning::rrt(cv_map, start.get(), goal.get(), num_nodes, step_size, goal_threshold, bias_probability);
            // // motion_planning::RRT rrt(cv_map, start.get(), goal.get(), num_nodes, step_size, goal_threshold, bias_probability);
            // // std::vector<motion_planning::Node*> nodes = rrt.run(); 
            // // rrt.plot(nodes, start.get(), goal.get(), true);
            // rrtsuccess = true;
            // }
            // std::vector<motion_planning::Node*> nodes = rrt.run();

            // if (nodes.empty()) {
            //     ROS_ERROR("No path found!");
            //     continue;
            // }
            
            // // rrt.plot(nodes, start.get(), goal.get(), true);
            // Plot start
            // {
            // std::lock_guard<std::mutex> lock(position_mutex);
            // ROS_INFO("Start: (%f, %f)", start->position[0], start->position[1]);
            // }

            ros::Time current_time = ros::Time::now();

            nav_msgs_path.header.stamp = current_time;
            nav_msgs_path.header.seq = 0;
            nav_msgs_path.header.frame_id = map.header.frame_id;
            nav_msgs_path_pub.publish(nav_msgs_path);
            rate->sleep();
        }
    }

    nav_msgs::OccupancyGrid pruneMap(const nav_msgs::OccupancyGrid& original, float xmin, float xmax, float ymin, float ymax) {
        nav_msgs::OccupancyGrid pruned;
        pruned.info.resolution = original.info.resolution;
        pruned.info.origin = original.info.origin;

        int min_x = std::floor((xmin - original.info.origin.position.x) / original.info.resolution);
        int max_x = std::ceil((xmax - original.info.origin.position.x) / original.info.resolution);
        int min_y = std::floor((ymin - original.info.origin.position.y) / original.info.resolution);
        int max_y = std::ceil((ymax - original.info.origin.position.y) / original.info.resolution);

        int width = max_x - min_x + 1;
        int height = max_y - min_y + 1;

        pruned.info.width = width;
        pruned.info.height = height;
        pruned.data.resize(width * height);

        for (int y = min_y; y <= max_y; y++) {
            for (int x = min_x; x <= max_x; x++) {
                int old_idx = y * original.info.width + x;
                int new_idx = (y - min_y) * width + (x - min_x);
                if (x >= 0 && y >= 0 && x < original.info.width && y < original.info.height) {
                    pruned.data[new_idx] = original.data[old_idx];
                } else {
                    pruned.data[new_idx] = -1; 
                }
            }
        }

        pruned.info.origin.position.x += min_x * pruned.info.resolution;
        pruned.info.origin.position.y += min_y * pruned.info.resolution;

        return pruned;
    }

    cv::Mat occupancyGridToCvMat(const nav_msgs::OccupancyGrid& grid) {
        // Create a cv::Mat with the same dimensions as the occupancy grid, but flipped dimensions
        cv::Mat mat(grid.info.width, grid.info.height, CV_8UC1);

        // Fill the cv::Mat with data from the occupancy grid, correcting for coordinate axes
        for (unsigned int y = 0; y < grid.info.height; y++) {
            for (unsigned int x = 0; x < grid.info.width; x++) {
                int i = x + y * grid.info.width; // Original index in the occupancy grid data array
                if (grid.data[i] == -1) {
                    mat.at<uchar>(x, grid.info.height - 1 - y) = 127;
                } else if (grid.data[i] == 0){
                    mat.at<uchar>(x, grid.info.height - 1 - y) = 255; 
                } else if (grid.data[i] == 100){
                    mat.at<uchar>(x, grid.info.height - 1 - y) = 0;}

            }
        }

        // Transpose and flip to correct orientation
        cv::Mat rotated;
        cv::rotate(mat, rotated, cv::ROTATE_90_CLOCKWISE);
        cv::flip(rotated, rotated, 1);

        return rotated;
    }

    void check_pruning_map_subscribers() {
        int current_count = pruned_map_pub.getNumSubscribers();
        if (current_count > last_map_pruning_subscriber_count) {
            ROS_INFO("New subscriber detected on pruned_map topic!");
            last_map_pruning_subscriber_count = current_count;
            pruned_map_pub.publish(pruned_map);
        }
    }

    std::vector<std::vector<double>> linspace(const std::vector<double>& start, const std::vector<double>& end, int num_points) {
        std::vector<std::vector<double>> result;
        if (num_points < 2) {
            result.push_back(start);
            return result;
        }

        double step_x = (end[0] - start[0]) / (num_points - 1);
        double step_y = (end[1] - start[1]) / (num_points - 1);

        for (int i = 0; i < num_points; ++i) {
            std::vector<double> point = {start[0] + step_x * i, start[1] + step_y * i};
            result.push_back(point);
        }

        return result;
    }

    std::vector<geometry_msgs::PoseStamped> discretizePath(const std::vector<motion_planning::Node*>& rrt_nodes, int points_per_segment) {
        std::vector<geometry_msgs::PoseStamped> discretized_path;

        if (rrt_nodes.empty()) {
            return discretized_path;    
        }

        for (size_t i = 0; i < rrt_nodes.size() - 1; ++i) {
            auto start_node = rrt_nodes[i];
            auto end_node = rrt_nodes[i + 1];

            // Get positions from nodes
            std::vector<double> start_pos = {start_node->position[0], start_node->position[1]};
            std::vector<double> end_pos = {end_node->position[0], end_node->position[1]};

            // Generate intermediate points using linspace
            std::vector<std::vector<double>> interpolated_points = linspace(start_pos, end_pos, points_per_segment);

            for (const auto& point : interpolated_points) {
                geometry_msgs::PoseStamped pose_stamped;
                pose_stamped.pose.position.x = point[0] * pruned_map.info.resolution + pruned_map.info.origin.position.x;
                pose_stamped.pose.position.y = (pruned_map.info.height - point[1]) * pruned_map.info.resolution + pruned_map.info.origin.position.y;
                pose_stamped.pose.position.z = 0;
                pose_stamped.pose.orientation.w = 1.0;
                pose_stamped.header.frame_id = map.header.frame_id;
                pose_stamped.header.stamp = ros::Time::now();

                discretized_path.push_back(pose_stamped);
            }
        }

        // Add the last node to ensure the path reaches the goal
        auto last_node = rrt_nodes.back();
        geometry_msgs::PoseStamped goal_pose;
        goal_pose.pose.position.x = last_node->position[0] * pruned_map.info.resolution + pruned_map.info.origin.position.x;
        goal_pose.pose.position.y = (pruned_map.info.height - last_node->position[1]) * pruned_map.info.resolution + pruned_map.info.origin.position.y;
        goal_pose.pose.position.z = 0;
        discretized_path.push_back(goal_pose);

        return discretized_path;
    }

private:
    ros::Subscriber map_subscriber;
    ros::Subscriber pose_subscriber;
    ros::Publisher pub;
    ros::Publisher pruned_map_pub;
    nav_msgs::OccupancyGrid map;
    cv::Mat cv_map;
    nav_msgs::OccupancyGrid pruned_map;
    std::vector<double> start_position;
    std::vector<double>  goal_position;
    std::unique_ptr<ros::Rate> rate;
    int last_map_pruning_subscriber_count;
    std::mutex position_mutex; 
    std::mutex map_mutex;
    int num_nodes;
    double step_size;
    double goal_threshold;
    double bias_probability;
    bool rrtsuccess;
    int radius_pixel;
    float radius_meter;
    std::string path_topic;

    std::string robot_frame;

    tf2_ros::Buffer tf_buffer;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener;

    // test
    ros::Publisher nav_msgs_path_pub;
    std::vector<geometry_msgs::PoseStamped> nav_msgs_points;
    nav_msgs::Path nav_msgs_path;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "route_publisher");
    ROS_INFO("Route Publisher Node Started!");
    RoutePublisher route;

    // Initialize the spinner with 4 threads 
    ros::AsyncSpinner spinner(4); 
    spinner.start();
    route.rrt_route();
    ros::waitForShutdown(); 

    return 0;
}
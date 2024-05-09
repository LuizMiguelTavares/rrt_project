#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <opencv2/core.hpp>

#include "rrt_simple_ptr.hpp"


class MapPathSubscriber {
public:
    MapPathSubscriber() : tf_listener_(tf_buffer_), quad_tree_initialized_(false), is_rrt_completed_(false) {
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");

        this->getParamOrThrow(private_nh, "local_map_topic", local_map_topic_);
        this->getParamOrThrow(private_nh, "path_topic", path_topic_);
        this->getParamOrThrow(private_nh, "rate", rate_);
        this->getParamOrThrow(private_nh, "num_nodes", num_nodes_);
        this->getParamOrThrow(private_nh, "step_size", step_size_);
        this->getParamOrThrow(private_nh, "goal_threshold", goal_threshold_);
        this->getParamOrThrow(private_nh, "bias_probability", bias_probability_);
        this->getParamOrThrow(private_nh, "radius", radius_);

        local_map_sub_ = nh.subscribe(local_map_topic_, 1, &MapPathSubscriber::mapCallback, this);
        path_sub_ = nh.subscribe(path_topic_, 1, &MapPathSubscriber::pathCallback, this);

        last_point_pub_ = nh.advertise<geometry_msgs::PointStamped>("last_point_inside_map", 10);
        path_pub_ = nh.advertise<nav_msgs::Path>("local_path", 10);
        static_map_client_ = nh.serviceClient<nav_msgs::GetMap>("/static_map");
    }

    bool fetchStaticMap() {
        nav_msgs::GetMap srv;
        if (static_map_client_.call(srv)) {
            global_map_ = srv.response.map;
            map_frame_id_ = srv.response.map.header.frame_id;
            ROS_INFO("Successfully called /static_map service");
            return true;
        } else {
            ROS_ERROR("Failed to call /static_map service");
            return false;
        }
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        if (msg->data.empty()) {
            ROS_WARN("Received empty occupancy grid map.");
            return;
        }
        
        ROS_ERROR("Received occupancy grid map.");
        if (map_.data.empty()){
            ROS_INFO("Received occupancy grid map.");

            radius_pixel_ = std::round(radius_ / msg->info.resolution);
            ROS_INFO("Radius in pixels: %d", radius_pixel_);
        }
        map_ = *msg;
        map_frame_id_ = msg->header.frame_id;
    }

    void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
        if (msg->poses.empty()) {
            ROS_WARN("Received empty path.");
            return;
        }

        if (path_.poses.empty()){
            ROS_INFO("Received path.");
        }

        path_ = *msg;
        path_number_ = msg->header.seq;
    }

    void run() {
        ros::Rate rate(rate_);
        while (ros::ok()) {
            ros::spinOnce();
            if (!map_.data.empty() && !path_.poses.empty()) {
                if (!quad_tree_initialized_) {
                    if (!fetchStaticMap() || global_map_.data.empty()) {
                        ROS_WARN("Global map is not yet available.");
                        ros::Duration(1.0).sleep();
                        continue;
                    }

                    QTree::Rectangle boundary((global_map_.info.width*global_map_.info.resolution)/2, (global_map_.info.height*global_map_.info.resolution)/2, global_map_.info.width*global_map_.info.resolution, global_map_.info.height*global_map_.info.resolution);
                    if (global_map_.header.frame_id == path_.header.frame_id) {
                        int index = 0;
                        // ROS_ERROR("position: %f, %f", global_map_.info.origin.position.x, global_map_.info.origin.position.y);
                        // ROS_ERROR("width: %d, height: %d", global_map_.info.width, global_map_.info.height);
                        // ROS_ERROR("resolution: %f", global_map_.info.resolution);

                        for (const auto& pose : path_.poses) {
                            // Pose on grig map
                            // ROS_ERROR("Pose: %f, %f", pose.pose.position.x, pose.pose.position.y);
                            double x = pose.pose.position.x - global_map_.info.origin.position.x;
                            double y = pose.pose.position.y - global_map_.info.origin.position.y;

                            // ROS_ERROR("Pose: %f, %f", x, y);
                            std::shared_ptr<motion_planning::Node> node = std::make_shared<motion_planning::Node>(x, y, nullptr);
                            node->index = index;
                            if (!quad_tree_initialized_){
                                ROS_ERROR("Inserting node");
                                quad_tree_ = std::make_shared<QTree::QuadTree<motion_planning::Node>>(boundary);
                                quad_tree_->insert(node);
                                ROS_ERROR("Quad tree initialized"); 
                                quad_tree_initialized_ = true;
                            } else {
                                quad_tree_->insert(node);
                            }
                            index++;
                        }
                    } else {
                        ROS_ERROR("Global map and path frame ids do not match. Shutting down the node.");
                        ros::shutdown();
                    }
                }
                generateLocalRRT();
            }
            rate.sleep();
        }
    }

private:   
    template<typename T>
    void getParamOrThrow(ros::NodeHandle& nh, const std::string& param_name, T& param_value) {
        std::string node_name = ros::this_node::getName();
        if (!nh.getParam(param_name, param_value)) {
            std::string error_msg = node_name + ": Required parameter '" + param_name + "' not set.";
            ROS_ERROR_STREAM(error_msg);
            throw std::runtime_error(error_msg);
        }
    }

    bool transformPoseToMapFrame(const geometry_msgs::PoseStamped& input_pose, geometry_msgs::PoseStamped& output_pose) {
        if (input_pose.header.frame_id == map_frame_id_) {
            output_pose = input_pose;
            return true;
        }

        geometry_msgs::TransformStamped transform;
        try {
            transform = tf_buffer_.lookupTransform(map_frame_id_, input_pose.header.frame_id, ros::Time(0), ros::Duration(3.0));
            tf2::doTransform(input_pose, output_pose, transform);
            return true;
        } catch (tf2::TransformException &ex) {
            ROS_ERROR("TF2 transform exception: %s", ex.what());
            return false;
        }
    }

    bool isInMap(const geometry_msgs::Point& point) {
        if (map_.data.empty()){
            ROS_WARN("Occupancy grid map is not yet available.");
            return false;
        }

        double x_min = map_.info.origin.position.x;
        double y_min = map_.info.origin.position.y;
        double x_max = x_min + map_.info.width * map_.info.resolution;
        double y_max = y_min + map_.info.height * map_.info.resolution;

        return point.x >= x_min && point.x <= x_max && point.y >= y_min && point.y <= y_max;
    }

    void updatePositionFromTF(const std::string& global_map_frame, const std::string& local_map_frame) {
        geometry_msgs::TransformStamped transform_stamped;
        try {
            transform_stamped = tf_buffer_.lookupTransform(global_map_frame, local_map_frame, ros::Time(0), ros::Duration(1.0));
            robot_position = {transform_stamped.transform.translation.x, transform_stamped.transform.translation.y};
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }
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
        // cv::flip(rotated, rotated, 1);

        //New debug
        cv::flip(rotated, rotated, 0);

        Debug_image = rotated.clone();
        return rotated;
    }

    void generateLocalRRT() {
        updatePositionFromTF(global_map_.header.frame_id, map_frame_id_);

        float robot_x = robot_position[0] - global_map_.info.origin.position.x;
        float robot_y = robot_position[1] - global_map_.info.origin.position.y;
        std::shared_ptr<motion_planning::Node> robot_position = std::make_shared<motion_planning::Node>(robot_x, robot_y, nullptr);

        std::shared_ptr<motion_planning::Node> closest_path_point = quad_tree_->nearest_neighbor(robot_position);

        geometry_msgs::PointStamped last_point_inside_map;
        geometry_msgs::PointStamped last_point_inside_map_local_frame;
        bool last_point_valid = false;

        cv::Mat local_map = occupancyGridToCvMat(map_);
        bool obstacle_encountered = false;

        geometry_msgs::PoseStamped last_local_pose;
        bool last_local_pose_valid = false;

        for (int i = closest_path_point->index; i < path_.poses.size(); i++) {
            geometry_msgs::PoseStamped pose;
            pose.header = path_.header;
            pose.pose = path_.poses[i].pose;

            geometry_msgs::PoseStamped pose_map_frame;
            if (transformPoseToMapFrame(pose, pose_map_frame)) {
                if (isInMap(pose_map_frame.pose.position)) {
                    if (!last_local_pose_valid) {
                        last_local_pose = pose_map_frame;
                        last_local_pose_valid = true;
                    } else {
                        if (!obstacle_encountered) {
                            // Check for obstacle intersection
                            // Convert the points to local map coordinates (pixels)
                            int x_old = static_cast<int>((last_local_pose.pose.position.x - map_.info.origin.position.x) / map_.info.resolution);
                            int y_old = static_cast<int>((last_local_pose.pose.position.y - map_.info.origin.position.y) / map_.info.resolution);
                            int x_new = static_cast<int>((pose_map_frame.pose.position.x - map_.info.origin.position.x) / map_.info.resolution);
                            int y_new = static_cast<int>((pose_map_frame.pose.position.y - map_.info.origin.position.y) / map_.info.resolution);

                            y_old = map_.info.height - y_old;
                            y_new = map_.info.height - y_new;
                            // ROS_ERROR("x: %d, y: %d", x_new, y_new);

                            // Plot x_old on the Debug_image
                            // cv::circle(Debug_image, cv::Point(x_old, y_old), 0.5, cv::Scalar(0), -1, CV_8UC1);
                            obstacle_encountered = motion_planning::check_obstacle_intersection(local_map, x_old, y_old, x_new, y_new, radius_);
                            last_local_pose = pose_map_frame;
                        }
                    }
                    last_point_inside_map.point = pose.pose.position;
                    last_point_inside_map_local_frame.point = pose_map_frame.pose.position;
                    last_point_valid = true;
                } else {
                    break;
                }
            }
        }
        
        // cv::Mat resized;
        // cv::resize(Debug_image, resized, cv::Size(300, 300));
        // cv::imshow("Local Map", resized);
        // cv::waitKey(1);

        if (!last_point_valid) {
            ROS_ERROR("%s: No global path point inside the local map.", ros::this_node::getName().c_str());
            return;
        }

        if (obstacle_encountered) {
            ROS_ERROR("%s: Obstacle encountered in the local map.", ros::this_node::getName().c_str());
            return;
        }

        last_point_inside_map.header.stamp = ros::Time::now();
        last_point_inside_map.header.frame_id = path_.header.frame_id;
        last_point_pub_.publish(last_point_inside_map);

        if (!is_rrt_completed_){
            // Convert last known valid pose to node
            double x_start = std::round(map_.info.width/2);
            double y_start = std::round(map_.info.height/2);

            double x_goal = std::round((last_point_inside_map_local_frame.point.x - map_.info.origin.position.x) / map_.info.resolution);
            double y_goal = std::round((last_point_inside_map_local_frame.point.y - map_.info.origin.position.y) / map_.info.resolution);
            y_goal = map_.info.height - y_goal;

            ROS_ERROR("x_start: %f, y_start: %f", x_start, y_start);
            ROS_ERROR("x_goal: %f, y_goal: %f", x_goal, y_goal);

            ROS_ERROR("map width: %d, map height: %d", map_.info.width, map_.info.height);
            
            if (x_goal == map_.info.width) x_goal = map_.info.width - 1;
            if (y_goal == map_.info.height) y_goal = map_.info.height - 1;

            if (x_goal < 0 || x_goal >= map_.info.width || y_goal < 0 || y_goal >= map_.info.height) {
                ROS_ERROR("%s: Goal point outside the local map.", ros::this_node::getName().c_str());
                return;
            }

            std::shared_ptr<motion_planning::Node> start_node = std::make_shared<motion_planning::Node>(x_start, y_start, nullptr);
            std::shared_ptr<motion_planning::Node> goal_node = std::make_shared<motion_planning::Node>(x_goal, y_goal, nullptr);
            // ROS_ERROR("Cheguei no RRT");
            std::vector<std::shared_ptr<motion_planning::Node>> nodes;
            nodes = run_rrt(start_node, goal_node, num_nodes_, step_size_, goal_threshold_, bias_probability_, radius_pixel_);
            // ROS_ERROR("Sai do RRT");
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
            }

            // Convert the path from nodes to a ROS path message
            nav_msgs::Path ros_path = convertNodesToPath(goal_path, map_);
            path_pub_.publish(ros_path);

            // is_rrt_completed_ = true;
        }
    }

    std::vector<std::shared_ptr<motion_planning::Node>> run_rrt(std::shared_ptr<motion_planning::Node> start, std::shared_ptr<motion_planning::Node> end, int num_nodes, double step_size, double goal_threshold, double bias_probability, int radius_pixel){
        cv::Mat local_map = occupancyGridToCvMat(map_);
        if (local_map.empty()) {
            ROS_ERROR("Local map is empty");
            return {};
        }

        return motion_planning::rrt(local_map, start, end, num_nodes, step_size, goal_threshold, bias_probability, radius_pixel);
    }

    nav_msgs::Path convertNodesToPath(const std::vector<std::shared_ptr<motion_planning::Node>> nodes, const nav_msgs::OccupancyGrid map) {
        nav_msgs::Path path;
        path.header.frame_id = map.header.frame_id;
        path.header.stamp = ros::Time::now();

        for (auto node : nodes) {
            if (node == nullptr) continue; // Skip null nodes

            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = map.header.frame_id;
            pose_stamped.header.stamp = ros::Time::now();

            pose_stamped.pose.position.x = node->x*map.info.resolution + map.info.origin.position.x;

            pose_stamped.pose.position.y = (map.info.height - node->y)*map.info.resolution + map.info.origin.position.y;
            pose_stamped.pose.position.z = 0;

            pose_stamped.pose.orientation.x = 0.0;
            pose_stamped.pose.orientation.y = 0.0;
            pose_stamped.pose.orientation.z = 0.0;
            pose_stamped.pose.orientation.w = 1.0;

            path.poses.push_back(pose_stamped);
        }

        return path;
    }

    cv::Mat Debug_image;
    int rate_;
    std::string local_map_topic_;
    std::string path_topic_;
    int path_number_;
    std::string map_frame_id_;
    int num_nodes_;
    double step_size_;
    double goal_threshold_;
    double bias_probability_;
    double radius_;
    int radius_pixel_;
    std::vector<double> robot_position;
    bool is_rrt_completed_;

    ros::Subscriber local_map_sub_;
    ros::Subscriber path_sub_;
    ros::Publisher last_point_pub_;
    ros::Publisher path_pub_;
    ros::ServiceClient static_map_client_;

    nav_msgs::OccupancyGrid map_;
    nav_msgs::OccupancyGrid global_map_;
    nav_msgs::Path path_;
    std::shared_ptr<QTree::QuadTree<motion_planning::Node>> quad_tree_;
    bool quad_tree_initialized_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_path_subscriber");
    MapPathSubscriber mps;
    mps.run();
    return 0;
}
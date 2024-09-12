#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/SetMap.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <opencv2/core.hpp>
#include <cmath>
#include <mutex>
#include <sensor_msgs/LaserScan.h>
#include "rrt.hpp"
#include <utility>

class MapPathSubscriber {
public:
    MapPathSubscriber() : tf_listener_(tf_buffer_), quad_tree_initialized_(false), is_rrt_completed_(false), Log_time(tic()), log_time(2.0){
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");

        this->getParamOrThrow(private_nh, "local_map_topic", local_map_topic_);
        this->getParamOrThrow(private_nh, "robot_frame", robot_frame_);
        this->getParamOrThrow(private_nh, "path_topic", path_topic_);
        this->getParamOrThrow(private_nh, "rate", rate_);
        this->getParamOrThrow(private_nh, "num_nodes", num_nodes_);
        this->getParamOrThrow(private_nh, "step_size", step_size_);
        this->getParamOrThrow(private_nh, "goal_threshold", goal_threshold_);
        this->getParamOrThrow(private_nh, "bias_probability", bias_probability_);
        this->getParamOrThrow(private_nh, "control_point_offset", control_point_offset_);
        this->getParamOrThrow(private_nh, "radius", radius_);

        path_sub_ = nh.subscribe(path_topic_, 1, &MapPathSubscriber::pathCallback, this);
        local_map_sub_ = nh.subscribe(local_map_topic_, 1, &MapPathSubscriber::mapCallback, this);

        last_point_pub_ = nh.advertise<geometry_msgs::PointStamped>("last_point_inside_map", 10);
        path_pub_ = nh.advertise<nav_msgs::Path>("local_path", 10);
        smoothed_path_pub_ = nh.advertise<nav_msgs::Path>("smoothed_local_path", 10);
        // traveled_path_pub_ = nh.advertise<nav_msgs::Path>("traveled_path", 10);

        ros::service::waitForService("/updated_map");
        static_map_client_ = nh.serviceClient<nav_msgs::GetMap>("/updated_map");
        fetchStaticMap();
        
        ros::service::waitForService("/update_map");
        update_map_client = nh.serviceClient<nav_msgs::SetMap>("/update_map");

        ros::service::waitForService("/merged_map");
        merged_map_client = nh.serviceClient<nav_msgs::GetMap>("/merged_map");

        private_nh.param<std::string>("laser_scan_topic", laser_scan_topic_, "/laser/scan");
        laser_scan_sub_ = nh.subscribe(laser_scan_topic_, 1, &MapPathSubscriber::laserScanCallback, this);

        laser_point_pub_ = nh.advertise<geometry_msgs::PointStamped>("laser_point", 10);
        merged_map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/merged_map", 10);
    }

    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        latest_laser_scan_ = msg;
    }

    bool fetchStaticMap() {
        nav_msgs::GetMap srv;
        if (static_map_client_.call(srv)) {
            global_map_ = srv.response.map;
            ROS_INFO("LocalMap: Successfully called /static_map service");
            return true;
        } else {
            ROS_ERROR("LocalMap: Failed to call /static_map service");
            return false;
        }
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(local_map_mutex);
        if (msg->data.empty()) {
            ROS_WARN("Received empty occupancy grid map.");
            return;
        }
        if (map_.data.empty()){
            ROS_INFO("Received occupancy grid map.");

            radius_pixel_ = std::round(radius_ / msg->info.resolution);
            ROS_INFO("Radius in pixels: %d", radius_pixel_);
        }
        map_ = *msg;
        map_frame_id_ = msg->header.frame_id;
    }

    bool updateMap() {
        nav_msgs::SetMap srv;
        srv.request.map = this->map_;
        srv.request.initial_pose = geometry_msgs::PoseWithCovarianceStamped();
        if (update_map_client.call(srv)) {
            ROS_INFO("LocalMap: Successfully updated map.");
            return true;
        } else {
            ROS_ERROR("LocalMap: Failed to call /update_map service");
            return false;
        }
    }

    bool mergeMaps() {
        nav_msgs::GetMap srv;
        if (merged_map_client.call(srv)) {
            merged_map_ = srv.response.map;
            // ROS_INFO("Successfully called /merged_map service");
            return true;
        } else {
            ROS_ERROR("Failed to call /merged_map service");
            return false;
        }
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
                        for (const auto& pose : path_.poses) {
                            double x = pose.pose.position.x - global_map_.info.origin.position.x;
                            double y = pose.pose.position.y - global_map_.info.origin.position.y;

                            std::shared_ptr<motion_planning::Node> node = std::make_shared<motion_planning::Node>(x, y, nullptr);
                            node->index = index;
                            if (!quad_tree_initialized_){
                                quad_tree_ = std::make_shared<QTree::QuadTree<motion_planning::Node>>(boundary);
                                quad_tree_->insert(node);
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
                // updateTraveledPath();
            } else {
                if (toc(Log_time) > log_time){
                    ROS_WARN("Occupancy grid map or path is not yet available.");
                    Log_time = tic();
                }    
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
        std::lock_guard<std::mutex> lock(local_map_mutex);
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
            transform_stamped = tf_buffer_.lookupTransform(global_map_frame, local_map_frame, ros::Time(0));
            robot_position = {transform_stamped.transform.translation.x, transform_stamped.transform.translation.y};
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }
    }

    cv::Mat occupancyGridToCvMat(const nav_msgs::OccupancyGrid& grid) {
        std::lock_guard<std::mutex> lock(local_map_mutex);
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

    void generateLocalRRT() {
        updatePositionFromTF(global_map_.header.frame_id, robot_frame_);

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
                    }

                    last_point_inside_map.point = pose.pose.position;
                    last_point_inside_map_local_frame.point = pose_map_frame.pose.position;
                    last_point_valid = true;
                } else {
                    break;
                }
            }
        }

        if (!last_point_valid) { // Generate a new global path
            ROS_ERROR("%s: No global path point inside the local map.", ros::this_node::getName().c_str());
            last_point_inside_map.header.stamp = ros::Time::now();
            last_point_inside_map.header.frame_id = path_.header.frame_id;
            last_point_pub_.publish(last_point_inside_map);
            return;
        }

        last_point_inside_map.header.stamp = ros::Time::now();
        last_point_inside_map.header.frame_id = path_.header.frame_id;
        last_point_pub_.publish(last_point_inside_map);

        // local RRT
        double x_start = std::round(map_.info.width/2 + control_point_offset_ / map_.info.resolution);
        double y_start = std::round(map_.info.height/2);

        double x_goal = std::round((last_point_inside_map_local_frame.point.x - map_.info.origin.position.x) / map_.info.resolution);
        double y_goal = std::round((last_point_inside_map_local_frame.point.y - map_.info.origin.position.y) / map_.info.resolution);

        y_goal = map_.info.height - y_goal;
        
        if (x_goal == map_.info.width) x_goal = map_.info.width - 1;
        if (y_goal == map_.info.height) y_goal = map_.info.height - 1;

        if (x_goal < 0 || x_goal >= map_.info.width || y_goal < 0 || y_goal >= map_.info.height) {
            ROS_ERROR("%s: Goal point outside the local map.", ros::this_node::getName().c_str());
            return;
        }

        // See if there is an obstacle between the robot and the goal to update global map
        // Calculate the angle from the robot to the goal
        double angle_to_goal = calculateAngleToGoal(last_point_inside_map_local_frame.point);

        // Find the closest laser index corresponding to this angle
        int closest_laser_index = findClosestLaserIndex(angle_to_goal);

        bool obstacle_detected = laserCollidesWithObstacle(closest_laser_index, last_point_inside_map_local_frame);
        if (obstacle_detected) {
            // ROS_WARN("Obstacle detected between robot and goal. Updating global map.");
            // Update the global map
            
            publishLaserPoint(closest_laser_index);
        } else {
            geometry_msgs::PoseStamped laser_point;
            laser_point_pub_.publish(laser_point);
        }

        std::shared_ptr<motion_planning::Node> start_node = std::make_shared<motion_planning::Node>(x_start, y_start, nullptr);
        std::shared_ptr<motion_planning::Node> goal_node = std::make_shared<motion_planning::Node>(x_goal, y_goal, nullptr);
        std::vector<std::shared_ptr<motion_planning::Node>> nodes;
        nodes = run_rrt(start_node, goal_node, num_nodes_, step_size_, goal_threshold_, bias_probability_, radius_pixel_);

        if (nodes.empty()) {
            ROS_ERROR("No path found from RRT within the local map.");
            return;
        }

        std::vector<std::shared_ptr<motion_planning::Node>> goal_path;
        if (nodes.back()->x != goal_node->x || nodes.back()->y != goal_node->y) {
            ROS_ERROR("No local path found!");
            if (obstacle_detected){
                if (!fetchStaticMap() || global_map_.data.empty()) {
                    ROS_WARN("Global map is not yet available.");
                    return;
                }

                // Update the global map
                updateMap();
                fetchStaticMap();
            }
            return;
        } else {
            goal_path = motion_planning::trace_goal_path(nodes.back());
        }

        // Convert the path from nodes to a ROS path message
        nav_msgs::Path ros_path = convertNodesToPath(goal_path, map_);
        nav_msgs::Path smoothed_path = smoothPath(ros_path, 20);

        smoothed_path_pub_.publish(smoothed_path);
        path_pub_.publish(ros_path);
    }

    std::vector<std::shared_ptr<motion_planning::Node>> run_rrt(std::shared_ptr<motion_planning::Node> start, std::shared_ptr<motion_planning::Node> end, int num_nodes, double step_size, double goal_threshold, double bias_probability, int radius_pixel){
        mergeMaps();
        merged_map_pub_.publish(merged_map_);
        cv::Mat local_map = occupancyGridToCvMat(merged_map_);
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
            if (node == nullptr) continue; 
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

    int findClosestLaserIndex(double angle) {
        if (!latest_laser_scan_) {
            ROS_WARN("Laser scan data not yet available.");
            return -1;
        }

        double laser_min_angle = latest_laser_scan_->angle_min;
        double laser_max_angle = latest_laser_scan_->angle_max;
        int num_laser_points = latest_laser_scan_->ranges.size();
        double laser_angle_increment = latest_laser_scan_->angle_increment;

        // Normalize the angle to be within [laser_min_angle, laser_max_angle]
        while (angle < laser_min_angle) {
            angle += 2 * M_PI;
        }
        while (angle > laser_max_angle) {
            angle -= 2 * M_PI;
        }

        // Ensure the angle is within the bounds of the laser scan angles
        if (angle < laser_min_angle) {
            angle = laser_min_angle;
        } else if (angle > laser_max_angle) {
            angle = laser_max_angle;
        }

        int closest_index = std::round((angle - laser_min_angle) / laser_angle_increment);

        // Ensure the index is within valid bounds
        if (closest_index < 0) {
            closest_index = 0;
        } else if (closest_index >= num_laser_points) {
            closest_index = num_laser_points - 1;
        }

        return closest_index;
    }

    void publishLaserPoint(int index) {
        if (!latest_laser_scan_ || index < 0 || index >= latest_laser_scan_->ranges.size()) {
            ROS_WARN("Invalid laser scan data or index.");
            return;
        }

        geometry_msgs::PointStamped laser_point;
        laser_point.header.frame_id = latest_laser_scan_->header.frame_id;
        laser_point.header.stamp = ros::Time::now();

        double range = latest_laser_scan_->ranges[index];
        if (std::isinf(range) || std::isnan(range)) {
            // ROS_WARN("Invalid range value at index %d: %f", index, range);
            return;
        }

        double angle = latest_laser_scan_->angle_min + index * latest_laser_scan_->angle_increment;

        laser_point.point.x = range * cos(angle);
        laser_point.point.y = range * sin(angle);
        laser_point.point.z = 0.0;

        ROS_INFO("Laser point: (%f, %f)", laser_point.point.x, laser_point.point.y);

        laser_point_pub_.publish(laser_point);
    }

    double calculateAngleToGoal(const geometry_msgs::Point& goal_point) {
        // Assuming the goal_point is already in the laser frame
        double angle_to_goal = std::atan2(goal_point.y, goal_point.x);
        return angle_to_goal;
    }

    bool laserCollidesWithObstacle(int index, const geometry_msgs::PointStamped last_point_inside_map_local_frame) {
        if (!latest_laser_scan_ || index < 0 || index >= latest_laser_scan_->ranges.size()) {
            ROS_WARN("Invalid laser scan data or index.");
            return false;
        }

        double range = latest_laser_scan_->ranges[index];
        if (std::isinf(range) || std::isnan(range)) {
            // ROS_WARN("Invalid range value at index %d: %f", index, range);
            return false;
        }

        double point_distance = std::sqrt(std::pow(last_point_inside_map_local_frame.point.x, 2) + std::pow(last_point_inside_map_local_frame.point.y, 2));

        if (range < point_distance) {
            return true;
        }

        return false;
    }

    // void updateTraveledPath() {
    //     geometry_msgs::PoseStamped current_pose;
    //     current_pose.header.frame_id = global_map_.header.frame_id;
    //     current_pose.header.stamp = ros::Time::now();
    //     current_pose.pose.position.x = robot_position[0];
    //     current_pose.pose.position.y = robot_position[1];
    //     current_pose.pose.position.z = 0;

    //     current_pose.pose.orientation.x = 0.0;
    //     current_pose.pose.orientation.y = 0.0;
    //     current_pose.pose.orientation.z = 0.0;
    //     current_pose.pose.orientation.w = 1.0;

    //     traveled_path_.poses.push_back(current_pose);
    //     traveled_path_.header.frame_id = global_map_.header.frame_id;
    //     traveled_path_.header.stamp = ros::Time::now();
    //     traveled_path_pub_.publish(traveled_path_);
    // }

    std::chrono::time_point<std::chrono::high_resolution_clock> tic() {
        return std::chrono::high_resolution_clock::now();
    }

    double toc(std::chrono::time_point<std::chrono::high_resolution_clock> start) {
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        return elapsed.count();
    }

    std::pair<double, double> bezierPoint(const std::vector<std::pair<double, double>>& controlPoints, double t) {
        int n = controlPoints.size() - 1;
        std::pair<double, double> p = {0.0, 0.0};

        for (int i = 0; i <= n; ++i) {
            double binomialCoefficient = std::tgamma(n + 1) / (std::tgamma(i + 1) * std::tgamma(n - i + 1));
            double term = binomialCoefficient * std::pow(1 - t, n - i) * std::pow(t, i);

            p.first += term * controlPoints[i].first;
            p.second += term * controlPoints[i].second;
        }

        return p;
    }

    // Function to generate a smoothed path using an n-point Bézier curve
    nav_msgs::Path smoothPath(const nav_msgs::Path& rrtPath, int numPoints, float smoothness = 0.01) {
        nav_msgs::Path smoothPath;
        smoothPath.header = rrtPath.header; 

        std::vector<std::pair<double, double>> controlPoints;
        std::vector<std::pair<double, double>> rrtPoints;

        // Extract points from the RRT path
        for (const auto& pose : rrtPath.poses) {
            rrtPoints.emplace_back(pose.pose.position.x, pose.pose.position.y);
        }

        int n = numPoints > 0 ? numPoints : rrtPoints.size();

        if (n > rrtPoints.size()) {
            n = rrtPoints.size();  // Ensure n does not exceed the size of the RRT path
        }

        if (n > rrtPoints.size() || rrtPoints.size() < 4) {
            ROS_ERROR("Not enough points to form a Bézier curve. Need at least 4 points.");
            return rrtPath;
        }

        int step = (rrtPoints.size() - 1) / (n - 1);

        controlPoints.push_back(rrtPoints.front());

        // Select equally spaced points from the RRT path, or use all points if numPoints is 0
        for (int i = 0; i < n; ++i) {
            if (i * step == 0){
                continue;
            } else if (i * step >= rrtPoints.size()){
                break;
            }
            controlPoints.push_back(rrtPoints[i * step]);
        }

        controlPoints.push_back(rrtPoints.back());

        // Generate points on the Bézier curve
        for (double t = 0.0; t <= 1.0; t += smoothness) {
            std::pair<double, double> pt = bezierPoint(controlPoints, t);

            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = pt.first;
            pose.pose.position.y = pt.second;
            pose.pose.position.z = 0.0;  // Assuming a 2D path, z remains 0
            smoothPath.poses.push_back(pose);
        }

        return smoothPath;
    }

    std::mutex local_map_mutex;
    int rate_;
    float log_time;
    std::chrono::time_point<std::chrono::high_resolution_clock> Log_time;
    std::string path_topic_, robot_frame_;
    int path_number_;
    std::string map_frame_id_;
    std::string local_map_topic_;
    int num_nodes_;
    double step_size_;
    double goal_threshold_;
    double bias_probability_, control_point_offset_;
    double radius_;
    int radius_pixel_;
    std::vector<double> robot_position;
    bool is_rrt_completed_;

    ros::Subscriber local_map_sub_;
    ros::Subscriber path_sub_;
    ros::Publisher last_point_pub_;
    ros::Publisher path_pub_;
    ros::Publisher smoothed_path_pub_;
    // ros::Publisher traveled_path_pub_;
    ros::ServiceClient static_map_client_;
    ros::ServiceClient update_map_client;
    ros::ServiceClient merged_map_client;

    nav_msgs::OccupancyGrid map_;
    nav_msgs::OccupancyGrid merged_map_;
    nav_msgs::OccupancyGrid global_map_;
    nav_msgs::Path path_;
    nav_msgs::Path traveled_path_;
    std::shared_ptr<QTree::QuadTree<motion_planning::Node>> quad_tree_;
    bool quad_tree_initialized_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    ros::Publisher laser_point_pub_;
    ros::Publisher merged_map_pub_;
    ros::Subscriber laser_scan_sub_;
    sensor_msgs::LaserScan::ConstPtr latest_laser_scan_;
    std::string laser_scan_topic_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_path_subscriber");
    MapPathSubscriber mps;
    mps.run();
    return 0;
}
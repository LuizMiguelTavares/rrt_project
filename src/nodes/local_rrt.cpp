#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/SetMap.h>
#include <sensor_msgs/PointCloud.h>
#include "rrt_project/ClusterObstacles.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point32.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <opencv2/core.hpp>
#include <cmath>
#include <limits>
#include <set>

#include "rrt_simple_ptr.hpp"


class MapPathSubscriber {
public:
    MapPathSubscriber() : tf_listener_(tf_buffer_), quad_tree_initialized_(false), is_rrt_completed_(false) {
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");

        this->getParamOrThrow(private_nh, "local_map_topic", local_map_topic_);
        private_nh.param<std::string>("local_points_topic", local_points_topic_, "local_points");
        this->getParamOrThrow(private_nh, "path_topic", path_topic_);
        this->getParamOrThrow(private_nh, "rate", rate_);
        this->getParamOrThrow(private_nh, "num_nodes", num_nodes_);
        this->getParamOrThrow(private_nh, "step_size", step_size_);
        this->getParamOrThrow(private_nh, "goal_threshold", goal_threshold_);
        this->getParamOrThrow(private_nh, "bias_probability", bias_probability_);
        this->getParamOrThrow(private_nh, "radius", radius_);

        local_map_sub_ = nh.subscribe(local_map_topic_, 1, &MapPathSubscriber::mapCallback, this);
        local_points_sub_ = nh.subscribe(local_points_topic_, 1, &MapPathSubscriber::localPointsCallback, this);
        path_sub_ = nh.subscribe(path_topic_, 1, &MapPathSubscriber::pathCallback, this);

        local_points_pub_ = nh.advertise<sensor_msgs::PointCloud>("local_points_cluster", 10);
        last_point_pub_ = nh.advertise<geometry_msgs::PointStamped>("last_point_inside_map", 10);
        first_and_last_point_pub_ = nh.advertise<sensor_msgs::PointCloud>("first_and_last_cluster_points", 10);
        path_pub_ = nh.advertise<nav_msgs::Path>("local_path", 10);

        debug_local_points_pub_ = nh.advertise<sensor_msgs::PointCloud>("debug_local_points", 10);
        
        ros::service::waitForService("/updated_map");
        static_map_client_ = nh.serviceClient<nav_msgs::GetMap>("/updated_map");
        
        ros::service::waitForService("/update_map");
        update_map_client = nh.serviceClient<nav_msgs::SetMap>("/update_map");

        cluster_client_ = nh.serviceClient<rrt_project::ClusterObstacles>("/cluster_obstacles");
    }

    bool fetchStaticMap() {
        nav_msgs::GetMap srv;
        if (static_map_client_.call(srv)) {
            global_map_ = srv.response.map;
            ROS_INFO("Successfully called /static_map service");
            return true;
        } else {
            ROS_ERROR("Failed to call /static_map service");
            return false;
        }
    }

    std::vector<double> PointCloud2Vector(const sensor_msgs::PointCloud& cloud) {
        std::vector<double> points;
        for (const auto& point : cloud.points) {
            points.push_back(static_cast<double>(point.x));
            points.push_back(static_cast<double>(point.y));
        }
        return points;
    }

    sensor_msgs::PointCloud convertMapToPointCloud(const nav_msgs::OccupancyGrid& map) {
        sensor_msgs::PointCloud cloud;
        cloud.header = map.header;

        // Iterate through the map data
        for (int i = 0; i < map.info.width; ++i)
        {
            for (int j = 0; j < map.info.height; ++j)
            {
                int index = i + j * map.info.width;
                if (map.data[index] == 100)
                {
                    geometry_msgs::Point32 point;
                    point.x = i * map.info.resolution + map.info.origin.position.x;
                    point.y = j * map.info.resolution + map.info.origin.position.y;
                    point.z = 0;
                    cloud.points.push_back(point);
                }
            }
        }

        return cloud;
    }

    bool requestClusterization() {
        std::lock_guard<std::mutex> lock(local_map_mutex);
        rrt_project::ClusterObstacles srv;
        sensor_msgs::PointCloud local_points = convertMapToPointCloud(map_);
        debug_local_points_pub_.publish(local_points);

        srv.request.obstacle_points = PointCloud2Vector(local_points_);
        srv.request.max_obstacle_distance = radius_*2;

        if (cluster_client_.call(srv)) {
            ROS_INFO("Successfully called /cluster_map service");
            cluster_indices_ = srv.response.clusters;
            return true;
        } else {
            ROS_ERROR("Failed to call /cluster_map service");
            return false;
        }
    }

    bool updateMap() {
        nav_msgs::SetMap srv;
        srv.request.map = this->map_;
        srv.request.initial_pose = geometry_msgs::PoseWithCovarianceStamped();
        if (update_map_client.call(srv)) {
            ROS_INFO("Successfully updated map.");
            return true;
        } else {
            ROS_ERROR("Failed to call /update_map service");
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

    void localPointsCallback(const sensor_msgs::PointCloud::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(local_map_mutex);
        local_points_ = *msg;
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
            transform_stamped = tf_buffer_.lookupTransform(global_map_frame, local_map_frame, ros::Time(0), ros::Duration(1.0));
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

        //New debug
        // cv::flip(rotated, rotated, 0);

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
                            int x_old = static_cast<int>((last_local_pose.pose.position.x - map_.info.origin.position.x) / map_.info.resolution);
                            int y_old = static_cast<int>((last_local_pose.pose.position.y - map_.info.origin.position.y) / map_.info.resolution);
                            int x_new = static_cast<int>((pose_map_frame.pose.position.x - map_.info.origin.position.x) / map_.info.resolution);
                            int y_new = static_cast<int>((pose_map_frame.pose.position.y - map_.info.origin.position.y) / map_.info.resolution);
                            y_old = map_.info.height - y_old;
                            y_new = map_.info.height - y_new;
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

        if (!last_point_valid) {
            ROS_ERROR("%s: No global path point inside the local map.", ros::this_node::getName().c_str());
            last_point_inside_map.header.stamp = ros::Time::now();
            last_point_inside_map.header.frame_id = path_.header.frame_id;
            last_point_pub_.publish(last_point_inside_map);
            return;
        }

        if (obstacle_encountered) {
            // ROS_ERROR("%s: Obstacle encountered in the local map.", ros::this_node::getName().c_str());
            // ROS_INFO("Requesting clusterization...");

            bool clusterization_successful = false;
            try {
                clusterization_successful = requestClusterization();
                if (clusterization_successful) {
                    std::lock_guard<std::mutex> lock(local_map_mutex);
                    local_points_cluster_ = local_points_;
                    // ROS_ERROR("Plotting size of local_points_: %zu", local_points_.points.size());
                    if (local_points_cluster_.channels.empty()) {
                        local_points_cluster_.channels.resize(1);  
                    }
                    local_points_cluster_.channels[0].values = cluster_indices_;
                    local_points_cluster_.channels[0].name = "intensity";
                    ROS_INFO("Clusterization request successful.");
                } else {
                    ROS_ERROR("Failed to request clusterization.");
                }
            } catch (const std::exception& e) {
                ROS_ERROR("Exception caught: %s", e.what());
                return;
            }

            // Find cluster of the obstacle
            if (clusterization_successful){
                updateMap();
                fetchStaticMap();
                std::set<float> uniqueElements(cluster_indices_.begin(), cluster_indices_.end());

                int index_point = findClosestPointIndex(local_points_cluster_.points, last_local_pose.pose.position.x, last_local_pose.pose.position.y);
                local_points_cluster_.channels[0].values[index_point] = 0;
                
                int index_cluster_left;
                int index_cluster_right;
                geometry_msgs::Point32 first_point;
                geometry_msgs::Point32 last_point;

                if (uniqueElements.size() > 1) {
                    std::vector<float> aux_indices;
                    aux_indices.reserve(cluster_indices_.size());
                    aux_indices = cluster_indices_;

                    std::vector<geometry_msgs::Point32> aux_points;
                    aux_points.reserve(local_points_cluster_.points.size());
                    aux_points = local_points_cluster_.points;

                    while ((aux_indices[0] == aux_indices.back()) && (aux_indices[0] == cluster_indices_[index_point]))
                    {
                        float aux = aux_indices.back();
                        geometry_msgs::Point32 obs_aux = aux_points.back();

                        aux_indices.pop_back();
                        aux_points.pop_back();

                        aux_indices.insert(aux_indices.begin(), aux);
                        aux_points.insert(aux_points.begin(), obs_aux); 
                    }
                    
                    bool first_point_not_found = true;
                    for (int i = 0; i < aux_indices.size() - 1; i++){
                        if (aux_indices[i] == aux_indices[index_point]){
                            if (first_point_not_found) {
                                first_point_not_found = false;
                                index_cluster_left = i;
                                first_point = aux_points[i];
                                // ROS_ERROR("First point: %f, %f", first_point.x, first_point.y);
                            } 
                            if ((aux_indices[i + 1] != aux_indices[index_point]) || (i == aux_indices.size() - 2)) {
                                index_cluster_right = i;
                                last_point = aux_points[i];
                                // ROS_ERROR("Last point: %f, %f", last_point.x, last_point.y);
                                break;
                            }
                        }
                    }
                }

                if (uniqueElements.size() == 1){
                    float biggest_distance = 0;
                    for (int i = 0; i < cluster_indices_.size() -1; i++){
                        float distance = sqDistance(local_points_cluster_.points[i].x, local_points_cluster_.points[i].y, local_points_cluster_.points[i+1].x, local_points_cluster_.points[i+1].y);
                        if (distance > biggest_distance){
                            biggest_distance = distance;
                            index_cluster_left = i;
                            index_cluster_right = i+1;
                            first_point = local_points_cluster_.points[index_cluster_left];
                            last_point = local_points_cluster_.points[index_cluster_right];
                        }
                    }

                    float distance = sqDistance(local_points_cluster_.points[0].x, local_points_cluster_.points[0].y, local_points_cluster_.points.back().x, local_points_cluster_.points.back().y);
                    if (distance > biggest_distance){
                        biggest_distance = distance;
                        index_cluster_left = 0;
                        index_cluster_right = local_points_cluster_.points.size() - 1;
                        first_point = local_points_cluster_.points[index_cluster_left];
                        last_point = local_points_cluster_.points[index_cluster_right];
                    }
                }

                first_and_last_cluster_points.points.clear();
                first_and_last_cluster_points.points.push_back(first_point);
                first_and_last_cluster_points.points.push_back(last_point);
                first_and_last_cluster_points.header.stamp = ros::Time::now();
                first_and_last_cluster_points.header.frame_id = map_.header.frame_id;

                first_and_last_point_pub_.publish(first_and_last_cluster_points);
            }

        } else {
            local_points_cluster_.points.clear();
            local_points_cluster_.channels.clear();
        }

        last_point_inside_map.header.stamp = ros::Time::now();
        last_point_inside_map.header.frame_id = path_.header.frame_id;
        last_point_pub_.publish(last_point_inside_map);

        local_points_cluster_.header.stamp = ros::Time::now();
        local_points_pub_.publish(local_points_cluster_);

        double x_start = std::round(map_.info.width/2);
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
            // ROS_ERROR("No local path found!");
            return;
        } else {
            goal_path = motion_planning::trace_goal_path(nodes.back());
        }

        // Convert the path from nodes to a ROS path message
        nav_msgs::Path ros_path = convertNodesToPath(goal_path, map_);
        path_pub_.publish(ros_path);

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

    double distance(double x1, double y1, double x2, double y2) {
        return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    }

    double sqDistance(double x1, double y1, double x2, double y2) {
        return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
    }

    int findClosestPointIndex(const std::vector<geometry_msgs::Point32>& points, double x, double y) {

        int closestIndex = 0;
        double minDistance = std::numeric_limits<double>::max();

        for (int i = 0; i < points.size(); i++) {
            double dist = distance(x, y, points[i].x, points[i].y);
            if (dist < minDistance) {
                minDistance = dist;
                closestIndex = i;
            }
        }

        return closestIndex;
    }

    std::mutex local_map_mutex;
    cv::Mat Debug_image;
    int rate_;
    std::string local_map_topic_;
    std::string local_points_topic_;
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
    ros::Subscriber local_points_sub_;
    ros::Publisher local_points_pub_;
    ros::Publisher first_and_last_point_pub_;
    ros::Publisher debug_local_points_pub_;
    ros::Subscriber path_sub_;
    ros::Publisher last_point_pub_;
    ros::Publisher path_pub_;
    ros::ServiceClient static_map_client_;
    ros::ServiceClient update_map_client;
    ros::ServiceClient cluster_client_;
    std::vector<float> cluster_indices_;

    nav_msgs::OccupancyGrid map_;
    sensor_msgs::PointCloud local_points_;
    sensor_msgs::PointCloud debug_local_points_;
    sensor_msgs::PointCloud local_points_cluster_;
    sensor_msgs::PointCloud first_and_last_cluster_points;
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
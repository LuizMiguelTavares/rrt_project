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
    MapPathSubscriber() : tf_listener_(tf_buffer_), quad_tree_initialized_(false), is_rrt_completed_(false), Log_time(tic()), log_time(2.0), penalty_(0), first_point_obstructed_(false) {
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
        this->getParamOrThrow(private_nh, "number_of_bezier_points", number_of_bezier_points_);
        this->getParamOrThrow(private_nh, "first_point_obstructed_offset_", first_point_obstructed_offset_);

        path_sub_ = nh.subscribe(path_topic_, 1, &MapPathSubscriber::pathCallback, this);
        local_map_sub_ = nh.subscribe(local_map_topic_, 1, &MapPathSubscriber::mapCallback, this);

        last_point_pub_ = nh.advertise<geometry_msgs::PointStamped>("last_point_inside_map", 10);
        path_pub_ = nh.advertise<nav_msgs::Path>("local_path", 10);
        smoothed_path_pub_ = nh.advertise<nav_msgs::Path>("smoothed_local_path", 10);

        ros::service::waitForService("/static_map");
        static_map_client_ = nh.serviceClient<nav_msgs::GetMap>("/static_map");
        fetchStaticMap();

        private_nh.param<std::string>("laser_scan_topic", laser_scan_topic_, "/laser/scan");
        laser_scan_sub_ = nh.subscribe(laser_scan_topic_, 1, &MapPathSubscriber::laserScanCallback, this);

        point_pub = nh.advertise<geometry_msgs::PointStamped>("start_point_local_map", 10);
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
        cv::Mat mat(grid.info.width, grid.info.height, CV_8UC1);

        for (unsigned int y = 0; y < grid.info.height; y++) {
            for (unsigned int x = 0; x < grid.info.width; x++) {
                int i = x + y * grid.info.width;
                if (grid.data[i] == -1) {
                    mat.at<uchar>(x, grid.info.height - 1 - y) = 127;
                } else if (grid.data[i] == 0){
                    mat.at<uchar>(x, grid.info.height - 1 - y) = 255; 
                } else if (grid.data[i] == 100){
                    mat.at<uchar>(x, grid.info.height - 1 - y) = 0;}

            }
        }

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

        int first_index = closest_path_point->index;
        int last_index;
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
                    last_index = i;
                    last_point_valid = true;
                }
            }
        }

        if (penalty_ > 0) {
            if (last_index - penalty_ <= first_index){
                ROS_ERROR_STREAM("There is no valid point for the local path. Probably the problem is with the first point.");
                first_point_obstructed_ = true;
                penalty_= 0;
                return;
            }
            geometry_msgs::PoseStamped pose;
            pose.header = path_.header;
            pose.pose = path_.poses[last_index - penalty_].pose;

            geometry_msgs::PoseStamped pose_map_frame;
            transformPoseToMapFrame(pose, pose_map_frame);
            last_point_inside_map.point = pose.pose.position;
            last_point_inside_map_local_frame.point = pose_map_frame.pose.position;
            last_point_valid = true;
        }

        if (!last_point_valid) {
            ROS_ERROR("%s: No global path point inside the local map.", ros::this_node::getName().c_str());
            last_point_inside_map.header.stamp = ros::Time::now();
            last_point_inside_map.header.frame_id = path_.header.frame_id;
            last_point_pub_.publish(last_point_inside_map);
            return;
        }

        last_point_inside_map.header.stamp = ros::Time::now();
        last_point_inside_map.header.frame_id = path_.header.frame_id;
        last_point_pub_.publish(last_point_inside_map);

        double x_start = std::round(map_.info.width/2 + control_point_offset_ / map_.info.resolution);
        double y_start = std::round(map_.info.height/2);

        if (first_point_obstructed_){
            x_start = std::round(map_.info.width/2 + first_point_obstructed_offset_ / map_.info.resolution);
        }

        geometry_msgs::PointStamped point_msg;
        point_msg.header.stamp = ros::Time::now();
        point_msg.header.frame_id = map_frame_id_;  
        point_msg.point.x = (x_start - map_.info.width/2) * map_.info.resolution; 
        point_msg.point.y = (y_start - map_.info.height/2) * map_.info.resolution; 
        point_msg.point.z = 0.0;

        point_pub.publish(point_msg);

        double x_goal = std::round((last_point_inside_map_local_frame.point.x - map_.info.origin.position.x) / map_.info.resolution);
        double y_goal = std::round((last_point_inside_map_local_frame.point.y - map_.info.origin.position.y) / map_.info.resolution);

        y_goal = map_.info.height - y_goal;
        
        if (x_goal == map_.info.width) x_goal = map_.info.width - 1;
        if (y_goal == map_.info.height) y_goal = map_.info.height - 1;

        if (x_goal < 0 || x_goal >= map_.info.width || y_goal < 0 || y_goal >= map_.info.height) {
            ROS_ERROR("%s: Goal point outside the local map.", ros::this_node::getName().c_str());
            penalty_++;
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
            ROS_ERROR("No local path found!");
            penalty_++;
            return;
        } else {
            penalty_ = 0;
            first_point_obstructed_ = false;
            goal_path = motion_planning::trace_goal_path(nodes.back());
        }

        nav_msgs::Path ros_path = convertNodesToPath(goal_path, map_);
        nav_msgs::Path smoothed_path = smoothPath(ros_path, number_of_bezier_points_);

        smoothed_path_pub_.publish(smoothed_path);
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

    nav_msgs::Path smoothPath(const nav_msgs::Path& rrtPath, int numPoints, float smoothness = 0.01) {
        nav_msgs::Path smoothPath;
        smoothPath.header = rrtPath.header; 

        std::vector<std::pair<double, double>> controlPoints;
        std::vector<std::pair<double, double>> rrtPoints;

        for (const auto& pose : rrtPath.poses) {
            rrtPoints.emplace_back(pose.pose.position.x, pose.pose.position.y);
        }

        int n = numPoints > 0 ? numPoints : rrtPoints.size();

        if (n > rrtPoints.size()) {
            n = rrtPoints.size();
        }

        if (n > rrtPoints.size() || rrtPoints.size() < 4) {
            ROS_ERROR("Not enough points to form a Bézier curve. Need at least 4 points.");
            return rrtPath;
        }

        int step = (rrtPoints.size() - 1) / (n - 1);

        controlPoints.push_back(rrtPoints.front());

        for (int i = 0; i < n; ++i) {
            if (i * step == 0){
                continue;
            } else if (i * step >= rrtPoints.size()){
                break;
            }
            controlPoints.push_back(rrtPoints[i * step]);
        }

        controlPoints.push_back(rrtPoints.back());

        for (double t = 0.0; t <= 1.0; t += smoothness) {
            std::pair<double, double> pt = bezierPoint(controlPoints, t);

            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = pt.first;
            pose.pose.position.y = pt.second;
            pose.pose.position.z = 0.0;
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
    double radius_, first_point_obstructed_offset_;
    int radius_pixel_, number_of_bezier_points_;
    std::vector<double> robot_position;
    bool is_rrt_completed_;
    int penalty_;
    bool first_point_obstructed_;

    ros::Subscriber local_map_sub_;
    ros::Subscriber path_sub_;
    ros::Publisher last_point_pub_;
    ros::Publisher path_pub_;
    ros::Publisher smoothed_path_pub_;
    ros::Publisher point_pub;
    ros::ServiceClient static_map_client_;
    ros::ServiceClient update_map_client;

    nav_msgs::OccupancyGrid map_;
    nav_msgs::OccupancyGrid merged_map_;
    nav_msgs::OccupancyGrid global_map_;
    nav_msgs::Path path_;
    nav_msgs::Path traveled_path_;
    std::shared_ptr<QTree::QuadTree<motion_planning::Node>> quad_tree_;
    bool quad_tree_initialized_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

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
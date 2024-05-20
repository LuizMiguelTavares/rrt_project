#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "rrt_simple_ptr.hpp"

class RRTPathPlanner {
public:
    RRTPathPlanner() : tf_listener_(tf_buffer_) {
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");

        // Get parameters
        private_nh.param<std::string>("path_topic", path_topic_, "/rrt_path");
        private_nh.param<int>("num_nodes", num_nodes_, 1000);
        private_nh.param<double>("step_size", step_size_, 1.0);
        private_nh.param<double>("goal_threshold", goal_threshold_, 1.0);
        private_nh.param<double>("bias_probability", bias_probability_, 0.1);
        private_nh.param<double>("radius", radius_, 1.0);

        private_nh.param<std::string>("robot_frame", robot_frame_, "P1");
        private_nh.param<std::string>("world_frame", world_frame_, "world");

        private_nh.param<double>("goal_x", goal_x_, 10.0);
        private_nh.param<double>("goal_y", goal_y_, 10.0);

        // Advertise the RRT path
        path_pub_ = nh.advertise<nav_msgs::Path>(path_topic_, 1);

        // Fetch the map from the service
        ros::service::waitForService("static_map");
        static_map_client_ = nh.serviceClient<nav_msgs::GetMap>("static_map");

        // Initialize path generation status
        path_generated_ = false;
    }

    bool fetchStaticMap() {
        nav_msgs::GetMap srv;
        if (static_map_client_.call(srv)) {
            global_map_ = srv.response.map;
            global_map_frame_ = global_map_.header.frame_id;
            ROS_INFO("Successfully called static_map service");
            return true;
        } else {
            ROS_ERROR("Failed to call static_map service");
            return false;
        }
    }

    void generateRRT() {
        // Transform the robot's pose to the map frame
        geometry_msgs::PoseStamped start_map;
        if (!transformPoseToMapFrame(robot_frame_, global_map_frame_, start_map)) {
            ROS_ERROR("Failed to transform robot pose to map frame");
            return;
        }

        // Transform the goal pose to the map frame
        geometry_msgs::PoseStamped goal_pose;
        goal_pose.header.frame_id = world_frame_;
        goal_pose.pose.position.x = goal_x_;
        goal_pose.pose.position.y = goal_y_;
        geometry_msgs::PoseStamped goal_map;
        if (!transformPoseToMapFrame(goal_pose, goal_map)) {
            ROS_ERROR("Failed to transform goal pose to map frame");
            return;
        }

        int start_x = (start_map.pose.position.x - global_map_.info.origin.position.x) / global_map_.info.resolution;
        int start_y = (start_map.pose.position.y - global_map_.info.origin.position.y) / global_map_.info.resolution;
        int goal_x = (goal_map.pose.position.x - global_map_.info.origin.position.x) / global_map_.info.resolution;
        int goal_y = (goal_map.pose.position.y - global_map_.info.origin.position.y) / global_map_.info.resolution;

        std::shared_ptr<motion_planning::Node> start_node = std::make_shared<motion_planning::Node>(start_x, start_y, nullptr);
        std::shared_ptr<motion_planning::Node> goal_node = std::make_shared<motion_planning::Node>(goal_x, goal_y, nullptr);

        ROS_INFO("Start: (%f, %f), Goal: (%f, %f)", start_map.pose.position.x, start_map.pose.position.y, goal_map.pose.position.x, goal_map.pose.position.y);

        // Convert the map to a CV Mat for RRT
        cv::Mat map_image = occupancyGridToCvMat(global_map_);

        // Run RRT
        std::vector<std::shared_ptr<motion_planning::Node>> nodes = motion_planning::rrt(map_image, start_node, goal_node, num_nodes_, step_size_, goal_threshold_, bias_probability_, radius_);

        if (nodes.empty()) {
            ROS_ERROR("No path found from RRT within the global map.");
            return;
        }

        // Convert nodes to ROS Path
        rrt_path_ = convertNodesToPath(nodes, global_map_);
        path_generated_ = true;
    }

    void publishPath() {
        if (path_generated_) {
            rrt_path_.header.stamp = ros::Time::now();
            path_pub_.publish(rrt_path_);
        }
    }

private:
    ros::Publisher path_pub_;
    ros::ServiceClient static_map_client_;
    nav_msgs::OccupancyGrid global_map_;
    nav_msgs::Path rrt_path_;

    std::string path_topic_;
    int num_nodes_;
    double step_size_;
    double goal_threshold_;
    double bias_probability_;
    double radius_;

    std::string robot_frame_;
    std::string world_frame_;
    double goal_x_;
    double goal_y_;
    std::string global_map_frame_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    bool path_generated_;

    cv::Mat occupancyGridToCvMat(const nav_msgs::OccupancyGrid& grid) {
        cv::Mat mat(grid.info.height, grid.info.width, CV_8UC1);
        for (unsigned int y = 0; y < grid.info.height; y++) {
            for (unsigned int x = 0; x < grid.info.width; x++) {
                int i = x + (grid.info.height - y - 1) * grid.info.width;
                if (grid.data[i] == -1) {
                    mat.at<uchar>(y, x) = 127;
                } else if (grid.data[i] == 0) {
                    mat.at<uchar>(y, x) = 255;
                } else {
                    mat.at<uchar>(y, x) = 0;
                }
            }
        }
        return mat;
    }

    nav_msgs::Path convertNodesToPath(const std::vector<std::shared_ptr<motion_planning::Node>>& nodes, const nav_msgs::OccupancyGrid& map) {
        nav_msgs::Path path;
        path.header.frame_id = map.header.frame_id;
        path.header.stamp = ros::Time::now();

        for (const auto& node : nodes) {
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = map.header.frame_id;
            pose.pose.position.x = node->x * map.info.resolution + map.info.origin.position.x;
            pose.pose.position.y = node->y * map.info.resolution + map.info.origin.position.y;
            pose.pose.orientation.w = 1.0;
            path.poses.push_back(pose);
        }

        return path;
    }

    bool transformPoseToMapFrame(const std::string& source_frame, const std::string& target_frame, geometry_msgs::PoseStamped& output_pose) {
        geometry_msgs::PoseStamped input_pose;
        input_pose.header.frame_id = source_frame;
        input_pose.pose.orientation.w = 1.0;

        try {
            geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(3.0));
            tf2::doTransform(input_pose, output_pose, transform);
            return true;
        } catch (tf2::TransformException& ex) {
            ROS_ERROR("Transform error: %s", ex.what());
            return false;
        }
    }

    bool transformPoseToMapFrame(const geometry_msgs::PoseStamped& input_pose, geometry_msgs::PoseStamped& output_pose) {
        try {
            geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(global_map_frame_, input_pose.header.frame_id, ros::Time(0), ros::Duration(3.0));
            tf2::doTransform(input_pose, output_pose, transform);
            return true;
        } catch (tf2::TransformException& ex) {
            ROS_ERROR("Transform error: %s", ex.what());
            return false;
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "rrt_path_planner");
    RRTPathPlanner planner;

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Duration(1.0).sleep();  // Allow time for fetching the map

    if (planner.fetchStaticMap()) {
        planner.generateRRT();
    }

    ros::Rate rate(1.0);  // Adjust the rate as needed
    while (ros::ok()) {
        planner.publishPath();
        rate.sleep();
    }

    ros::waitForShutdown();
    return 0;
}
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include "rrt_simple_ptr.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

class MinimalRRTTestNode {
public:
    MinimalRRTTestNode() : start_x_(20), start_y_(20), goal_x_(30), goal_y_(30){
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");

        map_sub_ = nh.subscribe("/local_map", 1, &MinimalRRTTestNode::mapCallback, this);
        start_node = std::make_shared<motion_planning::Node>(20, 20, nullptr);
        goal_node = std::make_shared<motion_planning::Node>(30, 30, nullptr);

        ROS_INFO("Start position: (%f, %f)", start_x_, start_y_);
        ROS_INFO("Goal position: (%f, %f)", goal_x_, goal_y_);
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        ROS_INFO("Received map");
        runRRT(*msg, start_x_, start_y_, goal_x_, goal_y_);
    }

    void runRRT(const nav_msgs::OccupancyGrid& map, double start_x, double start_y, double goal_x, double goal_y) {
        cv::Mat grid_map = convertMapToCvMat(map);

        

        int num_nodes = 100000;
        float step_size = 0.5;
        float goal_threshold = 0.2;
        float bias_probability = 1;
        int radius = 1;
        std::vector<std::shared_ptr<motion_planning::Node>> path = motion_planning::rrt(grid_map, start_node, goal_node, num_nodes, step_size, goal_threshold, bias_probability, radius);

        if (path.empty()) {
            ROS_WARN("No path found.");
        } else {
            ROS_ERROR("Path found with %lu nodes.", path.size());
        }
    }

    cv::Mat convertMapToCvMat(const nav_msgs::OccupancyGrid& map) {
        cv::Mat mat(map.info.height, map.info.width, CV_8UC1);

        for (unsigned int y = 0; y < map.info.height; y++) {
            for (unsigned int x = 0; x < map.info.width; x++) {
                int i = x + y * map.info.width;
                if (map.data[i] == 0) {
                    mat.at<uchar>(y, x) = 255; // Free space
                } else {
                    mat.at<uchar>(y, x) = 0; // Obstacle
                }
            }
        }
        return mat;
    }

private:
    ros::Subscriber map_sub_;
    double start_x_, start_y_, goal_x_, goal_y_;
    std::shared_ptr<motion_planning::Node> start_node;
    std::shared_ptr<motion_planning::Node> goal_node;

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "minimal_rrt_test_node");
    MinimalRRTTestNode node;
    ros::spin();
    return 0;
}
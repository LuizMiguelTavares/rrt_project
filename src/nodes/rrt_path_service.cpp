#include <ros/ros.h>
#include "your_package_name/PlanPath.srv"  // Include your custom service message
#include "rrt_simple_source.hpp"
#include <opencv2/core.hpp>

class RoutePlanner
{
public:
    RoutePlanner() {
        ros::NodeHandle nh;
        service = nh.advertiseService("plan_path", &RoutePlanner::planPath, this);
        ROS_INFO("Route Planner Ready to plan paths.");
    }

    bool planPath(your_package_name::PlanPath::Request &req,
                  your_package_name::PlanPath::Response &res) {
        ROS_INFO("Received path planning request.");

        auto map = req.map;
        auto start_point = req.start;
        auto goal_point = req.goal;

        // Process the map and points using your RRT algorithm
        // Assume you convert points to your internal representation and run RRT
        auto path = rrt_plan(map, start_point, goal_point);

        if (path.empty()) {
            res.success = false;
            res.message = "Path planning failed.";
            return true;
        }

        // Convert your path format back to nav_msgs/Path if necessary
        res.path = convertPathToNavMsgsPath(path);
        res.success = true;
        return true;
    }

private:
    ros::ServiceServer service;

    std::vector<motion_planning::Node*> rrt_plan(const nav_msgs::OccupancyGrid& map,
                                                 const geometry_msgs::Point& start,
                                                 const geometry_msgs::Point& goal) {
        // Convert ROS message types to internal representation
        cv::Mat cv_map = occupancyGridToCvMat(map);

        // Convert the start and goal points into grid coordinates
        std::vector<double> start_pos_grid = {
            std::round((start.x - map.info.origin.position.x) / map.info.resolution),
            std::round(map.info.height - ((start.y - map.info.origin.position.y) / map.info.resolution))
        };

        std::vector<double> goal_pos_grid = {
            std::round((goal.x - map.info.origin.position.x) / map.info.resolution),
            std::round(map.info.height - ((goal.y - map.info.origin.position.y) / map.info.resolution))
        };

        std::unique_ptr<motion_planning::Node> start_node = std::make_unique<motion_planning::Node>(start_pos_grid, nullptr);
        std::unique_ptr<motion_planning::Node> goal_node = std::make_unique<motion_planning::Node>(goal_pos_grid, nullptr);

        // Run RRT
        std::vector<motion_planning::Node*> nodes = motion_planning::rrt(cv_map, start_node.get(), goal_node.get(), num_nodes, step_size, goal_threshold, bias_probability, radius_pixel);

        if (!nodes.empty() && motion_planning::distance(*nodes.back(), *goal_node) < goal_threshold) {
            return motion_planning::trace_goal_path(nodes[nodes.size() - 1]);
        }
        return {};
    }


    nav_msgs::Path convertPathToNavMsgsPath(const std::vector<motion_planning::Node*>& nodes) {
        nav_msgs::Path path;
        path.header.stamp = ros::Time::now();
        path.header.frame_id = "map"; // Adjust according to your frame_id

        for (auto& node : nodes) {
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header = path.header;
            pose_stamped.pose.position.x = node->position[0] * map.info.resolution + map.info.origin.position.x;
            pose_stamped.pose.position.y = (map.info.height - node->position[1]) * map.info.resolution + map.info.origin.position.y;
            pose_stamped.pose.position.z = 0; // Assuming the floor plane
            pose_stamped.pose.orientation.w = 1.0; // No rotation
            path.poses.push_back(pose_stamped);
        }
        return path;
    }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "route_planner");
    RoutePlanner planner;
    ros::spin();
    return 0;
}
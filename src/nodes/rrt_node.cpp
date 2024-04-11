#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

class MapTrajectoryPlanner {
public:
    MapTrajectoryPlanner() {
        // Initialize the ROS node handle
        ros::NodeHandle nh;

        // Subscribe to the map topic
        map_subscriber_ = nh.subscribe("map", 1, &MapTrajectoryPlanner::mapCallback, this);
    }

    void planTrajectory() {
        // Implement your trajectory planning logic here
        // For example, you can use the map received in the callback to plan a trajectory
        ROS_INFO("Planning trajectory...");
    }

private:
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        // This callback function is called whenever a new map message is received
        ROS_INFO("Received a map!");

        // Here you can store the map data and use it in your planning algorithm
        // Example: map_ = *msg;

        // After receiving the map, you might want to plan a trajectory
        planTrajectory();
    }

    ros::Subscriber map_subscriber_;
    // nav_msgs::OccupancyGrid map_; // Uncomment if you want to store the map
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_trajectory_planner");
    MapTrajectoryPlanner planner;
    ros::spin();
    return 0;
}

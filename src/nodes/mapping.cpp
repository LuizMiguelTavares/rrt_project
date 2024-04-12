#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>

class GridMapper {
public:
    GridMapper() {
        // Initialize the node handle
        ros::NodeHandle nh;

        // Create a publisher for the grid map
        map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 1, true);

        // Subscribe to pose and laser scan topics
        pose_sub = nh.subscribe("odom", 10, &GridMapper::poseCallback, this);
        scan_sub = nh.subscribe("scan", 10, &GridMapper::scanCallback, this);

        // Initialize the grid map
        initializeMap();
    }

    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
        current_pose = *msg;
        updateMap();
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        current_scan = *msg;
        updateMap();
    }

    void initializeMap() {
        map.header.frame_id = "map";
        map.info.resolution = 0.1;  // 10 cm per grid cell
        map.info.width = 100;       // 10 meters wide
        map.info.height = 100;      // 10 meters high
        map.info.origin.position.x = -5.0;  // Center the map at the robot
        map.info.origin.position.y = -5.0;
        map.data.resize(map.info.width * map.info.height, -1);
    }

    void updateMap() {
        if (current_scan.ranges.empty()) {
            return; // No scan data available
        }

        // Convert pose to map coordinates
        double robot_x = current_pose.pose.pose.position.x - map.info.origin.position.x;
        double robot_y = current_pose.pose.pose.position.y - map.info.origin.position.y;
        double robot_theta = tf::getYaw(current_pose.pose.pose.orientation);

        // Process each scan point
        for (size_t i = 0; i < current_scan.ranges.size(); ++i) {
            // Calculate the angle of the current scan point
            double angle = robot_theta + current_scan.angle_min + i * current_scan.angle_increment;

            // Check if the scan is within range limits
            if (current_scan.ranges[i] < current_scan.range_min || current_scan.ranges[i] > current_scan.range_max) {
                continue; // Skip invalid or out of range values
            }

            // Calculate the coordinates of the scan point in the map frame
            double scan_x = robot_x + current_scan.ranges[i] * cos(angle) / map.info.resolution;
            double scan_y = robot_y + current_scan.ranges[i] * sin(angle) / map.info.resolution;

            // Convert to map grid indices
            int grid_x = static_cast<int>(floor(scan_x));
            int grid_y = static_cast<int>(floor(scan_y));

            // Check if indices are within map bounds
            if (grid_x >= 0 && grid_x < map.info.width && grid_y >= 0 && grid_y < map.info.height) {
                // Update the map cell
                int index = grid_x + grid_y * map.info.width;
                map.data[index] = 100; // Mark as occupied (100)
            }
        }

        // Publish the updated map
        map_pub.publish(map);
    }

private:
    ros::Publisher map_pub;
    ros::Subscriber pose_sub, scan_sub;
    geometry_msgs::PoseWithCovarianceStamped current_pose;
    sensor_msgs::LaserScan current_scan;
    nav_msgs::OccupancyGrid map;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "grid_mapper");
    GridMapper gridMapper;
    ros::spin();
    return 0;
}

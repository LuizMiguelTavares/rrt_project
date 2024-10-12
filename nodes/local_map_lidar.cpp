#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class LocalMapNode {
public:
    LocalMapNode(ros::NodeHandle& nh, double map_size)
        : map_size_(map_size), grid_resolution_(0.02) { 

        ros::NodeHandle private_nh("~");
        private_nh.param<std::string>("lidar_topic", lidar_topic, "/rplidar/scan");

        lidar_sub_ = nh.subscribe(lidar_topic, 10, &LocalMapNode::lidarCallback, this);

        map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/local_map_new", 10);

        // Initialize occupancy grid parameters
        local_map_.info.resolution = grid_resolution_;
        int grid_size = static_cast<int>(map_size_ / grid_resolution_);
        local_map_.info.width = grid_size;
        local_map_.info.height = grid_size;
        local_map_.info.origin.position.x = -map_size_ / 2;
        local_map_.info.origin.position.y = -map_size_ / 2;
        local_map_.info.origin.position.z = 0.0;
        local_map_.data.assign(grid_size * grid_size, 0);
    }

    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        std::fill(local_map_.data.begin(), local_map_.data.end(), 0);

        double angle = scan->angle_min;

        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            double range = scan->ranges[i];

            if (range >= scan->range_min && range <= scan->range_max) {
                double x = range * cos(angle);
                double y = range * sin(angle);

                int grid_x = static_cast<int>((x + map_size_ / 2) / grid_resolution_);
                int grid_y = static_cast<int>((y + map_size_ / 2) / grid_resolution_);

                if (grid_x >= 0 && grid_x < local_map_.info.width &&
                    grid_y >= 0 && grid_y < local_map_.info.height) {
                    int index = grid_x + grid_y * local_map_.info.width;
                    local_map_.data[index] = 100;
                }
            }
            angle += scan->angle_increment;
        }

        local_map_.header.stamp = ros::Time::now();
        local_map_.header.frame_id = scan->header.frame_id;
        map_pub_.publish(local_map_);
    }

private:
    std::string lidar_topic;
    ros::Subscriber lidar_sub_;
    ros::Publisher map_pub_;
    nav_msgs::OccupancyGrid local_map_;
    double map_size_;
    double grid_resolution_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "local_map_node");
    ros::NodeHandle nh;

    double map_size;
    nh.param("map_size", map_size, 2.0); 

    LocalMapNode local_map_node(nh, map_size);

    ros::spin();
    return 0;
}

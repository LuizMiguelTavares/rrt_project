#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class LocalOccupancyGrid {
public:
    LocalOccupancyGrid() : tf2_listener_(tf2_buffer_) {
        ros::NodeHandle private_nh("~");

        private_nh.param<std::string>("laser_topic", laser_topic_, "/RosAria/scan");
        private_nh.param<std::string>("grid_topic", grid_topic_, "local_grid");
        private_nh.param("rate", rate_, 10);
        private_nh.param("resolution", resolution_, 0.05);
        private_nh.param("width", width_, 40);
        private_nh.param("height", height_, 40);

        grid_.info.resolution = resolution_;
        grid_.info.width = width_;
        grid_.info.height = height_;
        grid_.data.resize(width_ * height_, -1);
        grid_.info.origin.position.x = -resolution_ * width_ / 2;
        grid_.info.origin.position.y = -resolution_ * height_ / 2;
        grid_.info.origin.orientation.w = 1.0;
        
        grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(grid_topic_, 50);
        laser_sub_ = nh_.subscribe(laser_topic_, 50, &LocalOccupancyGrid::laserCallback, this);
    }

    void run() {
        ros::Rate rate(rate_);
        while (ros::ok()) {
            ros::spinOnce();
            grid_.header.stamp = ros::Time::now();
            grid_pub_.publish(grid_);
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher grid_pub_;
    ros::Subscriber laser_sub_;
    nav_msgs::OccupancyGrid grid_;
    tf2_ros::Buffer tf2_buffer_;
    tf2_ros::TransformListener tf2_listener_;
    std::string grid_topic_;
    std::string laser_topic_;
    double resolution_;
    int rate_;
    int width_;
    int height_;

    void markCell(int x, int y, int value) {
        if (x >= 0 && x < width_ && y >= 0 && y < height_) {
            grid_.data[y * width_ + x] = value;
        }
    }

    void bresenhamLine(int x0, int y0, int x1, int y1, int value) {
        int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx + dy, e2;

        while (true) {
            markCell(x0, y0, value);
            if (x0 == x1 && y0 == y1) break;
            e2 = 2 * err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }
    }

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        // Adjust the frame ID of the grid to the incoming laser scan frame
        grid_.header.frame_id = scan->header.frame_id;

        // Origin of the laser in the grid
        int origin_x = width_ / 2;
        int origin_y = height_ / 2;

        for (unsigned int i = 0; i < scan->ranges.size(); ++i) {
            double angle = scan->angle_min + i * scan->angle_increment;
            double range = scan->ranges[i];
            if (range >= scan->range_min && range <= scan->range_max) {
                int end_x = origin_x + (int)(range * cos(angle) / resolution_);
                int end_y = origin_y + (int)(range * sin(angle) / resolution_);

                // Mark the path of the laser beam as free
                bresenhamLine(origin_x, origin_y, end_x, end_y, 0);

                // Mark the endpoint of the laser beam as occupied
                markCell(end_x, end_y, 100);
            }
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "local_occupancy_grid");
    LocalOccupancyGrid localGrid;
    localGrid.run();
    return 0;
}
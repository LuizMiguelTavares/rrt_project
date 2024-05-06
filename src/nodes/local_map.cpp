#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class LocalOccupancyGrid {
public:
    LocalOccupancyGrid() : tf2_listener_(tf2_buffer_) {
        ros::NodeHandle private_nh("~");  // Private node handle to read parameters

        // Read parameters
        private_nh.param<std::string>("world_frame", world_frame_, "world");
        private_nh.param<std::string>("laser_topic", laser_topic_, "/RosAria/scan");
        private_nh.param<std::string>("grid_topic", grid_topic_, "local_grid");
        private_nh.param("resolution", resolution_, 0.05);
        private_nh.param("width", width_, 40);
        private_nh.param("height", height_, 40);

        // Initialize the grid
        grid_.info.resolution = resolution_;
        grid_.info.width = width_;
        grid_.info.height = height_;
        grid_.data.resize(width_ * height_, -1);
        grid_.info.origin.orientation.w = 1.0;
        grid_.header.frame_id = world_frame_;

        // Setup ROS publishers and subscribers
        grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(grid_topic_, 50);
        laser_sub_ = nh_.subscribe(laser_topic_, 50, &LocalOccupancyGrid::laserCallback, this);
    }

    void run() {
        ros::Rate rate(1.0);
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
    std::string world_frame_;
    std::string grid_topic_;
    std::string laser_topic_;
    double resolution_;
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
        geometry_msgs::TransformStamped transform;
        try {
            transform = tf2_buffer_.lookupTransform(world_frame_, scan->header.frame_id,
                                                    ros::Time(0), ros::Duration(1.0));
        } catch (tf2::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            return;
        }

        // Compute the origin of the laser in the grid
        int origin_x = (transform.transform.translation.x - grid_.info.origin.position.x) / resolution_;
        int origin_y = (transform.transform.translation.y - grid_.info.origin.position.y) / resolution_;

        tf2::Transform tf2_transform;
        tf2::fromMsg(transform.transform, tf2_transform);

        for (unsigned int i = 0; i < scan->ranges.size(); ++i) {
            double angle = scan->angle_min + i * scan->angle_increment;
            double range = scan->ranges[i];
            if (range >= scan->range_min && range <= scan->range_max) {
                // Transform the point from the laser frame to the world frame
                tf2::Vector3 point_laser(range * cos(angle), range * sin(angle), 0.0);
                tf2::Vector3 point_world = tf2_transform * point_laser;

                // Convert world coordinates to grid coordinates
                int end_x = (point_world.x() - grid_.info.origin.position.x) / resolution_;
                int end_y = (point_world.y() - grid_.info.origin.position.y) / resolution_;

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
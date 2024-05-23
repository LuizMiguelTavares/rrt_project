#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/PointStamped.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <algorithm>

class MapCreator
{
public:
    MapCreator() : tfListener(tfBuffer), map_width(10.0), map_height(10.0), resolution(0.1) {
        laser_subscriber = nh.subscribe("/laser/scan", 10, &MapCreator::laser_reading, this);
        grid_width = static_cast<int>(map_width / resolution);
        grid_height = static_cast<int>(map_height / resolution);
        grid_map = cv::Mat(grid_height, grid_width, CV_32F, cv::Scalar(0.5));  // Initialize as unknown (0.5)
        cv::namedWindow("Laser Grid", cv::WINDOW_AUTOSIZE);
    }

    ~MapCreator()
    {
        cv::destroyAllWindows();
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber laser_subscriber;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    cv::Mat grid_map;
    double map_width, map_height, resolution;
    int grid_width, grid_height;

void laser_reading(const sensor_msgs::LaserScan::ConstPtr& msg) {
        geometry_msgs::TransformStamped transformStamped;
        try {
            transformStamped = tfBuffer.lookupTransform("odom", "base_link", ros::Time(0), ros::Duration(1.0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            return;
        }

        // Prepare to mark free areas
        int free_value = 0;  // White
        int occupied_value = 1;  // Black
        float unknown_value = 0.5f;  // Gray

        // Clear the grid map if necessary or leave it to accumulate values
        grid_map.setTo(cv::Scalar(unknown_value));

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            if (msg->ranges[i] < std::numeric_limits<float>::infinity()) {
                double angle = msg->angle_min + i * msg->angle_increment;
                geometry_msgs::PointStamped local_point, global_point;
                local_point.header.frame_id = "base_link";
                local_point.point.x = msg->ranges[i] * cos(angle);
                local_point.point.y = msg->ranges[i] * sin(angle);
                local_point.point.z = 0;

                tf2::doTransform(local_point, global_point, transformStamped);

                int grid_x = static_cast<int>((global_point.point.x + map_width / 2) / resolution);
                int grid_y = static_cast<int>((global_point.point.y + map_height / 2) / resolution);

                if (grid_x >= 0 && grid_x < grid_width && grid_y >= 0 && grid_y < grid_height) {
                    grid_map.at<float>(grid_y, grid_x) = occupied_value;
                }
            }
        }

        // Convert grid_map values for display: 0 (free) as white, 1 (occupied) as black, 0.5 (unknown) as gray
        cv::Mat display_map;
        grid_map.convertTo(display_map, CV_8UC1, 255, 127.5);  // Scale and offset to map 0, 0.5, 1 to 255, 127, 0

        cv::resize(display_map, display_map, cv::Size(500, 500), 0, 0, cv::INTER_NEAREST);
        cv::imshow("Laser Grid", display_map);
        cv::waitKey(1);  // Refresh the display
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_creator");
    MapCreator mapCreator;
    ros::spin();
    return 0;
}
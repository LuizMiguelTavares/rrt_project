#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>

class RectanglePerimeterPublisher
{
public:
    RectanglePerimeterPublisher() :
        nh_("~"),
        tf_listener_(tf_buffer_)
    {
        // Parameters
        nh_.param("width", x_size_, 0.322);  // Rectangle size in x (meters)
        nh_.param("height", y_size_, 0.220);  // Rectangle size in y (meters)
        nh_.param("resolution", resolution_, 0.1);  // Point cloud resolution
        nh_.param("x_offset", x_offset_, 0.0);  // Offset in x
        nh_.param("y_offset", y_offset_, 0.0);  // Offset in y
        nh_.param("robot_frame", robot_frame_, std::string("base_link"));  // Robot frame

        // Publisher for the PointCloud
        pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>("rectangle_perimeter", 1);

        // Timer to periodically publish the point cloud
        timer_ = nh_.createTimer(ros::Duration(0.1), &RectanglePerimeterPublisher::publishPointCloud, this);
    }

    void publishPointCloud(const ros::TimerEvent&)
    {
        sensor_msgs::PointCloud cloud;
        cloud.header.frame_id = robot_frame_;
        cloud.header.stamp = ros::Time::now();

        // Generate points for the perimeter of the rectangle

        // Bottom and top sides (vary x, keep y constant)
        for (double x = -x_size_ / 2.0; x <= x_size_ / 2.0; x += resolution_)
        {
            // Bottom side (y = -y_size_ / 2)
            geometry_msgs::Point32 point_bottom;
            point_bottom.x = x + x_offset_;
            point_bottom.y = -y_size_ / 2.0 + y_offset_;
            point_bottom.z = 0.0;
            cloud.points.push_back(point_bottom);

            // Top side (y = y_size_ / 2)
            geometry_msgs::Point32 point_top;
            point_top.x = x + x_offset_;
            point_top.y = y_size_ / 2.0 + y_offset_;
            point_top.z = 0.0;
            cloud.points.push_back(point_top);
        }

        // Left and right sides (vary y, keep x constant)
        for (double y = -y_size_ / 2.0 + resolution_; y <= y_size_ / 2.0 - resolution_; y += resolution_)  // Avoid duplicate corner points
        {
            // Left side (x = -x_size_ / 2)
            geometry_msgs::Point32 point_left;
            point_left.x = -x_size_ / 2.0 + x_offset_;
            point_left.y = y + y_offset_;
            point_left.z = 0.0;
            cloud.points.push_back(point_left);

            // Right side (x = x_size_ / 2)
            geometry_msgs::Point32 point_right;
            point_right.x = x_size_ / 2.0 + x_offset_;
            point_right.y = y + y_offset_;
            point_right.z = 0.0;
            cloud.points.push_back(point_right);
        }

        // Publish the cloud
        pointcloud_pub_.publish(cloud);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pointcloud_pub_;
    ros::Timer timer_;

    // tf2 listener
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Parameters
    double x_size_, y_size_, resolution_, x_offset_, y_offset_;
    std::string robot_frame_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rectangle_perimeter_publisher");
    RectanglePerimeterPublisher node;
    ros::spin();
    return 0;
}
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>

class RobotPointsPublisher {
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    std::vector<geometry_msgs::Point32> base_points_;
    int robot_density_;
    double robot_height_, robot_width_;
    std::string world_frame_, robot_frame_;

public:
    RobotPointsPublisher() : tfListener_(tfBuffer_) {
        nh_.param("robot_density", robot_density_, 100);
        nh_.param("robot_height", robot_height_, 1.0);
        nh_.param("robot_width", robot_width_, 1.0);
        nh_.param("world_frame", world_frame_, std::string("world"));
        nh_.param("robot_frame", robot_frame_, std::string("robot_base"));
        pub_ = nh_.advertise<sensor_msgs::PointCloud>("points", 10);

        ROS_INFO_STREAM("robot_points_publisher: Using world frame: " << world_frame_ << " and robot frame: " << robot_frame_ << "\n");

        double theta = 0.0;
        double step = 2 * M_PI / robot_density_;
        for (int i = 0; i < robot_density_; i++, theta += step) {
            geometry_msgs::Point32 point;
            point.x = (robot_height_ / 2) * cos(theta);
            point.y = (robot_width_ / 2) * sin(theta);
            point.z = 0;
            base_points_.push_back(point);
        }
    }

    void publishTransformedPoints() {
        geometry_msgs::TransformStamped transform_stamped;
        try {
            transform_stamped = tfBuffer_.lookupTransform(world_frame_, robot_frame_, ros::Time(0), ros::Duration(3.0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            return;
        }

        sensor_msgs::PointCloud cloud;
        cloud.header.stamp = ros::Time::now();
        cloud.header.frame_id = world_frame_;

        for (auto &point : base_points_) {
            geometry_msgs::PointStamped local_point, global_point;
            local_point.point.x = point.x;
            local_point.point.y = point.y;
            local_point.point.z = point.z;
            local_point.header.frame_id = robot_frame_;
            local_point.header.stamp = ros::Time(0);

            tf2::doTransform(local_point, global_point, transform_stamped);

            geometry_msgs::Point32 transformed_point;
            transformed_point.x = global_point.point.x;
            transformed_point.y = global_point.point.y;
            transformed_point.z = global_point.point.z;
            cloud.points.push_back(transformed_point);
        }

        pub_.publish(cloud);
    }

    void run() {
        ros::Rate rate(10);
        while (ros::ok()) {
            publishTransformedPoints();
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_points_publisher");
    RobotPointsPublisher robot_points_publisher;
    robot_points_publisher.run();
    return 0;
}
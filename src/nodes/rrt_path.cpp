#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <obstacle_avoidance_drone_follower/ObjectPoints.h>
#include "rrt.hpp"
#include <cmath>
#include <vector>

class RoutePublisher
{
public:
    RoutePublisher() {
        ros::NodeHandle nh("~");

        QTree::Rectangle boundary(10, 10, 20, 20);
        motion_planning::Node* start = new motion_planning::Node({10, 10}, nullptr);
        QTree::QuadTree<motion_planning::Node> tree(boundary, start, 4);

        nh.param("x", x, 0.0);
        nh.param("y", y, 0.0);

        std::string ns = ros::this_node::getNamespace();
        pub = nh.advertise<obstacle_avoidance_drone_follower::ObjectPoints>(ns + "/route", 10);

        rate = new ros::Rate(30);  // Rate at which to publish the points
        ROS_INFO_STREAM(ns.substr(1) << " route publisher node started!");
    }

    ~RoutePublisher() {
        delete rate;
    }

    void circular_route() {
        while (ros::ok()) {
            // ros::Time current_time = ros::Time::now();
            // std::vector<geometry_msgs::Point> points;
            // points.reserve(resolution);

            // double radians_per_step = 2 * M_PI / resolution;
            // for (int i = 0; i < resolution; ++i) {
            //     double rad = i * radians_per_step;
            //     geometry_msgs::Point point;
            //     point.x = x_radius * cos(rad) + x_center;
            //     point.y = y_radius * sin(rad) + y_center;
            //     point.z = z_height;
            //     points.push_back(point);
            // }

            // obstacle_avoidance_drone_follower::ObjectPoints route;
            // route.header.stamp = current_time;
            // route.header.frame_id = "route";
            // route.points = points;
            // pub.publish(route);

            rate->sleep();
            ros::spinOnce();
        }
    }

private:
    ros::Publisher pub;
    ros::Rate *rate;
    double x, y;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "circular_path_publisher");
    RoutePublisher route;
    route.circular_route();

    return 0;
}
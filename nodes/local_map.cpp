#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <cmath>

class LocalMapNode {
public:
    LocalMapNode() {
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");

        // Parameters
        private_nh.param("global_map_topic", global_map_topic, std::string("/map"));
        private_nh.param("local_map_topic", local_map_topic, std::string("local_map"));
        private_nh.param("pose_topic", pose_topic, std::string("/vrpn_client_node/P1/pose"));
        private_nh.param("width_meters", width_meters, 2.0);
        private_nh.param("height_meters", height_meters, 2.0);
        private_nh.param("local_resolution", local_resolution, 0.01);
        private_nh.param("publish_rate", publish_rate, 10);

        // Subscribers and Publishers
        global_map_sub = nh.subscribe(global_map_topic, 1, &LocalMapNode::globalMapCallback, this);
        pose_sub = nh.subscribe(pose_topic, 1, &LocalMapNode::poseCallback, this);
        local_map_pub = nh.advertise<nav_msgs::OccupancyGrid>(local_map_topic, 10);

        // Main loop
        ros::Rate rate(publish_rate);
        while (ros::ok()) {
            publishLocalMap();
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    void globalMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        global_map = *msg;
        global_resolution = msg->info.resolution;
        width_cells = static_cast<int>(width_meters / local_resolution);
        height_cells = static_cast<int>(height_meters / local_resolution);
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_pose = *msg;
    }

    void publishLocalMap() {
        if (!global_map.data.empty() && current_pose.header.stamp != ros::Time(0)) {
            double robot_x = current_pose.pose.position.x;
            double robot_y = current_pose.pose.position.y;
            tf::Quaternion quat(current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w);
            double roll, pitch, yaw;
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

            double map_origin_x = global_map.info.origin.position.x;
            double map_origin_y = global_map.info.origin.position.y;
            int map_width = global_map.info.width;
            int map_height = global_map.info.height;

            nav_msgs::OccupancyGrid local_map;
            local_map.header.frame_id = "base_link";
            local_map.header.stamp = ros::Time::now();
            local_map.info.resolution = local_resolution;
            local_map.info.width = width_cells;
            local_map.info.height = height_cells;
            local_map.info.origin.position.x = -(width_cells * local_resolution) / 2.0;
            local_map.info.origin.position.y = -(height_cells * local_resolution) / 2.0;
            local_map.info.origin.position.z = 0.0;
            local_map.info.origin.orientation.w = 1.0;

            local_map.data.resize(width_cells * height_cells, -1);

            for (int y = 0; y < height_cells; ++y) {
                for (int x = 0; x < width_cells; ++x) {
                    double dx = (x - width_cells / 2) * local_resolution;
                    double dy = (y - height_cells / 2) * local_resolution;
                    double rotated_dx = dx * cos(yaw) - dy * sin(yaw);
                    double rotated_dy = dx * sin(yaw) + dy * cos(yaw);
                    int global_x = static_cast<int>((robot_x + rotated_dx - map_origin_x) / global_resolution);
                    int global_y = static_cast<int>((robot_y + rotated_dy - map_origin_y) / global_resolution);

                    if (global_x >= 0 && global_x < map_width && global_y >= 0 && global_y < map_height) {
                        int global_index = global_y * map_width + global_x;
                        int local_index = y * width_cells + x;
                        local_map.data[local_index] = global_map.data[global_index];
                    }
                }
            }

            local_map_pub.publish(local_map);
        }
    }

    ros::Subscriber global_map_sub;
    ros::Subscriber pose_sub;
    ros::Publisher local_map_pub;

    nav_msgs::OccupancyGrid global_map;
    geometry_msgs::PoseStamped current_pose;
    double global_resolution;
    int width_cells, height_cells;

    std::string global_map_topic, local_map_topic, pose_topic;
    double width_meters, height_meters, local_resolution;
    int publish_rate;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "local_map_node");
    LocalMapNode localMapNode;
    return 0;
}
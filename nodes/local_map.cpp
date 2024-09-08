#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <cmath>

class LocalMapInMapFrameNode {
public:
    LocalMapInMapFrameNode() {
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");

        // Parameters
        private_nh.param("global_map_topic", global_map_topic, std::string("/map"));
        private_nh.param("local_map_topic", local_map_topic, std::string("local_map_in_map_frame"));
        private_nh.param("pose_topic", pose_topic, std::string("/vrpn_client_node/P1/pose"));
        private_nh.param("width_meters", width_meters, 2.0);
        private_nh.param("height_meters", height_meters, 2.0);
        private_nh.param("local_resolution", local_resolution, 0.01);
        private_nh.param("publish_rate", publish_rate, 10);
        private_nh.param("robot_frame", robot_frame, std::string("base_link"));
        private_nh.param("map_frame", map_frame, std::string("map_frame_for_local_rrt"));
        private_nh.param("control_point_offset", control_point_offset, 0.15);

        // Subscribers and Publishers
        global_map_sub = nh.subscribe(global_map_topic, 1, &LocalMapInMapFrameNode::globalMapCallback, this);
        pose_sub = nh.subscribe(pose_topic, 1, &LocalMapInMapFrameNode::poseCallback, this);
        local_map_pub = nh.advertise<nav_msgs::OccupancyGrid>(local_map_topic, 10);

        // Initialize the tf listener
        tf_listener = new tf::TransformListener();

        // Main loop
        ros::Rate rate(publish_rate);
        while (ros::ok()) {
            publishLocalMapInMapFrame();
            ros::spinOnce();
            rate.sleep();
        }
    }

    ~LocalMapInMapFrameNode() {
        delete tf_listener;
    }

private:
    ros::Subscriber global_map_sub;
    ros::Subscriber pose_sub;
    ros::Publisher local_map_pub;

    nav_msgs::OccupancyGrid global_map;
    geometry_msgs::PoseStamped current_pose;
    double global_resolution, control_point_offset;
    int width_cells, height_cells;

    std::string global_map_topic, local_map_topic, pose_topic, robot_frame, map_frame;
    double width_meters, height_meters, local_resolution;
    int publish_rate;

    tf::TransformListener* tf_listener;

    void globalMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        global_map = *msg;
        global_resolution = msg->info.resolution;
        width_cells = static_cast<int>(width_meters / local_resolution);
        height_cells = static_cast<int>(height_meters / local_resolution);
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_pose = *msg;
    }

    void publishLocalMapInMapFrame() {
        if (!global_map.data.empty() && current_pose.header.stamp != ros::Time(0)) {
            // Get the transform from the robot frame to the map frame
            tf::StampedTransform transform;
            try {
                tf_listener->lookupTransform(map_frame, robot_frame, ros::Time(0), transform);
            } catch (tf::TransformException& ex) {
                ROS_WARN("%s", ex.what());
                return;
            }

            double robot_x_map_frame = transform.getOrigin().x();
            double robot_y_map_frame = transform.getOrigin().y();

            double roll, pitch, yaw;
            transform.getBasis().getRPY(roll, pitch, yaw);

            // Offset in front of the robot (0.15 meters in the direction of yaw)
            double offset_x = control_point_offset * cos(yaw);
            double offset_y = control_point_offset * sin(yaw);

            // Adjust the position of the local map center
            robot_x_map_frame += offset_x;
            robot_y_map_frame += offset_y;

            double map_origin_x = global_map.info.origin.position.x;
            double map_origin_y = global_map.info.origin.position.y;
            int map_width = global_map.info.width;
            int map_height = global_map.info.height;

            nav_msgs::OccupancyGrid local_map;
            local_map.header.frame_id = map_frame;
            local_map.header.stamp = ros::Time::now();
            local_map.info.resolution = local_resolution;
            local_map.info.width = width_cells;
            local_map.info.height = height_cells;
            local_map.info.origin.position.x = robot_x_map_frame - (width_cells * local_resolution) / 2.0;
            local_map.info.origin.position.y = robot_y_map_frame - (height_cells * local_resolution) / 2.0;
            local_map.info.origin.position.z = 0.0;
            local_map.info.origin.orientation.w = 1.0;

            local_map.data.resize(width_cells * height_cells, -1);

            for (int y = 0; y < height_cells; ++y) {
                for (int x = 0; x < width_cells; ++x) {
                    double dx = (x - width_cells / 2) * local_resolution;
                    double dy = (y - height_cells / 2) * local_resolution;

                    int global_x = static_cast<int>((robot_x_map_frame + dx - map_origin_x) / global_resolution);
                    int global_y = static_cast<int>((robot_y_map_frame + dy - map_origin_y) / global_resolution);

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
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "local_map_in_map_frame_node");
    LocalMapInMapFrameNode localMapInMapFrameNode;
    return 0;
}
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>

#include "rrt_simple_source.hpp"

class MapPathSubscriber {
public:
    MapPathSubscriber() : tf_listener_(tf_buffer_), quad_tree_initialized_(false) {
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");

        private_nh.getParam("local_map_topic", local_map_topic_);
        private_nh.getParam("path_topic", path_topic_);
        private_nh.param("rate", rate_, 10);

        local_map_sub_ = nh.subscribe(local_map_topic_, 1, &MapPathSubscriber::mapCallback, this);
        path_sub_ = nh.subscribe(path_topic_, 1, &MapPathSubscriber::pathCallback, this);

        inside_pub_ = nh.advertise<nav_msgs::Path>("inside_path", 10);
        outside_pub_ = nh.advertise<nav_msgs::Path>("outside_path", 10);
        last_point_pub_ = nh.advertise<geometry_msgs::PointStamped>("last_point_inside_map", 10);
        static_map_client_ = nh.serviceClient<nav_msgs::GetMap>("/static_map");
    }

    bool fetchStaticMap() {
        nav_msgs::GetMap srv;
        if (static_map_client_.call(srv)) {
            global_map_ = srv.response.map;
            map_frame_id_ = srv.response.map.header.frame_id;
            ROS_INFO("Successfully called /static_map service");
            return true;
        } else {
            ROS_ERROR("Failed to call /static_map service");
            return false;
        }
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        map_ = *msg;
        map_frame_id_ = msg->header.frame_id;
    }

    void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
        path_ = *msg;
        path_number_ = msg->header.seq;
    }

    void run() {
        ros::Rate rate(rate_);
        while (ros::ok()) {
            ros::spinOnce();
            if (!map_.data.empty() && !path_.poses.empty()) {
                if (!quad_tree_initialized_) {
                    if (!fetchStaticMap() || global_map_.data.empty()) {
                        ROS_WARN("Global map is not yet available.");
                        ros::Duration(1.0).sleep();
                        continue;
                    }

                    QTree::Rectangle boundary(global_map_.info.width/2, global_map_.info.height/2, global_map_.info.width, global_map_.info.height);
                    if (global_map_.header.frame_id == path_.header.frame_id) {
                        int index = 0;
                        for (const auto& pose : path_.poses) {
                            // Pose on grig map
                            ROS_ERROR("Pose: %f, %f", pose.pose.position.x, pose.pose.position.y);
                            double x = (pose.pose.position.x - global_map_.info.origin.position.x) / global_map_.info.resolution;
                            double y = (pose.pose.position.y - global_map_.info.origin.position.y) / global_map_.info.resolution;

                            std::vector<double> position = {x, y};
                            motion_planning::Node* node = new motion_planning::Node(position);
                            if (!quad_tree_initialized_){
                                quad_tree_ = std::make_unique<QTree::QuadTree<motion_planning::Node>>(boundary, node, 4);
                                quad_tree_initialized_ = true;
                            } else {
                                quad_tree_->insert(node);
                            }
                            node->index = index;
                            index++;
                        }
                    } else {
                        ROS_ERROR("Global map and path frame ids do not match. Shutting down the node.");
                        ros::shutdown();
                    }
                }
                generateLocalRRT();
            }
            rate.sleep();
        }
    }

private:
    bool transformPoseToMapFrame(const geometry_msgs::PoseStamped& input_pose, geometry_msgs::PoseStamped& output_pose) {
        if (input_pose.header.frame_id == map_frame_id_) {
            output_pose = input_pose;
            return true;
        }

        geometry_msgs::TransformStamped transform;
        try {
            transform = tf_buffer_.lookupTransform(map_frame_id_, input_pose.header.frame_id, ros::Time(0), ros::Duration(3.0));
            tf2::doTransform(input_pose, output_pose, transform);
            return true;
        } catch (tf2::TransformException &ex) {
            ROS_ERROR("TF2 transform exception: %s", ex.what());
            return false;
        }
    }

    void updatePositionFromTF(const std::string& global_map_frame, const std::string& local_map_frame) {
        geometry_msgs::TransformStamped transform_stamped;
        try {
            transform_stamped = tf_buffer_.lookupTransform(global_map_frame, local_map_frame, ros::Time(0), ros::Duration(1.0));
            robot_position = {transform_stamped.transform.translation.x, transform_stamped.transform.translation.y};
            ROS_INFO("Updated start position from TF (x: %f, y: %f)", robot_position[0], robot_position[1]);
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }
    }

    void generateLocalRRT() {
        updatePositionFromTF(global_map_.header.frame_id, map_frame_id_);
        // ROS_ERROR("Robot position: %f, %f", robot_position[0], robot_position[1]);

        // Transforming path to map frame
        robot_position[0] = (robot_position[0] - global_map_.info.origin.position.x) / global_map_.info.resolution;
        robot_position[1] = (robot_position[1] - global_map_.info.origin.position.y) / global_map_.info.resolution;

        motion_planning::Node start(robot_position);
        motion_planning::Node* closest_path_point = quad_tree_->nearest_neighbor(&start);

        closest_path_point->position[0] = closest_path_point->position[0] * global_map_.info.resolution + global_map_.info.origin.position.x;
        closest_path_point->position[1] = closest_path_point->position[1] * global_map_.info.resolution + global_map_.info.origin.position.y;

        geometry_msgs::PointStamped last_point_inside_map;
        last_point_inside_map.header.frame_id = global_map_.header.frame_id;
        last_point_inside_map.header.stamp = ros::Time::now();
        last_point_inside_map.point.x = closest_path_point->position[0];
        last_point_inside_map.point.y = closest_path_point->position[1];

        if (!last_point_inside_map.header.frame_id.empty()) {
            last_point_pub_.publish(last_point_inside_map);
        }
    }

    int rate_;
    std::string local_map_topic_;
    std::string path_topic_;
    int path_number_;
    std::string map_frame_id_;
    std::vector<double> robot_position;

    ros::Subscriber local_map_sub_;
    ros::Subscriber path_sub_;
    ros::Publisher inside_pub_;
    ros::Publisher outside_pub_;
    ros::Publisher last_point_pub_;
    ros::ServiceClient static_map_client_;

    nav_msgs::OccupancyGrid map_;
    nav_msgs::OccupancyGrid global_map_;
    nav_msgs::Path path_;
    std::unique_ptr<QTree::QuadTree<motion_planning::Node>> quad_tree_;
    bool quad_tree_initialized_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_path_subscriber");
    MapPathSubscriber mps;
    mps.run();
    return 0;
}
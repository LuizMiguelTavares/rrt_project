#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <algorithm>

class MapMerger {
public:
    MapMerger() : tf_listener_(tf_buffer_) {
        ros::NodeHandle private_nh("~");

        private_nh.param<std::string>("local_map_topic", local_map_topic_, "local_map");
        private_nh.param<std::string>("global_map_service", global_map_service_, "/updated_map");
        private_nh.param<std::string>("merged_map_service", merged_map_service_, "/merged_map");
        
        local_map_sub_ = nh_.subscribe(local_map_topic_, 1, &MapMerger::localMapCallback, this);
        merged_map_service_server_ = nh_.advertiseService(merged_map_service_, &MapMerger::handleMergedMap, this);

        global_map_client_ = nh_.serviceClient<nav_msgs::GetMap>(global_map_service_);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber local_map_sub_;
    ros::ServiceServer merged_map_service_server_;
    ros::ServiceClient global_map_client_;
    nav_msgs::OccupancyGrid local_map_;
    nav_msgs::OccupancyGrid global_map_;
    nav_msgs::OccupancyGrid merged_map_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::string local_map_topic_;
    std::string global_map_service_;
    std::string merged_map_service_;

    void localMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        local_map_ = *msg;
    }

    bool handleMergedMap(nav_msgs::GetMap::Request& req, nav_msgs::GetMap::Response& res) {
        if (!fetchGlobalMap()) {
            ROS_ERROR("Failed to fetch the global map.");
            return false;
        }
        
        if (local_map_.data.empty()) {
            ROS_WARN("Local map is not available.");
            return false;
        }

        if (!mergeMaps()) {
            ROS_ERROR("Failed to merge maps.");
            return false;
        }

        res.map = merged_map_;
        return true;
    }

    bool fetchGlobalMap() {
        nav_msgs::GetMap srv;
        if (global_map_client_.call(srv)) {
            global_map_ = srv.response.map;
            // ROS_INFO("Fetched global map with width: %d, height: %d", global_map_.info.width, global_map_.info.height);
            return true;
        } else {
            ROS_ERROR("Failed to call service %s", global_map_service_.c_str());
            return false;
        }
    }

    bool mergeMaps() {
        merged_map_ = local_map_;
        merged_map_.header.stamp = ros::Time::now();

        geometry_msgs::TransformStamped transform;
        try {
            transform = tf_buffer_.lookupTransform(local_map_.header.frame_id, global_map_.header.frame_id, ros::Time(0), ros::Duration(1.0));
        } catch (tf2::TransformException &ex) {
            ROS_ERROR("Failed to get transform from %s to %s: %s", global_map_.header.frame_id.c_str(), local_map_.header.frame_id.c_str(), ex.what());
            return false;
        }

        for (int y = 0; y < global_map_.info.height; ++y) {
            for (int x = 0; x < global_map_.info.width; ++x) {
                int global_idx = y * global_map_.info.width + x;
                int global_value = global_map_.data[global_idx];

                if (global_value == -1) continue;

                double global_x = x * global_map_.info.resolution + global_map_.info.origin.position.x;
                double global_y = y * global_map_.info.resolution + global_map_.info.origin.position.y;

                geometry_msgs::PointStamped global_point;
                global_point.header.frame_id = global_map_.header.frame_id;
                global_point.header.stamp = ros::Time(0);
                global_point.point.x = global_x;
                global_point.point.y = global_y;
                global_point.point.z = 0;

                geometry_msgs::PointStamped local_point;
                try {
                    tf2::doTransform(global_point, local_point, transform);
                } catch (tf2::TransformException &ex) {
                    ROS_WARN("Failed to transform point: %s", ex.what());
                    continue;
                }

                int local_x = static_cast<int>((local_point.point.x - local_map_.info.origin.position.x) / local_map_.info.resolution);
                int local_y = static_cast<int>((local_point.point.y - local_map_.info.origin.position.y) / local_map_.info.resolution);

                if (isInMap(local_x, local_y)) {
                    int local_idx = local_y * local_map_.info.width + local_x;
                    int local_value = local_map_.data[local_idx];
                    

                    if (local_value == 0) {
                        merged_map_.data[local_idx] = 0;
                    } else if (local_value == 100) {
                        merged_map_.data[local_idx] = 100;
                    } else if (global_value == 100) {
                        merged_map_.data[local_idx] = 100;
                    } else if (global_value == 0 || local_value == 0) {
                        merged_map_.data[local_idx] = 0;
                    }
                }
            }
        }

        return true;
    }

    bool isInMap(int x, int y) {
        return x >= 0 && x < local_map_.info.width && y >= 0 && y < local_map_.info.height;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_merger");
    MapMerger merger;
    ros::spin();
    return 0;
}
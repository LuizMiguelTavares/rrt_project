#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>

class LocalMapGenerator {
public:
    LocalMapGenerator(ros::NodeHandle& nh)
        : nh_(nh),
          tf_listener_() {

        ros::NodeHandle private_nh("~");

        private_nh.param<std::string>("local_map_frame", local_map_frame_, "laser");
        nh_.param<std::string>("global_map_topic_1", global_map_topic_1_, "/map_topic_for_local_rrt");
        nh_.param<std::string>("global_map_topic_2", global_map_topic_2_, "/local_map_new");
        nh_.param<std::string>("local_map_topic_1", local_map_topic_, "/local_map");
        nh_.param<double>("local_map_size", local_map_size_, 1.5);
        nh_.param<double>("local_map_resolution", local_map_resolution_, 0.02);
        nh_.param<double>("publish_frequency", publish_frequency_, 10.0);

        global_map_sub_1_ = nh_.subscribe<nav_msgs::OccupancyGrid>(global_map_topic_1_, 1, &LocalMapGenerator::globalMapCallback1, this);
        global_map_sub_2_ = nh_.subscribe<nav_msgs::OccupancyGrid>(global_map_topic_2_, 1, &LocalMapGenerator::globalMapCallback2, this);

        local_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(local_map_topic_, 1);

        publish_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_frequency_), &LocalMapGenerator::publishLocalMaps, this);
    }

    void globalMapCallback1(const nav_msgs::OccupancyGrid::ConstPtr& global_map) {
        global_map_1_ = *global_map;
    }

    void globalMapCallback2(const nav_msgs::OccupancyGrid::ConstPtr& global_map) {
        global_map_2_ = *global_map;
    }

    void publishLocalMaps(const ros::TimerEvent&) {
        if (!global_map_1_.data.empty()) {
            if(!createLocalMap(global_map_1_, local_map_1_)){
                return;
            }
        }
        if (!global_map_2_.data.empty()) {
            if(!createLocalMap(global_map_2_, local_map_2_)){
                return;
            }
        }

        local_map_.data.assign(local_map_1_.data.size(), -1);

        if (!local_map_1_.data.empty() && !local_map_2_.data.empty()) {
            
            for (int i = 0; i < local_map_1_.data.size(); i++) {
                if (local_map_1_.data[i] > 0 || local_map_2_.data[i] > 0) {
                    local_map_.data[i] = 100;
                } else if (local_map_1_.data[i] == -1 || local_map_2_.data[i] == -1) {
                    local_map_.data[i] = -1;
                } else {
                    local_map_.data[i] = 0;
                }
            }
            
            local_map_.header.stamp = ros::Time::now();
            local_map_.header.frame_id = local_map_frame_;
            local_map_.info.resolution = local_map_resolution_;
            local_map_.info.width = local_map_1_.info.width;
            local_map_.info.height = local_map_1_.info.height;
            local_map_.info.origin.position.x = local_map_1_.info.origin.position.x;
            local_map_.info.origin.position.y = local_map_1_.info.origin.position.y;
            local_map_.info.origin.position.z = local_map_1_.info.origin.position.z;

            local_map_pub_.publish(local_map_);
        }
    }

private:
    bool createLocalMap(const nav_msgs::OccupancyGrid& global_map, nav_msgs::OccupancyGrid& local_map_now) {
        int local_width = local_map_size_ / local_map_resolution_;
        double resolution = local_map_resolution_;

        nav_msgs::OccupancyGrid local_map;
        local_map.header.frame_id = local_map_frame_;
        local_map.info.resolution = local_map_resolution_;
        local_map.info.width = local_width;
        local_map.info.height = local_width;
        local_map.info.origin.position.x = -local_map_size_ / 2;
        local_map.info.origin.position.y = -local_map_size_ / 2;
        local_map.info.origin.position.z = 0;
        local_map.data.resize(local_width * local_width);

        tf::StampedTransform transform;
        try {
            tf_listener_.lookupTransform(global_map.header.frame_id, local_map_frame_, ros::Time(0), transform);
        } catch (tf::TransformException& ex) {
            ROS_WARN("%s", ex.what());
            return false;
        }

        for (int i = 0; i < local_width; ++i) {
            for (int j = 0; j < local_width; ++j) {
                double local_x = (i - local_width / 2) * resolution;
                double local_y = (j - local_width / 2) * resolution;

                tf::Vector3 local_point(local_x, local_y, 0.0);
                tf::Vector3 global_point = transform * local_point;

                int global_i = (global_point.x() - global_map.info.origin.position.x) / resolution;
                int global_j = (global_point.y() - global_map.info.origin.position.y) / resolution;

                if (global_i >= 0 && global_j >= 0 && global_i < global_map.info.width && global_j < global_map.info.height) {
                    int global_index = global_j * global_map.info.width + global_i;
                    local_map.data[j * local_width + i] = global_map.data[global_index];
                } else {
                    local_map.data[j * local_width + i] = -1;
                }
            }
        }

        local_map_now = local_map;
        return true;
    }

    ros::NodeHandle& nh_;
    ros::Subscriber global_map_sub_1_;
    ros::Subscriber global_map_sub_2_;
    ros::Publisher local_map_pub_;
    nav_msgs::OccupancyGrid local_map_1_;
    nav_msgs::OccupancyGrid local_map_2_;
    nav_msgs::OccupancyGrid local_map_;
    ros::Timer publish_timer_;
    nav_msgs::OccupancyGrid global_map_1_; 
    nav_msgs::OccupancyGrid global_map_2_;  
    std::string local_map_frame_;

    tf::TransformListener tf_listener_;
    std::string global_map_topic_1_;
    std::string global_map_topic_2_;
    std::string local_map_topic_;
    double local_map_size_, local_map_resolution_;
    double publish_frequency_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "local_map_generator");

    ros::NodeHandle nh("~");

    LocalMapGenerator local_map_gen(nh);

    ros::spin();

    return 0;
}
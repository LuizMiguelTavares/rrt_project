#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point32.h>
#include <math.h>
#include <vector>
#include <thread>
#include <mutex>
#include "nanoflann.hpp"

// Point Cloud class definition for nanoflann
struct PointCloud {
    struct Point {
        float x, y;
    };

    std::vector<Point> pts;

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return pts.size(); }

    // Returns the dim'th component of the idx'th point in the class
    inline float kdtree_get_pt(const size_t idx, const size_t dim) const {
        if (dim == 0) return pts[idx].x;
        else return pts[idx].y;
    }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /*bb*/) const { return false; }
};

class ObstacleAvoidance {
public:
    ObstacleAvoidance() : latest_scan(nullptr), running(true) {
        // Initialize the ROS node
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");

        // Get parameters from private namespace
        private_nh.param("robot_density", robot_density_, 100);
        private_nh.param("robot_height", robot_height_, 1.0);
        private_nh.param("robot_width", robot_width_, 1.0);
        private_nh.param("num_clusters", num_clusters_, 6);
        private_nh.param("distance_threshold", distance_threshold_, 0.5);

        ROS_INFO("Robot parameters: density=%d, height=%f, width=%f, clusters=%d", robot_density_, robot_height_, robot_width_, num_clusters_);

        // Compute the discretized ellipse points
        double theta = 0.0;
        double step = 2 * M_PI / robot_density_;
        for (int i = 0; i < robot_density_; i++, theta += step) {
            geometry_msgs::Point32 point;
            point.x = (robot_height_ / 2) * cos(theta) - 0.1;  // Translate by -10 cm in x direction
            point.y = (robot_width_ / 2) * sin(theta);
            point.z = 0;
            base_points_.push_back(point);
        }

        ROS_INFO("Computed %lu base points for the robot.", base_points_.size());

        // Subscribe to the LiDAR topic
        lidar_sub = nh.subscribe("/laser/scan", 10, &ObstacleAvoidance::lidarCallback, this);

        // Publishers for visualization
        quadrants_pub = nh.advertise<visualization_msgs::MarkerArray>("/quadrants", 10);
        closest_points_pub = nh.advertise<visualization_msgs::MarkerArray>("/closest_points", 10);
        base_points_pub = nh.advertise<visualization_msgs::MarkerArray>("/base_points", 10);

        // Start the processing thread
        processing_thread = std::thread(&ObstacleAvoidance::processLoop, this);
    }

    ~ObstacleAvoidance() {
        running = false;
        if (processing_thread.joinable()) {
            processing_thread.join();
        }
    }

    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(scan_mutex);
        latest_scan = msg;
    }

    void processLoop() {
        ros::Rate rate(10); // 10 Hz
        while (running && ros::ok()) {
            sensor_msgs::LaserScan::ConstPtr scan;
            {
                std::lock_guard<std::mutex> lock(scan_mutex);
                scan = latest_scan;
            }
            if (scan) {
                processScan(scan);
            }
            rate.sleep();
        }
    }

    void processScan(const sensor_msgs::LaserScan::ConstPtr& msg) {
        // Number of LiDAR points
        int n_points = msg->ranges.size();

        // Number of points per cluster
        int points_per_cluster = n_points / num_clusters_;

        // Calculate the offset to start the first cluster so that 0 degrees is in the middle of a cluster
        int offset = points_per_cluster / 2;

        // Reset the closest points list
        closest_points.clear();

        // Marker array for clusters and closest points
        visualization_msgs::MarkerArray clusters_marker_array;
        visualization_msgs::MarkerArray closest_points_marker_array;

        // Colors for clusters
        std::vector<std::vector<float>> colors = {
            {1.0, 0.0, 0.0},  // Red
            {0.0, 1.0, 0.0},  // Green
            {0.0, 0.0, 1.0},  // Blue
            {1.0, 1.0, 0.0},  // Yellow
            {1.0, 0.0, 1.0},  // Magenta
            {0.0, 1.0, 1.0}   // Cyan
            // Add more colors if needed
        };

        // Process each cluster
        for (int i = 0; i < num_clusters_; i++) {
            int start_index = (i * points_per_cluster + offset) % n_points;
            int end_index = (start_index + points_per_cluster) % n_points;

            // Get the points in the current cluster
            std::vector<float> cluster_points;
            if (start_index < end_index) {
                cluster_points = std::vector<float>(msg->ranges.begin() + start_index, msg->ranges.begin() + end_index);
            } else {
                cluster_points = std::vector<float>(msg->ranges.begin() + start_index, msg->ranges.end());
                cluster_points.insert(cluster_points.end(), msg->ranges.begin(), msg->ranges.begin() + end_index);
            }

            // Find the closest point in the current cluster
            auto closest_point = *std::min_element(cluster_points.begin(), cluster_points.end());
            int closest_index = std::distance(cluster_points.begin(), std::find(cluster_points.begin(), cluster_points.end(), closest_point));
            closest_index = (start_index + closest_index) % n_points;

            // Convert polar coordinates (range, angle) to Cartesian coordinates (x, y)
            float angle = msg->angle_min + closest_index * msg->angle_increment;
            float x = closest_point * cos(angle);
            float y = closest_point * sin(angle);

            // Store the closest point in Cartesian coordinates
            closest_points.push_back(std::make_pair(x, y));

            // Create a marker for the closest point
            visualization_msgs::Marker closest_point_marker;
            closest_point_marker.header.frame_id = msg->header.frame_id; // Use the LiDAR frame
            closest_point_marker.header.stamp = ros::Time::now();
            closest_point_marker.ns = "closest_points";
            closest_point_marker.id = i;
            closest_point_marker.type = visualization_msgs::Marker::SPHERE;
            closest_point_marker.action = visualization_msgs::Marker::ADD;
            closest_point_marker.pose.position.x = x;
            closest_point_marker.pose.position.y = y;
            closest_point_marker.pose.position.z = 0;
            closest_point_marker.pose.orientation.x = 0.0;
            closest_point_marker.pose.orientation.y = 0.0;
            closest_point_marker.pose.orientation.z = 0.0;
            closest_point_marker.pose.orientation.w = 1.0;
            closest_point_marker.scale.x = 0.1;
            closest_point_marker.scale.y = 0.1;
            closest_point_marker.scale.z = 0.1;
            closest_point_marker.color.a = 1.0;  // Alpha
            closest_point_marker.color.r = 1.0;
            closest_point_marker.color.g = 1.0;
            closest_point_marker.color.b = 1.0;

            closest_points_marker_array.markers.push_back(closest_point_marker);

            // Create markers for the cluster points
            for (int j = start_index; j != end_index; j = (j + 1) % n_points) {
                float point_range = msg->ranges[j];
                if (point_range < msg->range_max && point_range > msg->range_min) {
                    float angle = msg->angle_min + j * msg->angle_increment;
                    float x = point_range * cos(angle);
                    float y = point_range * sin(angle);

                    // Filter out points inside the ellipse
                    if (isPointInsideEllipse(x, y)) {
                        continue;
                    }

                    visualization_msgs::Marker cluster_point_marker;
                    cluster_point_marker.header.frame_id = msg->header.frame_id; // Use the LiDAR frame
                    cluster_point_marker.header.stamp = ros::Time::now();
                    cluster_point_marker.ns = "cluster_points";
                    cluster_point_marker.id = j + i * points_per_cluster;
                    cluster_point_marker.type = visualization_msgs::Marker::SPHERE;
                    cluster_point_marker.action = visualization_msgs::Marker::ADD;
                    cluster_point_marker.pose.position.x = x;
                    cluster_point_marker.pose.position.y = y;
                    cluster_point_marker.pose.position.z = 0;
                    cluster_point_marker.pose.orientation.x = 0.0;
                    cluster_point_marker.pose.orientation.y = 0.0;
                    cluster_point_marker.pose.orientation.z = 0.0;
                    cluster_point_marker.pose.orientation.w = 1.0;
                    cluster_point_marker.scale.x = 0.02;
                    cluster_point_marker.scale.y = 0.02;
                    cluster_point_marker.scale.z = 0.02;
                    cluster_point_marker.color.a = 1.0;  // Alpha
                    cluster_point_marker.color.r = colors[i % colors.size()][0];
                    cluster_point_marker.color.g = colors[i % colors.size()][1];
                    cluster_point_marker.color.b = colors[i % colors.size()][2];

                    clusters_marker_array.markers.push_back(cluster_point_marker);
                }
            }
        }

        // Publish the markers
        quadrants_pub.publish(clusters_marker_array);
        closest_points_pub.publish(closest_points_marker_array);

        // Filter and publish cleaned closest points
        filterClosestPoints(msg->header.frame_id);
    }

    bool isPointInsideEllipse(float x, float y) {
        // Translate the point by +10 cm in x direction (reverse the -10 cm translation)
        float translated_x = x + 0.1;

        // Check if the point is inside the ellipse
        double normalized_x = translated_x / (robot_height_ / 2);
        double normalized_y = y / (robot_width_ / 2);
        return (normalized_x * normalized_x + normalized_y * normalized_y) <= 1.0;
    }

    void filterClosestPoints(const std::string& frame_id) {
        if (closest_points.empty()) return;

        // Build point cloud for KDTree
        PointCloud cloud;
        for (const auto& pt : closest_points) {
            PointCloud::Point point;
            point.x = pt.first;
            point.y = pt.second;
            cloud.pts.push_back(point);
        }

        // Construct a kd-tree index
        typedef nanoflann::KDTreeSingleIndexAdaptor<
            nanoflann::L2_Simple_Adaptor<float, PointCloud>,
            PointCloud,
            2 /* dimension */
        > my_kd_tree_t;

        my_kd_tree_t index(2 /*dim*/, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10 /* max leaf */));
        index.buildIndex();

        std::vector<bool> filtered(closest_points.size(), false);
        std::vector<std::pair<float, float>> cleaned_points;

        for (size_t i = 0; i < closest_points.size(); ++i) {
            if (filtered[i]) continue;

            const auto& pt = closest_points[i];
            cleaned_points.push_back(pt);
            filtered[i] = true;

            // Query the KDTree for neighbors within distance threshold
            const float query_pt[2] = { pt.first, pt.second };
            std::vector<nanoflann::ResultItem<unsigned int, float>> ret_matches;
            nanoflann::SearchParameters params;
            size_t nMatches = index.radiusSearch(query_pt, distance_threshold_ * distance_threshold_, ret_matches, params);

            for (const auto& match : ret_matches) {
                if (match.first != i) {
                    filtered[match.first] = true;
                }
            }
        }

        // Create markers for cleaned closest points
        visualization_msgs::MarkerArray cleaned_points_marker_array;
        for (size_t i = 0; i < cleaned_points.size(); ++i) {
            const auto& pt = cleaned_points[i];
            visualization_msgs::Marker cleaned_point_marker;
            cleaned_point_marker.header.frame_id = frame_id; // Use the LiDAR frame
            cleaned_point_marker.header.stamp = ros::Time::now();
            cleaned_point_marker.ns = "cleaned_closest_points";
            cleaned_point_marker.id = i;
            cleaned_point_marker.type = visualization_msgs::Marker::SPHERE;
            cleaned_point_marker.action = visualization_msgs::Marker::ADD;
            cleaned_point_marker.pose.position.x = pt.first;
            cleaned_point_marker.pose.position.y = pt.second;
            cleaned_point_marker.pose.position.z = 0;
            cleaned_point_marker.pose.orientation.x = 0.0;
            cleaned_point_marker.pose.orientation.y = 0.0;
            cleaned_point_marker.pose.orientation.z = 0.0;
            cleaned_point_marker.pose.orientation.w = 1.0;
            cleaned_point_marker.scale.x = 0.1;
            cleaned_point_marker.scale.y = 0.1;
            cleaned_point_marker.scale.z = 0.1;
            cleaned_point_marker.color.a = 1.0;  // Alpha
            cleaned_point_marker.color.r = 0.0;
            cleaned_point_marker.color.g = 1.0;
            cleaned_point_marker.color.b = 0.0;

            cleaned_points_marker_array.markers.push_back(cleaned_point_marker);
        }

        // Publish the cleaned closest points
        closest_points_pub.publish(cleaned_points_marker_array);
    }

    void publishBasePoints(const std::string& frame_id) {
        visualization_msgs::MarkerArray base_points_marker_array;

        for (size_t i = 0; i < base_points_.size(); ++i) {
            const auto& point = base_points_[i];

            visualization_msgs::Marker base_point_marker;
            base_point_marker.header.frame_id = frame_id;  // Use the same frame as the LiDAR data
            base_point_marker.header.stamp = ros::Time::now();
            base_point_marker.ns = "base_points";
            base_point_marker.id = i;
            base_point_marker.type = visualization_msgs::Marker::SPHERE;
            base_point_marker.action = visualization_msgs::Marker::ADD;
            base_point_marker.pose.position.x = point.x;
            base_point_marker.pose.position.y = point.y;
            base_point_marker.pose.position.z = point.z;
            base_point_marker.pose.orientation.x = 0.0;
            base_point_marker.pose.orientation.y = 0.0;
            base_point_marker.pose.orientation.z = 0.0;
            base_point_marker.pose.orientation.w = 1.0;
            base_point_marker.scale.x = 0.05;
            base_point_marker.scale.y = 0.05;
            base_point_marker.scale.z = 0.05;
            base_point_marker.color.a = 1.0;  // Alpha
            base_point_marker.color.r = 0.0;
            base_point_marker.color.g = 1.0;
            base_point_marker.color.b = 0.0;

            base_points_marker_array.markers.push_back(base_point_marker);
        }

        // Debugging information
        ROS_INFO("Publishing %lu base points.", base_points_marker_array.markers.size());

        base_points_pub.publish(base_points_marker_array);
    }

    void run() {
        ros::spin();
    }

private:
    ros::Subscriber lidar_sub;
    ros::Publisher quadrants_pub;
    ros::Publisher closest_points_pub;
    ros::Publisher base_points_pub;
    std::vector<std::pair<float, float>> closest_points;
    std::vector<geometry_msgs::Point32> base_points_;
    sensor_msgs::LaserScan::ConstPtr latest_scan;
    std::thread processing_thread;
    std::mutex scan_mutex;
    bool running;

    // Parameters
    int robot_density_;
    double robot_height_;
    double robot_width_;
    int num_clusters_;
    double distance_threshold_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "obstacle_avoidance");
    ObstacleAvoidance node;
    node.run();
    return 0;
}
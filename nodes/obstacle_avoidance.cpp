#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <math.h>
#include <vector>
#include <thread>
#include <mutex>
#include "nanoflann.hpp"
#include "potential.hpp"

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

typedef nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<float, PointCloud>,
    PointCloud,
    2 /* dimension */
> my_kd_tree_t;

class ObstacleAvoidance {
public:
    ObstacleAvoidance() : latest_scan(nullptr), running(true), max_filtered_points_(0), max_matched_base_points_(0), potential(2.0, 0.3, 0.3, 1.0) {
        // Initialize the ROS node
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");

        // Get parameters from private namespace
        private_nh.param("robot_density", robot_density_, 100);
        private_nh.param("robot_height", robot_height_, 0.47);
        private_nh.param("robot_width", robot_width_, 0.4);
        private_nh.param("num_clusters", num_clusters_, 8);
        private_nh.param("x_offset", x_offset_, 0.15);
        private_nh.param("min_dist_threshold", min_sqr_dist_threshold_, 0.4);
        min_sqr_dist_threshold_ = min_sqr_dist_threshold_ * min_sqr_dist_threshold_;
        private_nh.param("use_angle_filter", use_angle_filter_, false);

        ROS_INFO("Robot parameters: density=%d, height=%f, width=%f, clusters=%d", robot_density_, robot_height_, robot_width_, num_clusters_);

        // Get obstacle avoidance parameters
        double n, a, b, k;
        private_nh.param("obstacle_avoidance_n", n, 2.0);
        private_nh.param("obstacle_avoidance_a", a, 1.0);
        private_nh.param("obstacle_avoidance_b", b, 1.0);
        private_nh.param("obstacle_avoidance_k", k, 1.0);

        // Initialize potential class
        potential = potential::ObstacleAvoidance(n, a, b, k);

        // Compute the discretized ellipse points
        double theta = 0.0;
        double step = 2 * M_PI / robot_density_;
        for (int i = 0; i < robot_density_; i++, theta += step) {
            geometry_msgs::Point32 point;
            point.x = (robot_height_ / 2) * cos(theta) - x_offset_;  // Translate by -10 cm in x direction
            point.y = (robot_width_ / 2) * sin(theta);
            point.z = 0;
            base_points_.push_back(point);
        }

        ROS_INFO("Computed %lu base points for the robot.", base_points_.size());

        // Build KDTree for base points
        for (const auto& pt : base_points_) {
            PointCloud::Point point;
            point.x = pt.x;
            point.y = pt.y;
            base_cloud_.pts.push_back(point);
        }
        base_index_ = std::make_unique<my_kd_tree_t>(2 /*dim*/, base_cloud_, nanoflann::KDTreeSingleIndexAdaptorParams(10 /* max leaf */));
        base_index_->buildIndex();

        // Subscribe to the LiDAR topic
        lidar_sub = nh.subscribe("/laser/scan", 10, &ObstacleAvoidance::lidarCallback, this);

        // Publishers for visualization
        quadrants_pub = nh.advertise<sensor_msgs::PointCloud>("/quadrants", 10);
        closest_points_pub = nh.advertise<sensor_msgs::PointCloud>("/closest_points", 10);
        filtered_closest_points_pub = nh.advertise<sensor_msgs::PointCloud>("/filtered_closest_points", 10);
        base_points_pub = nh.advertise<sensor_msgs::PointCloud>("/base_points", 10);
        matched_base_points_pub = nh.advertise<sensor_msgs::PointCloud>("/matched_base_points", 10);

        // Publisher for arrows
        arrows_pub = nh.advertise<visualization_msgs::MarkerArray>("/potential_arrows", 10);

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
        ros::Rate rate(30);
        while (running && ros::ok()) {
            sensor_msgs::LaserScan::ConstPtr scan;
            {
                std::lock_guard<std::mutex> lock(scan_mutex);
                scan = latest_scan;
            }
            if (scan) {
                processScan(scan);
            }

            scan = nullptr;
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

            // Find the closest point in the current cluster
            float closest_point = std::numeric_limits<float>::max();
            int closest_index = -1;

            for (int j = start_index; j != end_index; j = (j + 1) % n_points) {
                float point_range = msg->ranges[j];
                if (point_range < closest_point && point_range >= msg->range_min && point_range <= msg->range_max) {
                    closest_point = point_range;
                    closest_index = j;
                }
            }

            if (closest_index == -1) {
                // Handle the case where no valid point was found
                ROS_WARN("No valid closest point found in the cluster");
                continue;
            }

            // Convert polar coordinates (range, angle) to Cartesian coordinates (x, y)
            float angle = msg->angle_min + closest_index * msg->angle_increment;
            float x = closest_point * cos(angle);
            float y = closest_point * sin(angle);

            // Store the closest point in Cartesian coordinates
            closest_points.push_back(std::make_pair(x, y));
        }

        // Publish the closest points
        publishPointCloud(closest_points_pub, closest_points, msg->header.frame_id);

        // Filter and publish cleaned closest points
        filterClosestPoints(msg->header.frame_id);

        calculatePotential(msg->header.frame_id);

        // Publish robot points
        publishBasePoints(msg->header.frame_id);
    }

    bool isPointInsideEllipse(float x, float y) {
        // Translate the point by +10 cm in x direction (reverse the -10 cm translation)
        float translated_x = x + x_offset_;

        // Check if the point is inside the ellipse
        double normalized_x = translated_x / (robot_height_ / 2);
        double normalized_y = y / (robot_width_ / 2);
        return (normalized_x * normalized_x + normalized_y * normalized_y) <= 1.0;
    }

    void filterClosestPoints(const std::string& frame_id) {
        if (closest_points.empty()) return;

        std::vector<float> distances(closest_points.size(), std::numeric_limits<float>::max());
        std::vector<std::pair<float, float>> closest_base_points;

        // Find the nearest base point for each closest point
        for (size_t i = 0; i < closest_points.size(); ++i) {
            const auto& pt = closest_points[i];

            // Query for the nearest base point
            const float query_pt[2] = { pt.first, pt.second };
            size_t num_results = 1;
            std::vector<size_t> ret_index(num_results);
            std::vector<float> out_dist_sqr(num_results);
            nanoflann::KNNResultSet<float> resultSet(num_results);
            resultSet.init(&ret_index[0], &out_dist_sqr[0]);
            base_index_->findNeighbors(resultSet, query_pt, nanoflann::SearchParameters());

            // Store the closest base point and its distance
            closest_base_points.push_back(std::make_pair(base_cloud_.pts[ret_index[0]].x, base_cloud_.pts[ret_index[0]].y));
            distances[i] = out_dist_sqr[0];
        }

        // Initialize candidate flags
        std::vector<bool> candidates(closest_points.size(), true);

        // Calculate squared distance between the first and last point
        float sqr_distance_between_first_and_last_point = (closest_points[0].first - closest_points.back().first) * (closest_points[0].first - closest_points.back().first) +
                                                          (closest_points[0].second - closest_points.back().second) * (closest_points[0].second - closest_points.back().second);

        int last_candidate_index = -1;
        for (size_t i = 0; i < closest_points.size(); ++i) {
            if (i == 0) {
                if (sqr_distance_between_first_and_last_point > min_sqr_dist_threshold_) {
                    last_candidate_index = i;
                } else {
                    if (distances[i] < distances.back()) {
                        candidates.back() = false;
                        last_candidate_index = i;
                    } else {
                        candidates[i] = false;
                        last_candidate_index = closest_points.size() - 1;
                    }
                }
                continue;
            }

            if (last_candidate_index == -1) {
                ROS_ERROR_STREAM("Obstacle avoidance node: last_candidate_index is -1");
                return;
            }

            if (i == closest_points.size() - 1) {
                break;
            }

            // Calculate squared distance to the last candidate
            float sqr_dist_last_candidate = (closest_points[last_candidate_index].first - closest_points[i].first) * (closest_points[last_candidate_index].first - closest_points[i].first) +
                                            (closest_points[last_candidate_index].second - closest_points[i].second) * (closest_points[last_candidate_index].second - closest_points[i].second);
            if (sqr_dist_last_candidate > min_sqr_dist_threshold_) {
                last_candidate_index = i;
            } else {
                if (distances[i] < distances[last_candidate_index]) {
                    candidates[last_candidate_index] = false;
                    last_candidate_index = i;
                } else {
                    candidates[i] = false;
                }
            }
        }

        // Collect cleaned points
        cleaned_points.clear();
        cleaned_points.reserve(closest_points.size());

        for (size_t i = 0; i < closest_points.size(); ++i) {
            if (candidates[i]) {
                cleaned_points.push_back(closest_points[i]);
                cleaned_closest_base_points.push_back(closest_base_points[i]);
            }
        }

        // Publish the cleaned closest points
        publishPointCloud(filtered_closest_points_pub, cleaned_points, frame_id);
    }

    void calculatePotential(const std::string& frame_id){
        std::pair<double, double> aux_x_dot_y_dot = {0.0, 0.0};
        std::pair<double, double> x_dot_y_dot = {0.0, 0.0};

        visualization_msgs::MarkerArray arrow_array;

        for (size_t i = 0; i < cleaned_points.size(); ++i) {
            const auto& pt = cleaned_points[i];
            const auto& base_pt = cleaned_closest_base_points[i];

            Eigen::Vector2d robot_point(base_pt.first, base_pt.second);
            Eigen::Vector2d obstacle_point(pt.first, pt.second);

            std::vector<Eigen::Vector2d> obstacle_points = {obstacle_point};

            aux_x_dot_y_dot = potential.obstacle_avoidance(robot_point, obstacle_points);

            if (use_angle_filter_) {
                if (base_pt.first > robot_width_) {
                    x_dot_y_dot.first += 0.0;
                } else {
                    x_dot_y_dot.first += aux_x_dot_y_dot.first;
                }
            } else {
                x_dot_y_dot.first += aux_x_dot_y_dot.first;
            }
            
            x_dot_y_dot.second += aux_x_dot_y_dot.second;

            // Create and add arrow marker for aux_x_dot_y_dot
            visualization_msgs::Marker arrow_marker;
            arrow_marker.header.frame_id = frame_id; // Update with your lidar frame
            arrow_marker.header.stamp = ros::Time::now();
            arrow_marker.ns = "potential_arrows";
            arrow_marker.id = i;
            arrow_marker.type = visualization_msgs::Marker::ARROW;
            arrow_marker.action = visualization_msgs::Marker::ADD;

            geometry_msgs::Point start, end;
            start.x = pt.first;
            start.y = pt.second;
            start.z = 0.0;

            end.x = start.x + aux_x_dot_y_dot.first;
            end.y = start.y + aux_x_dot_y_dot.second;
            end.z = 0.0;

            arrow_marker.points.push_back(start);
            arrow_marker.points.push_back(end);

            arrow_marker.scale.x = 0.02; // Shaft diameter
            arrow_marker.scale.y = 0.04; // Head diameter
            arrow_marker.scale.z = 0.1;  // Head length

            arrow_marker.color.r = 0.0;
            arrow_marker.color.g = 0.0;
            arrow_marker.color.b = 0.0;
            arrow_marker.color.a = 1.0; // Alpha

            arrow_array.markers.push_back(arrow_marker);
        }

        // Create and add arrow marker for final x_dot_y_dot
        visualization_msgs::Marker final_arrow_marker;
        final_arrow_marker.header.frame_id = frame_id; // Update with your lidar frame
        final_arrow_marker.header.stamp = ros::Time::now();
        final_arrow_marker.ns = "potential_arrows";
        final_arrow_marker.id = closest_points.size() + 1;
        final_arrow_marker.type = visualization_msgs::Marker::ARROW;
        final_arrow_marker.action = visualization_msgs::Marker::ADD;

        geometry_msgs::Point start, end;
        start.x = 0.0;
        start.y = 0.0;
        start.z = 0.0;

        end.x = start.x + x_dot_y_dot.first;
        end.y = start.y + x_dot_y_dot.second;
        end.z = 0.0;

        final_arrow_marker.points.push_back(start);
        final_arrow_marker.points.push_back(end);

        final_arrow_marker.scale.x = 0.05; // Shaft diameter
        final_arrow_marker.scale.y = 0.1;  // Head diameter
        final_arrow_marker.scale.z = 0.2;  // Head length

        final_arrow_marker.color.r = 1.0;
        final_arrow_marker.color.g = 0.0;
        final_arrow_marker.color.b = 0.0;
        final_arrow_marker.color.a = 1.0; // Alpha

        arrow_array.markers.push_back(final_arrow_marker);

        // Publish the marker array
        arrows_pub.publish(arrow_array);

        ROS_INFO_STREAM("x_dot: " << x_dot_y_dot.first << " y_dot: " << x_dot_y_dot.second);
    }

    void publishBasePoints(const std::string& frame_id) {
        publishPointCloud(base_points_pub, base_points_, frame_id);
    }

    void publishPointCloud(const ros::Publisher& pub, const std::vector<std::pair<float, float>>& points, const std::string& frame_id) {
        sensor_msgs::PointCloud point_cloud;
        point_cloud.header.frame_id = frame_id;
        point_cloud.header.stamp = ros::Time::now();

        for (const auto& pt : points) {
            geometry_msgs::Point32 point;
            point.x = pt.first;
            point.y = pt.second;
            point.z = 0;
            point_cloud.points.push_back(point);
        }

        pub.publish(point_cloud);
    }

    void publishPointCloud(const ros::Publisher& pub, const std::vector<geometry_msgs::Point32>& points, const std::string& frame_id) {
        sensor_msgs::PointCloud point_cloud;
        point_cloud.header.frame_id = frame_id;
        point_cloud.header.stamp = ros::Time::now();
        point_cloud.points = points;

        pub.publish(point_cloud);
    }

    void run() {
        ros::spin();
    }

private:
    ros::Subscriber lidar_sub;
    ros::Publisher quadrants_pub;
    ros::Publisher closest_points_pub;
    ros::Publisher filtered_closest_points_pub;
    ros::Publisher base_points_pub;
    ros::Publisher matched_base_points_pub;
    ros::Publisher arrows_pub;
    std::vector<std::pair<float, float>> closest_points;
    std::vector<geometry_msgs::Point32> base_points_;
    PointCloud base_cloud_;
    std::unique_ptr<my_kd_tree_t> base_index_;
    sensor_msgs::LaserScan::ConstPtr latest_scan;
    std::thread processing_thread;
    std::mutex scan_mutex;
    bool running;

    // Parameters
    int robot_density_;
    double robot_height_;
    double robot_width_;
    int num_clusters_;
    double x_offset_;
    double min_sqr_dist_threshold_;
    size_t max_filtered_points_;
    size_t max_matched_base_points_;

    std::vector<std::pair<float, float>> cleaned_points;
    std::vector<std::pair<float, float>> cleaned_closest_base_points;
    potential::ObstacleAvoidance potential;
    bool use_angle_filter_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "obstacle_avoidance");
    ObstacleAvoidance node;
    node.run();
    return 0;
}
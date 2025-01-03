#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <vector>
#include <thread>
#include <mutex>
#include "nanoflann.hpp"
#include "potential.hpp"
#include <filters/mean.hpp>
#include <filters/filter_chain.hpp>

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
    ObstacleAvoidance() : latest_scan(nullptr), running(true), max_filtered_points_(0), max_matched_base_points_(0), potential(2.0, 0.3, 0.3, 1.0),
                          filter_chain_x("double"), filter_chain_y("double") {
        // Initialize the ROS node
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");

        colors_ = {
            {1.0, 0.0, 0.0},  // Red
            {0.0, 1.0, 0.0},  // Green
            {0.0, 0.0, 1.0},  // Blue
            {1.0, 1.0, 0.0},  // Yellow
            {1.0, 0.0, 1.0},  // Magenta
            {0.0, 1.0, 1.0},  // Cyan
            {1.0, 0.0, 0.0},  // Red
            {0.0, 1.0, 0.0},  // Green
            {0.0, 0.0, 1.0},  // Blue
            {1.0, 1.0, 0.0},  // Yellow
            {1.0, 0.0, 1.0},  // Magenta
            {0.0, 1.0, 1.0}   // Cyan
        };

        // Get parameters from private namespace
        private_nh.param("robot_density", robot_density_, 100);
        private_nh.param("robot_height", robot_height_, 0.47);
        private_nh.param("robot_width", robot_width_, 0.4);
        private_nh.param("lidar_topic", lidar_topic_, std::string("/scan"));
        private_nh.param("num_clusters", num_clusters_, 8);
        private_nh.param("x_offset", x_offset_, 0.15);
        private_nh.param("min_dist_threshold", min_sqr_dist_threshold_, 0.4);
        min_sqr_dist_threshold_ = min_sqr_dist_threshold_ * min_sqr_dist_threshold_;
        private_nh.param("use_angle_filter", use_angle_filter_, false);
        private_nh.param("angle_filter_margin", angle_filter_margin_, 0.1);
        private_nh.param("potential_gain", potential_gain_, 1.0);
        private_nh.param("saturate_potential", saturate_potential_, 1.0);
        private_nh.param("filter_gain", filter_gain_, 0.7);
        private_nh.param("filter_type", filter_type_, 1); // 0: no filter, 1: first-order, 2: second-order
        ROS_INFO("Filter type: %d", filter_type_);
        ROS_INFO("Potential gain: %f", potential_gain_);
        ROS_INFO("Robot parameters: density=%d, height=%f, width=%f, clusters=%d", robot_density_, robot_height_, robot_width_, num_clusters_);

        private_nh.param("enable_min_threshold_for_obstacle_points", enable_min_threshold_for_obstacle_points_, false);
        private_nh.param("min_threshold_for_obstacle_points", min_threshold_for_obstacle_points_, 0.3);
        // Get obstacle avoidance parameters
        double n, a, b, k, lambda;
        private_nh.param("obstacle_avoidance_n", n, 2.0);
        private_nh.param("obstacle_avoidance_a", a, 0.1);
        private_nh.param("obstacle_avoidance_b", b, 0.1);
        private_nh.param("obstacle_avoidance_k", k, 1.0);
        private_nh.param("obstacle_avoidance_lambda", lambda, 0.01);

        // Initialize potential class
        potential = potential::ObstacleAvoidance(n, a, b, k, lambda);

        // Compute the discretized ellipse points
        double theta = 0.0;
        double step = 2 * M_PI / robot_density_;
        for (int i = 0; i < robot_density_; i++, theta += step) {
            geometry_msgs::Point32 point;
            point.x = (robot_height_ / 2) * cos(theta) - x_offset_;
            point.y = (robot_width_ / 2) * sin(theta);
            point.z = 0;
            base_points_.push_back(point);
        }

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
        lidar_sub = nh.subscribe(lidar_topic_, 1, &ObstacleAvoidance::lidarCallback, this);

        // Publishers for visualization
        quadrants_pub = nh.advertise<sensor_msgs::PointCloud>("/quadrants_2", 10);
        closest_points_pub = nh.advertise<sensor_msgs::PointCloud>("/closest_points_2", 10);
        filtered_closest_points_pub = nh.advertise<sensor_msgs::PointCloud>("/filtered_closest_points_2", 10);
        base_points_pub = nh.advertise<sensor_msgs::PointCloud>("/base_points_2", 10);
        matched_base_points_pub = nh.advertise<sensor_msgs::PointCloud>("/matched_base_points_2", 10);
        potential_pub = nh.advertise<geometry_msgs::Point>("potential_2", 10);

        // Publisher for arrows
        arrows_pub = nh.advertise<visualization_msgs::MarkerArray>("/potential_arrows_2", 10);
        text_markers_pub = nh.advertise<visualization_msgs::MarkerArray>("/closest_points_text_2", 10);
        quadrants_markers_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/quadrants_markers", 10);

        // Initialize the filter parameters
        initFilterParameters();

        // Initialize the filter chain
        filter_chain_x.configure("double", private_nh);
        filter_chain_y.configure("double", private_nh);
    }

    ~ObstacleAvoidance() {
        running = false;
    }

    void initFilterParameters() {
        X_dot_prev = {0.0, 0.0};
    }

    void applyFirstOrderFilter(const std::pair<double, double>& x_dot_y_dot, std::pair<double, double>& filtered_x_dot_y_dot) {
        double alpha = filter_gain_; // Filter coefficient (example value, adjust as needed)
        filtered_x_dot_y_dot.first = alpha * x_dot_y_dot.first + (1 - alpha) * X_dot_prev.first;
        filtered_x_dot_y_dot.second = alpha * x_dot_y_dot.second + (1 - alpha) * X_dot_prev.second;

        X_dot_prev = filtered_x_dot_y_dot;
    }

    void applySecondOrderFilter(const std::pair<double, double>& x_dot_y_dot, std::pair<double, double>& filtered_x_dot_y_dot) {
        double input_x = x_dot_y_dot.first;
        double input_y = x_dot_y_dot.second;

        // Apply the filter chain for x
        filter_chain_x.update(input_x, filtered_x_dot_y_dot.first);

        // Apply the filter chain for y
        filter_chain_y.update(input_y, filtered_x_dot_y_dot.second);
    }

    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        processScan(msg);
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

        std::vector<std::vector<geometry_msgs::Point>> quadrants_points(num_clusters_);

        // Process each cluster
        for (int i = 0; i < num_clusters_; i++) {
            int start_index = (i * points_per_cluster + offset) % n_points;
            int end_index = (start_index + points_per_cluster) % n_points;

            // Find the closest point in the current cluster
            float closest_point = std::numeric_limits<float>::max();
            int closest_index = -1;

            int j = start_index;
            do {
                float point_range = msg->ranges[j];

                if (point_range >= msg->range_min && point_range <= msg->range_max) {
                    float angle = msg->angle_min + j * msg->angle_increment;
                    float x = point_range * cos(angle);
                    float y = point_range * sin(angle);

                    geometry_msgs::Point point;
                    point.x = x;
                    point.y = y;
                    point.z = 0.0;
                    quadrants_points[i].push_back(point);

                    if (point_range < closest_point) {
                        closest_point = point_range;
                        closest_index = j;
                    }
                }

                j = (j + 1) % n_points;
            } while (j != end_index);

            if (closest_index == -1) {
                continue;
            }

            // Convert polar coordinates (range, angle) to Cartesian coordinates (x, y) for the closest point
            float angle = msg->angle_min + closest_index * msg->angle_increment;
            float x = closest_point * cos(angle);
            float y = closest_point * sin(angle);

            if (isPointInsideEllipse(x, y)) {
                ROS_ERROR("Collision detected at x=%f, y=%f", x, y);
                continue;
            }

            // Store the closest point in Cartesian coordinates
            closest_points.push_back(std::make_pair(x, y));
        }

        // Publish quadrants with all laser scan points
        publishQuadrants(quadrants_points, msg->header.frame_id);

        publishPointCloud(closest_points_pub, closest_points, msg->header.frame_id);

        publishPointCloudWithText(closest_points_pub, closest_points, msg->header.frame_id);

        filterClosestPoints(msg->header.frame_id);

        calculatePotential(msg->header.frame_id);

        publishBasePoints(msg->header.frame_id);
    }


    void publishQuadrants(const std::vector<std::vector<geometry_msgs::Point>>& quadrants_points, const std::string& frame_id) {
        visualization_msgs::MarkerArray marker_array;

        for (size_t i = 0; i < quadrants_points.size(); ++i) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = frame_id;
            marker.header.stamp = ros::Time::now();
            marker.ns = "quadrants";
            marker.id = i;
            marker.type = visualization_msgs::Marker::POINTS;
            marker.action = visualization_msgs::Marker::ADD;

            marker.points = quadrants_points[i];

            // Set the color for this quadrant
            marker.color.r = colors_[i % colors_.size()][0];
            marker.color.g = colors_[i % colors_.size()][1];
            marker.color.b = colors_[i % colors_.size()][2];
            marker.color.a = 1.0;

            // Set the scale of the points
            marker.scale.x = 0.02;
            marker.scale.y = 0.02;

            marker_array.markers.push_back(marker);
        }

        quadrants_markers_pub_.publish(marker_array);
    }

    bool isPointInsideEllipse(float x, float y) {
        float translated_x = x + x_offset_;

        double normalized_x = translated_x / (robot_height_ / 2);
        double normalized_y = y / (robot_width_ / 2);
        return (normalized_x * normalized_x + normalized_y * normalized_y) <= 1.0;
    }

    // Blended tanh function for smooth transition
    double blended_tanh(double x) {
        auto sigmoid = [](double x, double a = 2) {
            return 1 / (1 + std::exp(-a * x));
        };

        double blend = sigmoid(x);
        return ((1 - blend) * std::tanh(x * 1) + blend * std::tanh(x * 2) + 1) / 2;
    }

    float squared_distance(const std::pair<float, float>& p1, const std::pair<float, float>& p2) {
        return (p1.first - p2.first) * (p1.first - p2.first) + (p1.second - p2.second) * (p1.second - p2.second);
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

        // Find the index of the smallest element
        auto min_it = std::min_element(distances.begin(), distances.end());
        int min_index = std::distance(distances.begin(), min_it);

        // Rotate the array so that the smallest element is at the top
        std::rotate(distances.begin(), distances.begin() + min_index, distances.end());
        std::rotate(closest_points.begin(), closest_points.begin() + min_index, closest_points.end());
        std::rotate(closest_base_points.begin(), closest_base_points.begin() + min_index, closest_base_points.end());

        // Initialize candidate flags
        std::vector<bool> candidates(closest_points.size(), true);

        double sqr_distance_between_first_and_last_point = squared_distance(closest_points[0], closest_points.back());
        // std::cout << "Sqr Distance Between First and Last Point: " << sqr_distance_between_first_and_last_point << "\n";

        int last_candidate_index = 0;
        for (size_t i = 0; i < closest_points.size(); ++i) {
            if (i == 0) {
                if (sqr_distance_between_first_and_last_point < min_sqr_dist_threshold_) {
                    candidates.back() = false;
                } else {
                    last_candidate_index = 0;
                }
                
                continue;
            }std::vector<std::vector<geometry_msgs::Point>> quadrants_points(num_clusters_);

            if (candidates[i]) {
                double sqr_dist_last_candidate = squared_distance(closest_points[last_candidate_index], closest_points[i]);
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
        }

        // Compare last candidate with the first point
        double sqr_distance_between_last_candidate_and_first_point = squared_distance(closest_points[last_candidate_index], closest_points[0]);

        if (sqr_distance_between_last_candidate_and_first_point < min_sqr_dist_threshold_) {
            candidates[last_candidate_index] = false;
        }

        // Collect cleaned points
        cleaned_points.clear();
        cleaned_points.reserve(closest_points.size());
        cleaned_closest_base_points.clear();
        cleaned_closest_base_points.reserve(closest_points.size());

        for (size_t i = 0; i < closest_points.size(); ++i) {
            if (candidates[i]) {
                cleaned_points.push_back(closest_points[i]);
                cleaned_closest_base_points.push_back(closest_base_points[i]);
            }
        }

        publishPointCloud(filtered_closest_points_pub, cleaned_points, frame_id);
        publishPointCloud(matched_base_points_pub, cleaned_closest_base_points, frame_id);
    }

    void calculatePotential(const std::string& frame_id){
        std::pair<double, double> aux_x_dot_y_dot = {0.0, 0.0};
        std::pair<double, double> x_dot_y_dot = {0.0, 0.0};

        visualization_msgs::MarkerArray arrow_array;

        Eigen::Vector2d combined_gradient(0.0, 0.0);
        double combined_v = 0.0;
        for (size_t i = 0; i < cleaned_points.size(); ++i) {
            const auto& pt = cleaned_points[i];
            const auto& base_pt = cleaned_closest_base_points[i];

            Eigen::Vector2d robot_point(base_pt.first, base_pt.second);
            Eigen::Vector2d obstacle_point(pt.first, pt.second);

            if (use_angle_filter_) {
                if (std::abs(pt.second) <= robot_width_/2) {
                    aux_x_dot_y_dot = potential.obstacle_avoidance(robot_point, obstacle_point, 1.0);
                    x_dot_y_dot.first += aux_x_dot_y_dot.first;
                    
                } else if (std::abs(pt.second) < robot_width_/2 + angle_filter_margin_) {
                    double d = robot_width_/2 + angle_filter_margin_ - std::abs(pt.second);
                    double product = d / angle_filter_margin_;  // 0 - 1
                    double mapped_product = product * (1.5 + 3) - 3;
                    double value_gain = blended_tanh(mapped_product);
                    aux_x_dot_y_dot = potential.obstacle_avoidance(robot_point, obstacle_point, 1.0);
                    x_dot_y_dot.first += aux_x_dot_y_dot.first*value_gain;
                } else {
                    aux_x_dot_y_dot = potential.obstacle_avoidance(robot_point, obstacle_point, 1.0);
                    x_dot_y_dot.first += 0.0;
                }
            } else {
                aux_x_dot_y_dot = potential.obstacle_avoidance(robot_point, obstacle_point, 1.0);
                x_dot_y_dot.first += aux_x_dot_y_dot.first;
            }

            // x_dot_y_dot.first += aux_x_dot_y_dot.first;
            x_dot_y_dot.second += aux_x_dot_y_dot.second;

            combined_gradient += potential.get_J();
            combined_v += potential.get_v();

            // Create and add arrow marker for aux_x_dot_y_dot
            visualization_msgs::Marker arrow_marker;
            arrow_marker.header.frame_id = frame_id;
            arrow_marker.header.stamp = ros::Time::now();
            arrow_marker.ns = "potential_arrows";
            arrow_marker.id = i;
            arrow_marker.type = visualization_msgs::Marker::ARROW;
            arrow_marker.action = visualization_msgs::Marker::ADD;

            geometry_msgs::Point start, end;
            start.x = pt.first;
            start.y = pt.second;
            start.z = 0.0;

            if (use_angle_filter_) {
                if (std::abs(pt.second) <= robot_width_/2) {
                    end.x = start.x + aux_x_dot_y_dot.first;
                } else if (std::abs(pt.second) < robot_width_/2 + angle_filter_margin_) {
                    double d = robot_width_/2 + angle_filter_margin_ - std::abs(pt.second);
                    double product = d / angle_filter_margin_;  // 0 - 1
                    double mapped_product = product * (1.5 + 3) - 3;
                    double value_gain = blended_tanh(mapped_product);
                    end.x = start.x + aux_x_dot_y_dot.first*value_gain;
                } else {
                    end.x = start.x;
                }
            } else {
                end.x = start.x + aux_x_dot_y_dot.first;
            }

            end.y = start.y + aux_x_dot_y_dot.second;
            end.z = 0.0;

            arrow_marker.points.push_back(start);
            arrow_marker.points.push_back(end);

            arrow_marker.scale.x = 0.02; // Shaft diameter
            arrow_marker.scale.y = 0.04; // Head diameter
            arrow_marker.scale.z = 0.02;  // Head length

            arrow_marker.color.r = 0.0;
            arrow_marker.color.g = 0.0;
            arrow_marker.color.b = 0.0;
            arrow_marker.color.a = 1.0; // Alpha

            arrow_array.markers.push_back(arrow_marker);
        }

        for (size_t i = cleaned_points.size(); i < num_clusters_; ++i) {
            visualization_msgs::Marker delete_marker;
            delete_marker.header.frame_id = frame_id;
            delete_marker.header.stamp = ros::Time::now();
            delete_marker.ns = "potential_arrows";
            delete_marker.id = i;
            delete_marker.action = visualization_msgs::Marker::DELETE;

            arrow_array.markers.push_back(delete_marker);
        }

        // // Create and add arrow marker for final x_dot_y_dot
        std::pair<double, double> filtered_x_dot_y_dot = x_dot_y_dot;

        if (filter_type_ == 1) {
            applyFirstOrderFilter(x_dot_y_dot, filtered_x_dot_y_dot);
        } else if (filter_type_ == 2) {
            applySecondOrderFilter(x_dot_y_dot, filtered_x_dot_y_dot);
        }

        // visualization_msgs::Marker final_arrow_marker;
        // final_arrow_marker.header.frame_id = frame_id; 
        // final_arrow_marker.header.stamp = ros::Time::now();
        // final_arrow_marker.ns = "potential_arrows";
        // final_arrow_marker.id = closest_points.size() + 1;
        // final_arrow_marker.type = visualization_msgs::Marker::ARROW;
        // final_arrow_marker.action = visualization_msgs::Marker::ADD;

        // geometry_msgs::Point start, end;
        // start.x = 0.0;
        // start.y = 0.0;
        // start.z = 0.0;

        // end.x = start.x + filtered_x_dot_y_dot.first;
        // end.y = start.y + filtered_x_dot_y_dot.second;
        // end.z = 0.0;

        // final_arrow_marker.points.push_back(start);
        // final_arrow_marker.points.push_back(end);

        // final_arrow_marker.scale.x = 0.05; // Shaft diameter
        // final_arrow_marker.scale.y = 0.1;  // Head diameter
        // final_arrow_marker.scale.z = 0.2;  // Head length

        // final_arrow_marker.color.r = 1.0;
        // final_arrow_marker.color.g = 0.0;
        // final_arrow_marker.color.b = 0.0;
        // final_arrow_marker.color.a = 1.0; // Alpha

        // // Initialize quaternion to identity
        // final_arrow_marker.pose.orientation.x = 0.0;
        // final_arrow_marker.pose.orientation.y = 0.0;
        // final_arrow_marker.pose.orientation.z = 0.0;
        // final_arrow_marker.pose.orientation.w = 1.0;

        // arrow_array.markers.push_back(final_arrow_marker);

        // Publish the marker array
        arrows_pub.publish(arrow_array);

        geometry_msgs::Point potential_pt;
        potential_pt.x = filtered_x_dot_y_dot.first * potential_gain_;
        potential_pt.y = filtered_x_dot_y_dot.second * potential_gain_;

        if (std::abs(potential_pt.x) > saturate_potential_) {
            potential_pt.x = std::copysign(saturate_potential_, potential_pt.x);
        }

        if (std::abs(potential_pt.y) > saturate_potential_) {
            potential_pt.y = std::copysign(saturate_potential_, potential_pt.y);
        }

        potential_pub.publish(potential_pt);
    }

    void publishBasePoints(const std::string& frame_id) {
        publishPointCloud(base_points_pub, base_points_, frame_id);
    }

    void publishPointCloudWithText(const ros::Publisher& pub, const std::vector<std::pair<float, float>>& points, const std::string& frame_id) {
        sensor_msgs::PointCloud point_cloud;
        point_cloud.header.frame_id = frame_id;
        point_cloud.header.stamp = ros::Time::now();

        visualization_msgs::MarkerArray text_marker_array;
        
        for (size_t i = 0; i < points.size(); ++i) {
            const auto& pt = points[i];
            geometry_msgs::Point32 point;
            point.x = pt.first;
            point.y = pt.second;
            point.z = 0;
            point_cloud.points.push_back(point);

            // Create a text marker
            visualization_msgs::Marker text_marker;
            text_marker.header.frame_id = frame_id;
            text_marker.header.stamp = ros::Time::now();
            text_marker.ns = "closest_points_text";
            text_marker.id = i;
            text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::Marker::ADD;
            text_marker.pose.position.x = pt.first;
            text_marker.pose.position.y = pt.second;
            text_marker.pose.position.z = 0.2;  // Slightly above the point
            text_marker.pose.orientation.x = 0.0;
            text_marker.pose.orientation.y = 0.0;
            text_marker.pose.orientation.z = 0.0;
            text_marker.pose.orientation.w = 1.0;
            text_marker.scale.z = 0.1;  // Size of the text
            text_marker.color.r = 1.0;
            text_marker.color.g = 1.0;
            text_marker.color.b = 1.0;
            text_marker.color.a = 1.0;
            text_marker.text = std::to_string(i);

            text_marker_array.markers.push_back(text_marker);
        }

        pub.publish(point_cloud);
        text_markers_pub.publish(text_marker_array);
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
    ros::Publisher potential_pub;
    ros::Publisher arrows_pub;
    ros::Publisher text_markers_pub;
    ros::Publisher quadrants_markers_pub_;
    std::vector<std::pair<float, float>> closest_points;
    std::vector<geometry_msgs::Point32> base_points_;
    PointCloud base_cloud_;
    std::unique_ptr<my_kd_tree_t> base_index_;
    sensor_msgs::LaserScan::ConstPtr latest_scan;
    std::mutex scan_mutex;
    bool running;

    // Parameters
    int robot_density_;
    double robot_height_;
    double robot_width_;
    std::string lidar_topic_;
    int num_clusters_;
    double x_offset_;
    double min_sqr_dist_threshold_;
    size_t max_filtered_points_;
    size_t max_matched_base_points_;
    std::vector<std::vector<float>> colors_;

    std::vector<std::pair<float, float>> cleaned_points;
    std::vector<std::pair<float, float>> cleaned_closest_base_points;
    potential::ObstacleAvoidance potential;
    bool use_angle_filter_;
    double angle_filter_margin_;
    double potential_gain_;
    double saturate_potential_;
    double filter_gain_;

    bool enable_min_threshold_for_obstacle_points_;
    double min_threshold_for_obstacle_points_;

    // Filter parameters
    int filter_type_; // 0: no filter, 1: first-order, 2: second-order

    // First-order filter state variables
    std::pair<double, double> x_dot_y_dot_prev;
    std::pair<double, double> X_dot_prev;

    // Filter chains for x and y components
    filters::FilterChain<double> filter_chain_x;
    filters::FilterChain<double> filter_chain_y;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "obstacle_avoidance");
    ObstacleAvoidance node;
    node.run();
    return 0;
}
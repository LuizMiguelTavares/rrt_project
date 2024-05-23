#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>
#include <opencv2/opencv.hpp>
#include "QTree.hpp"
#include <omp.h>
#include <thread>
#include <memory>
#include <chrono>

namespace motion_planning{
    class Node {
    public:
        int index;
        float x;
        float y;
        std::shared_ptr<Node> parent;

        Node(float x, float y, std::shared_ptr<Node> parent = nullptr) : x(x), y(y), parent(parent) {}
    };

    float distance(const Node& node1, const Node& node2){
        return std::sqrt(std::pow(node1.x - node2.x, 2) + std::pow(node1.y - node2.y, 2));
    }

    float sq_distance(const Node& node1, const Node& node2){
        return std::pow(node1.x - node2.x, 2) + std::pow(node1.y - node2.y, 2);
    }

    std::shared_ptr<Node> steer(std::shared_ptr<Node> nearest, const Node& sample, const float step_size)
    {
        float dist = distance(*nearest, sample);
        if (dist < 0.001) {
            return nearest;
        }

        float x = nearest->x + step_size * (sample.x - nearest->x) / dist;
        float y = nearest->y + step_size * (sample.y - nearest->y) / dist;
        return std::make_shared<Node>(x, y, nearest);
    }

    std::shared_ptr<Node> biased_sample(const cv::Mat& map, const Node& goal, const float bias_probability) {
        static std::default_random_engine generator(std::chrono::system_clock::now().time_since_epoch().count());
        static std::uniform_real_distribution<float> distribution(0.0, 1.0);

        if (distribution(generator) < bias_probability) {
            return std::make_shared<Node>(goal.x, goal.y);
        }

        float rand_x = distribution(generator) * (map.rows - 1);
        float rand_y = distribution(generator) * (map.cols - 1);
        
        int rand_x_int = static_cast<int>(rand_x);
        int rand_y_int = static_cast<int>(rand_y);

        if (rand_x_int < 0) {
            rand_x_int = 0;
        } else if (rand_x_int >= map.rows) {
            rand_x_int = map.rows - 1;
        }

        if (rand_y_int < 0) {
            rand_y_int = 0;
        } else if (rand_y_int >= map.cols) {
            rand_y_int = map.cols - 1;
        }

        return std::make_shared<Node>(rand_y_int, rand_x_int);
    }

    bool isBlack(const cv::Mat& image, int x, int y) {
        if (x >= 0 && x < image.cols && y >= 0 && y < image.rows) {
            return image.at<uchar>(x, y) == 0;
        }
        return false;
    }

    void _bresenhamCircleCollision(const cv::Mat& image, int yc, int xc, int radius, bool &collisionDetected) {
        int x = 0;
        int y = radius;
        int d = 3 - 2 * radius;
        while (y >= x) {
            // Check points using 8-way symmetry
            if (isBlack(image, xc + x, yc + y) || isBlack(image, xc - x, yc + y) ||
                isBlack(image, xc + x, yc - y) || isBlack(image, xc - x, yc - y) ||
                isBlack(image, xc + y, yc + x) || isBlack(image, xc - y, yc + x) ||
                isBlack(image, xc + y, yc - x) || isBlack(image, xc - y, yc - x)) {
                collisionDetected = true;
                return;
            }
            x++;
            if (d > 0) {
                y--;
                d += 4 * (x - y) + 10;
            } else {
                d += 4 * x + 6;
            }
        }
    }

    bool check_obstacle_intersection(const cv::Mat& image, int xBegin, int yBegin, const int xEnd, const int yEnd, const int radius) {
        int dx = abs(xEnd - xBegin), sx = xBegin < xEnd ? 1 : -1;
        int dy = -abs(yEnd - yBegin), sy = yBegin < yEnd ? 1 : -1; 
        int error = dx + dy, error2;

        while (true) {
            bool collisionDetected = false;
            _bresenhamCircleCollision(image, xBegin, yBegin, radius, collisionDetected);
            if (collisionDetected) {
                return true; }
            if (xBegin == xEnd && yBegin == yEnd) break;
            error2 = 2 * error;
            if (error2 >= dy) { error += dy; xBegin += sx; }
            if (error2 <= dx) { error += dx; yBegin += sy; }
        }
        return false;
    }

    std::vector<std::shared_ptr<Node>> rrt(const cv::Mat& grid_map, std::shared_ptr<Node> start, std::shared_ptr<Node> goal, const int num_nodes, const float step_size, const float goal_threshold, const float bias_probability, const int radius) {
        QTree::Rectangle boundary(grid_map.cols/2, grid_map.rows/2, grid_map.cols, grid_map.rows);
        std::shared_ptr<QTree::QuadTree<Node>> tree = std::make_shared<QTree::QuadTree<Node>>(boundary, 4);

        const bool debug = false;
        const bool time_debug = false;
        const float sq_distance_threshold = goal_threshold * goal_threshold;

        if (time_debug) {
            std::cout << "Time Debugging Enabled" << std::endl;
        }
        std::chrono::steady_clock::time_point startFullTime = std::chrono::steady_clock::now();
        std::chrono::steady_clock::time_point startTime;
        std::vector<std::shared_ptr<Node>> nodes;
        
        if (debug){
            std::cout << "Starting RRT" << std::endl;
            std::cout << "Grid Map Size: " << grid_map.cols << ", " << grid_map.rows << std::endl;
            std::cout << "Start Node at :" << start->x << ", " << start->y << std::endl;
            std::cout << "Goal Node at :" << goal->x << ", " << goal->y << std::endl;
        }

        if (std::isnan(start->x) || std::isnan(start->y) || std::isnan(goal->x) || std::isnan(goal->y)) {
            std::cerr << "Error: Start or goal node is NaN." << std::endl;
            return nodes;
        }

        if (start->x < 0 || start->x >= grid_map.cols || start->y < 0 || start->y >= grid_map.rows) {
            std::cerr << "Error: Start node is outside the boundaries of the map." << std::endl;
            return nodes;
        }

        nodes.reserve(num_nodes);
        try {
            tree->insert(start);
        } catch (const std::runtime_error& e) {
            std::cerr << "Failed to insert start node into QuadTree: " << e.what() << std::endl;
            return nodes;
        }

        nodes.push_back(start);

        for (int i = 0; i < num_nodes; i++) {
            if (time_debug) {
                startTime = std::chrono::steady_clock::now();
            }
            
            if (debug){
                std::cout << "Iteration: " << i << std::endl;
                std::cout << "Going to sample" << std::endl;
            }

                std::shared_ptr<Node> sample = biased_sample(grid_map, *goal, bias_probability);

            if (debug){
                std::cout << "Sampled Node at :" << sample->x << ", " << sample->y << std::endl;
            }

            if(sample->x < 0 || sample->x >= grid_map.cols || sample->y < 0 || sample->y >= grid_map.rows) {
                if (debug){
                    std::cout << "Sample node is outside the boundaries of the map" << std::endl;
                }

                std::cerr << "Error: Sample node is outside the boundaries of the map." << std::endl;
                std::cerr << "X: " << sample->x << ", Y: " << sample->y << std::endl;
                std::cerr << "Map Size: " << grid_map.cols << ", " << grid_map.rows << std::endl;
                continue;
            }

            if (std::isnan(sample->x) || std::isnan(sample->y)) {
                std::cerr << "Error: Sample node is NaN." << std::endl;
                continue;
            }

            if (debug){
                std::cout << "Going to nearest node" << std::endl;
            }
            std::shared_ptr<Node> nearest_node = tree->nearest_neighbor(sample);
            if (debug){
                std::cout << "Nearest Node at :" << nearest_node->x << ", " << nearest_node->y << std::endl;
            }

            if (std::isnan(nearest_node->x) || std::isnan(nearest_node->y)) {
                std::cerr << "Error: Nearest node is NaN." << std::endl;
                continue;
            }

            if (debug){
                std::cout << "Going to steer" << std::endl;
            }

            std::shared_ptr<Node> new_node = steer(nearest_node, *sample, step_size);

            if (new_node == nearest_node) {
                if (debug){
                    std::cout << "New node is the same as nearest node" << std::endl;
                }
                continue;
            }

            if (debug){
                std::cout << "Steered Node at :" << new_node->x << ", " << new_node->y << std::endl;
                std::cout << "Going to check boundaries" << std::endl;
            }

            if (new_node->x < 0 || new_node->x >= grid_map.cols || new_node->y < 0 || new_node->y >= grid_map.rows) {
                std::cerr << "Error: New node is outside the boundaries of the map." << std::endl;
                std::cerr << "Should not happen! Check steering function!";
                std::vector<std::shared_ptr<Node>> error_node;
                return error_node;
            }
            if (debug){
                std::cout << "Going to check obstacle intersection" << std::endl;
            }
            if (check_obstacle_intersection(grid_map, nearest_node->x, nearest_node->y, new_node->x, new_node->y, radius)) {
                if (debug){
                std::cout << "Obstacle detected" << std::endl;}
                continue;
            }

            if (debug){
                std::cout << "No obstacle detected" << std::endl;
                std::cout << "Going to insert node" << std::endl;
            }

            try {
                if (tree->insert(new_node)){
                    if (debug){
                    std::cout << "Node inserted at :" << new_node->x << ", " << new_node->y << "\n" <<std::endl;}
                    nodes.push_back(new_node);
                } else {
                    std::cerr << "Failed to insert node into QuadTree" << std::endl;
                    std::cerr << "Node: (" << new_node->x << ", " << new_node->y << ")" << std::endl;
                    std::cerr << "Boundaries: (" << boundary.x << ", " << boundary.y << ")" << std::endl;
                    std::cerr << "Iteration: " << i << std::endl;
                }
                
            } catch (const std::runtime_error& e) {
                std::cerr << "Failed to insert node into QuadTree: " << e.what() << std::endl;
                std::cerr << "Node: (" << new_node->x << ", " << new_node->y << ")" << std::endl;
                std::cerr << "Boundaries: (" << boundary.x << ", " << boundary.y << ")" << std::endl;
                std::cerr << "Iteration: " << i << std::endl;
                std::vector<std::shared_ptr<Node>> error_node;
                return error_node;
            }

            float sq_distance_ = sq_distance(*new_node, *goal);

            if (sq_distance_ <= sq_distance_threshold) {
                if (debug) {
                    std::cout << "Goal reached!" << std::endl;
                    std::cout << nodes.size() << std::endl;
                }
                goal->parent = new_node;
                nodes.push_back(goal);
                break;
            }

            if (time_debug) {
                std::chrono::steady_clock::time_point endTime = std::chrono::steady_clock::now();
                std::chrono::duration<double> elapsedTime = endTime - startTime;
                std::cout << "Elapsed time: " << elapsedTime.count() << " seconds" << std::endl;

                std::chrono::duration<double> fullElapsedTime = endTime - startFullTime;
                std::cout << "Full Elapsed time: " << fullElapsedTime.count() << " seconds\n" << std::endl;   
            }
        }
        return nodes;
    }

    std::vector<std::shared_ptr<Node>> trace_goal_path(std::shared_ptr<Node> goal_node) {
        std::vector<std::shared_ptr<Node>> path;
        std::shared_ptr<Node> current_node = goal_node;
        while (current_node->parent) {
            path.push_back(current_node);
            current_node = current_node->parent;
        }
        path.push_back(current_node);
        std::reverse(path.begin(), path.end());
        return path;
    }

    void plot_rrt(const cv::Mat& map, const std::shared_ptr<Node> start, const std::shared_ptr<Node> end, const bool reached, const std::vector<std::shared_ptr<Node>>& nodes) {
        // Create a white image
        cv::Mat image(map.size(), CV_8UC3, cv::Scalar(255, 255, 255));

        // Overlay the grayscale map onto the white image
        cv::cvtColor(map, image, cv::COLOR_GRAY2BGR);

        // Draw nodes and edges
        for (int i = 1; i < nodes.size()-1; i++) {
            cv::circle(image, 
                cv::Point(static_cast<int>(nodes[i-1]->x), static_cast<int>(nodes[i-1]->y)), 
                3, cv::Scalar(0, 0, 255), -1);

            std::shared_ptr<Node> parent_node = nodes[i]->parent;

            cv::line(image, 
                cv::Point(static_cast<int>(nodes[i]->x), static_cast<int>(nodes[i]->y)),
                cv::Point(static_cast<int>(parent_node->x), static_cast<int>(parent_node->y)), 
                cv::Scalar(255, 0, 0), 1);
        }

        cv::circle(image, 
                    cv::Point(static_cast<int>(nodes[nodes.size()-2]->x), static_cast<int>(nodes[nodes.size()-2]->y)), 
                    3, cv::Scalar(0, 0, 255), -1);

        if (reached){
            cv::circle(image, 
                    cv::Point(static_cast<int>(end->x), static_cast<int>(end->y)), 
                    3, cv::Scalar(0, 0, 255), -1);

            cv::line(image, 
                    cv::Point(static_cast<int>(nodes[nodes.size()-2]->x), static_cast<int>(nodes[nodes.size()-2]->y)),
                    cv::Point(static_cast<int>(end->x), static_cast<int>(end->y)), 
                    cv::Scalar(255, 0, 0), 1);
        }
        // Draw start and goal
        cv::circle(image, cv::Point(static_cast<int>(start->x), static_cast<int>(start->y)), 6, cv::Scalar(0, 0, 255), 1);
        cv::circle(image, cv::Point(static_cast<int>(end->x), static_cast<int>(end->y)), 6, cv::Scalar(0, 0, 255), 1);

        // reshape to 500,500
        cv::resize(image, image, cv::Size(500, 500), 0, 0, cv::INTER_NEAREST);

        cv::imshow("RRT", image);
        cv::waitKey(0);
    }
} // namespace motion_planning
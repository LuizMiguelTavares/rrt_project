#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>
#include <opencv2/opencv.hpp>
#include "QTree.hpp"
#include <omp.h>
#include <thread>

namespace motion_planning{
    class Node {
    public:
        int index;
        std::vector<double> position;
        Node* parent;
        int x;
        int y;

        Node(std::vector<double> pos, Node* par = nullptr) : position(pos), parent(par), x(pos[0]), y(pos[1]) {}
    };

    double distance(const Node& node1, const Node& node2) {
        double dist = 0.0;
        for (size_t i = 0; i < node1.position.size(); i++) {
            dist += (node1.position[i] - node2.position[i]) * (node1.position[i] - node2.position[i]);
        }
        return std::sqrt(dist);
    }

    Node* nearest_node(const Node& sample, const std::vector<Node*>& nodes) {
        return *std::min_element(nodes.begin(), nodes.end(), [&sample](Node* a, Node* b) {
            return distance(sample, *a) < distance(sample, *b);
        });
    }

    Node* steer(Node* nearest, const Node& sample, const double step_size) {
        std::vector<double> direction;
        double dist = distance(*nearest, sample);
        for (size_t i = 0; i < sample.position.size(); i++) {
            direction.push_back((sample.position[i] - nearest->position[i]) / dist);
        }

        std::vector<double> new_position;
        for (size_t i = 0; i < nearest->position.size(); i++) {
            new_position.push_back(nearest->position[i] + step_size * direction[i]);
        }

        return new Node(new_position, nearest);
    }

    Node* biased_sample(const cv::Mat& map, const Node& goal, const double bias_probability) {
        static std::default_random_engine generator(std::chrono::system_clock::now().time_since_epoch().count());
        static std::uniform_real_distribution<double> distribution(0.0, 1.0);

        if (distribution(generator) < bias_probability) {
            return new Node(goal.position);
        }

        std::vector<double> random_position;

        // Generate random float coordinates
        float rand_x = distribution(generator) * map.rows;
        float rand_y = distribution(generator) * map.cols;
        
        // Convert to integer coordinates
        int rand_x_int = static_cast<int>(rand_x);
        int rand_y_int = static_cast<int>(rand_y);

        random_position.push_back(rand_x_int);
        random_position.push_back(rand_y_int);

        return new Node(random_position);
    }

    bool isBlack(const cv::Mat& image, int x, int y) {
        if (x >= 0 && x < image.cols && y >= 0 && y < image.rows) {
           
            return image.at<uchar>(x, y) != 255;
        }
        return false;
    }

    // void _bresenhamCircleCollision(const cv::Mat& image, int xc, int yc, int radius, bool &collisionDetected) {
    //     int x = 0;
    //     int y = radius;
    //     int d = 3 - 2 * radius;

    //     #pragma omp parallel firstprivate(x, y, d) shared(collisionDetected)
    //     {
    //         while (y >= x) {
    //             #pragma omp single nowait
    //             {
    //                 // Check points using 8-way symmetry
    //                 if (isBlack(image, xc + x, yc + y) || isBlack(image, xc - x, yc + y) ||
    //                     isBlack(image, xc + x, yc - y) || isBlack(image, xc - x, yc - y) ||
    //                     isBlack(image, xc + y, yc + x) || isBlack(image, xc - y, yc + x) ||
    //                     isBlack(image, xc + y, yc - x) || isBlack(image, xc - y, yc - x)) {
    //                     #pragma omp critical
    //                     {
    //                         collisionDetected = true;
    //                     }
    //                 }
    //             }

    //             #pragma omp barrier

    //             #pragma omp single
    //             {
    //                 if (!collisionDetected) {
    //                     x++;
    //                     if (d > 0) {
    //                         y--;
    //                         d += 4 * (x - y) + 10;
    //                     } else {
    //                         d += 4 * x + 6;
    //                     }
    //                 }
    //             }
    //         }
    //     }
    // }

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
                return; // Exit as soon as a collision is detected
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

        // std::vector<cv::Point> line_points; 

        while (true) {
            // line_points.push_back(cv::Point(xBegin, yBegin)); // Add point to the vector

            // // Check if the point is within the image boundaries and is an obstacle
            // if (xBegin >= 0 && xBegin < image.cols && yBegin >= 0 && yBegin < image.rows) {
            //     if (image.at<uchar>(yBegin, xBegin) != 255) { 
            //         return true;
            //     }
            // }

            // Check if the point of the circle centered at (xBegin, yBegin) with radius r intersects an obstacle
            bool collisionDetected = false;

            _bresenhamCircleCollision(image, xBegin, yBegin, radius, collisionDetected);

            if (collisionDetected) {

                return true;
            }

            if (xBegin == xEnd && yBegin == yEnd) break;
            error2 = 2 * error;
            if (error2 >= dy) { error += dy; xBegin += sx; }
            if (error2 <= dx) { error += dx; yBegin += sy; }
        }
        return false;
    }

    std::vector<Node*> rrt(const cv::Mat& grid_map, Node* start, Node* goal, const int num_nodes, const double step_size, const double goal_threshold, const double bias_probability, const int radius) {

        QTree::Rectangle boundary(grid_map.cols/2, grid_map.rows/2, grid_map.cols, grid_map.rows);
        QTree::QuadTree<Node> tree(boundary, start, 4);

        std::vector<Node*> nodes;
        nodes.reserve(num_nodes);
        nodes.push_back(start);

        for (int i = 0; i < num_nodes; i++) {
            Node* sample = biased_sample(grid_map, *goal, bias_probability);
            Node* nearest_node = tree.nearest_neighbor(sample);
            Node* new_node = steer(nearest_node, *sample, step_size);

            if (check_obstacle_intersection(grid_map, nearest_node->position[0], nearest_node->position[1], new_node->position[0], new_node->position[1], radius)) {
                delete sample;
                delete new_node;
                continue;
            }

            nodes.push_back(new_node);
            tree.insert(new_node);

            if (distance(*new_node, *goal) <= goal_threshold) {
                std::cout << "Goal reached!" << std::endl;
                std::cout << nodes.size() << std::endl;
                nodes.push_back(goal);
                break;
            }
            delete sample;
        }
        return nodes;
    }

    void plot_rrt(const cv::Mat& map, const Node* start, const Node* end, const bool reached, const std::vector<Node*>& nodes) {
        // Create a white image
        cv::Mat image(map.size(), CV_8UC3, cv::Scalar(255, 255, 255));

        // Overlay the grayscale map onto the white image
        cv::cvtColor(map, image, cv::COLOR_GRAY2BGR);

        // Draw nodes and edges
        for (int i = 1; i < nodes.size()-1; i++) {
            cv::circle(image, 
                cv::Point(static_cast<int>(nodes[i-1]->position[0]), static_cast<int>(nodes[i-1]->position[1])), 
                3, cv::Scalar(0, 0, 255), -1);

            Node* parent_node = nodes[i]->parent;

            cv::line(image, 
                cv::Point(static_cast<int>(nodes[i]->position[0]), static_cast<int>(nodes[i]->position[1])),
                cv::Point(static_cast<int>(parent_node->position[0]), static_cast<int>(parent_node->position[1])), 
                cv::Scalar(255, 0, 0), 1);
        }

        cv::circle(image, 
                    cv::Point(static_cast<int>(nodes[nodes.size()-2]->position[0]), static_cast<int>(nodes[nodes.size()-2]->position[1])), 
                    3, cv::Scalar(0, 0, 255), -1);

        if (reached){
            cv::circle(image, 
                    cv::Point(static_cast<int>(end->position[0]), static_cast<int>(end->position[1])), 
                    3, cv::Scalar(0, 0, 255), -1);

            cv::line(image, 
                    cv::Point(static_cast<int>(nodes[nodes.size()-2]->position[0]), static_cast<int>(nodes[nodes.size()-2]->position[1])),
                    cv::Point(static_cast<int>(end->position[0]), static_cast<int>(end->position[1])), 
                    cv::Scalar(255, 0, 0), 1);
        }
        // Draw start and goal
        cv::circle(image, cv::Point(static_cast<int>(start->position[0]), static_cast<int>(start->position[1])), 6, cv::Scalar(0, 0, 255), 1);
        cv::circle(image, cv::Point(static_cast<int>(end->position[0]), static_cast<int>(end->position[1])), 6, cv::Scalar(0, 0, 255), 1);

        // rechape to 500,500
        cv::resize(image, image, cv::Size(500, 500), 0, 0, cv::INTER_NEAREST);

        cv::imshow("RRT", image);
        cv::waitKey(1);
    }

    std::vector<Node*> trace_goal_path(Node* goal_node) {
        std::vector<Node*> path;
        Node* current_node = goal_node;
        while (current_node->parent) {
            path.push_back(current_node);
            current_node = current_node->parent;
        }
        path.push_back(current_node);  // add start node
        std::reverse(path.begin(), path.end());
        return path;
    }
} // namespace motion_planning
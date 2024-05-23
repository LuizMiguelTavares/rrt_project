#include "rrt.hpp"
#include <iostream>
#include <cmath>
#include <random>
#include <chrono>

namespace motion_planning {
    Node::Node(std::vector<double> pos, Node* par) : parent(par), is_empty(false) {
        if (!pos.empty()) {
            position = pos;
            x = pos[0];
            y = pos[1];
        } else {
            is_empty = true; 
        }
    }
 
    Node::Node() : is_empty(true) {}
    Node::~Node() {
        delete parent;
    }

    bool Node::empty() {
        return position.empty();
    }

    void Node::setParent(Node* par) {
        parent = par;
    }

    void Node::setPosition(std::vector<double> pos) {
        position = pos;
        x = pos[0];
        y = pos[1];
        is_empty = false;
    }

    RRT::RRT(const cv::Mat& map, Node* start, Node* goal, int num_nodes, double step_size, double goal_threshold, double bias_probability)
        : map_(map), start_(start), goal_(goal), num_nodes_(num_nodes), step_size_(step_size), goal_threshold_(goal_threshold), bias_probability_(bias_probability) {}

    RRT::~RRT() {
        delete start_;
        delete goal_;
    }

    std::vector<Node*> RRT::run() {
        QTree::Rectangle boundary(map_.cols/2, map_.rows/2, map_.cols, map_.rows);
        QTree::QuadTree<Node> tree(boundary, start_, 4);

        std::vector<Node*> nodes;
        nodes.reserve(num_nodes_);
        nodes.push_back(start_);

        for (int i = 0; i < num_nodes_; i++) {
            Node* sample = biasedSample();
            Node* nearest_node = tree.nearest_neighbor(sample);
            Node* new_node = steer(nearest_node, *sample);

            if (checkObstacleIntersection(nearest_node, new_node)) {
                delete sample;
                delete new_node;
                continue;}

            nodes.push_back(new_node);
            tree.insert(new_node);

            if (distance(*new_node, *goal_) <= goal_threshold_) {
                std::cout << "Goal reached!" << std::endl;
                std::cout << nodes.size() << std::endl;
                nodes.push_back(goal_);
                break;}
            delete sample;
        }
        return nodes;
    }

    void RRT::plot(const std::vector<Node*>& nodes, const Node* start, const Node* end, bool reached) {
        // Create a white image
        cv::Mat image(map_.size(), CV_8UC3, cv::Scalar(255, 255, 255));

        // Overlay the grayscale map_ onto the white image
        cv::cvtColor(map_, image, cv::COLOR_GRAY2BGR);

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
        cv::waitKey(0);
    }

    std::vector<Node*> RRT::traceGoalPath(Node* goal_node) {
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
    
    Node* RRT::biasedSample() {
        static std::default_random_engine generator(std::chrono::system_clock::now().time_since_epoch().count());
        static std::uniform_real_distribution<double> distribution(0.0, 1.0);

        if (distribution(generator) < bias_probability_) {
            return new Node(goal_->position);}

        std::vector<double> random_position;

        // Generate random float coordinates
        float rand_x = distribution(generator) * map_.rows;
        float rand_y = distribution(generator) * map_.cols;
        
        // Convert to integer coordinates
        int rand_x_int = static_cast<int>(rand_x);
        int rand_y_int = static_cast<int>(rand_y);

        random_position.push_back(rand_x_int);
        random_position.push_back(rand_y_int);

        return new Node(random_position);}

    Node* RRT::steer(Node* nearest, const Node& sample) const {
        std::vector<double> direction;
        double dist = distance(*nearest, sample);
        for (size_t i = 0; i < sample.position.size(); i++) {
            direction.push_back((sample.position[i] - nearest->position[i]) / dist);
        }

        std::vector<double> new_position;
        for (size_t i = 0; i < nearest->position.size(); i++) {
            new_position.push_back(nearest->position[i] + step_size_ * direction[i]);
        }

        return new Node(new_position, nearest);
    }

    double RRT::distance(const Node& node1, const Node& node2) const {
        double sum = 0.0;
        for (size_t i = 0; i < node1.position.size(); i++) {
            double diff = node1.position[i] - node2.position[i];
            sum += diff * diff;}
        return std::sqrt(sum);
    }

    bool RRT::checkObstacleIntersection(const Node* start, const Node* end) const {

        int xBegin = start->x;
        int yBegin = start->y;
        const int xEnd = end->x;
        const int yEnd = end->y;

        int dx = abs(xEnd - xBegin), sx = xBegin < xEnd ? 1 : -1;
        int dy = -abs(yEnd - yBegin), sy = yBegin < yEnd ? 1 : -1; 
        int error = dx + dy, error2;

        std::vector<cv::Point> line_points; 

        while (true) {
            line_points.push_back(cv::Point(xBegin, yBegin));

            // Check if the point is within the map_ boundaries and is an obstacle
            if (xBegin >= 0 && xBegin < map_.cols && yBegin >= 0 && yBegin < map_.rows) {
                if (map_.at<uchar>(yBegin, xBegin) != 255) { 
                    return true;
                }
            }

            if (xBegin == xEnd && yBegin == yEnd) break;
            error2 = 2 * error;
            if (error2 >= dy) { error += dy; xBegin += sx; }
            if (error2 <= dx) { error += dx; yBegin += sy; }}
        return false;
    }
} // namespace motion_planning
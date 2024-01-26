#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>
#include <opencv2/opencv.hpp>

class Node {
public:
    std::vector<double> position;
    Node* parent;

    Node(std::vector<double> pos, Node* par = nullptr) : position(pos), parent(par) {}
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

Node* steer(Node* nearest, const Node& sample, double step_size) {
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

Node* biased_sample(const Node& goal, double bias_probability, int space_dim, const std::vector<double>& space_size) {
    static std::default_random_engine generator(std::chrono::system_clock::now().time_since_epoch().count());
    static std::uniform_real_distribution<double> distribution(0.0, 1.0);
    if (distribution(generator) < bias_probability) {
        return new Node(goal.position);
    }

    std::vector<double> random_position;
    for (int i = 0; i < space_dim; i++) {
        random_position.push_back(distribution(generator) * space_size[i]);
    }

    return new Node(random_position);
}

std::vector<Node*> rrt(Node* start, Node* goal, int space_dim, const std::vector<double>& space_size, int num_nodes, double step_size, double goal_threshold, double bias_probability) {
    std::vector<Node*> nodes = { start };
    for (int i = 0; i < num_nodes; i++) {
        Node* sample = biased_sample(*goal, bias_probability, space_dim, space_size);
        Node* nearest = nearest_node(*sample, nodes);
        Node* new_node = steer(nearest, *sample, step_size);
        nodes.push_back(new_node);

        if (distance(*new_node, *goal) <= goal_threshold) {
            std::cout << "Goal reached!" << std::endl;
            std::cout << nodes.size() << std::endl;
            nodes.push_back(goal);
            break;
        }
        delete sample; // Avoid memory leaks
    }
    return nodes;
}

void plot_rrt(const std::vector<Node*>& nodes) {
    int width = 500;
    int height = 500;
    cv::Mat image(height, width, CV_8UC3, cv::Scalar(255, 255, 255));

    // Draw nodes and edges
    for (auto node : nodes) {
        if (node->parent) {
            cv::line(image, 
                     cv::Point(static_cast<int>(node->parent->position[0] * width/10), static_cast<int>(node->parent->position[1] * height/10)),
                     cv::Point(static_cast<int>(node->position[0] * width/10), static_cast<int>(node->position[1] * height/10)), 
                     cv::Scalar(255, 0, 0), 1);
        }
        cv::circle(image, 
                   cv::Point(static_cast<int>(node->position[0] * width/10), static_cast<int>(node->position[1] * height/10)), 
                   3, cv::Scalar(0, 0, 255), -1);
    }

    // Draw start and goal
    cv::circle(image, cv::Point(static_cast<int>(nodes[0]->position[0] * width/10), static_cast<int>(nodes[0]->position[1] * height/10)), 6, cv::Scalar(0, 0, 0), -1);
    cv::circle(image, cv::Point(static_cast<int>(nodes.back()->position[0] * width/10), static_cast<int>(nodes.back()->position[1] * height/10)), 6, cv::Scalar(0, 0, 0), -1);

    cv::imshow("RRT", image);
    cv::waitKey(0);
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

int main() {
    auto start_time = std::chrono::high_resolution_clock::now();

    Node* start = new Node({ 0.0, 0.0 });
    Node* goal = new Node({ 6.0, 9.0 });
    double goal_threshold = 0.05;
    std::vector<Node*> nodes = rrt(start, goal, 2, { 10, 10 }, 5000, 0.1, goal_threshold, 0.1);

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    std::cout << "Time taken by function: " << duration << " milliseconds" << std::endl;

    plot_rrt(nodes);

    if (distance(*nodes.back(), *goal) < goal_threshold) {
        std::vector<Node*> goal_path = trace_goal_path(nodes[nodes.size() - 2]);
        plot_rrt(goal_path);
    } else {
        std::cout << "Goal not reached!" << std::endl;
    }

    // Clean up memory
    for (auto node : nodes) {
        delete node;
    }

    return 0;
}

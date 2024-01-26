#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>
#include <opencv2/opencv.hpp>

class Node {
public:
    std::vector<double> position;  // x, y, theta, v, w
    Node* parent;

    Node(const std::vector<double>& pos, Node* par = nullptr) : position(pos), parent(par) {}
};

double distance(const Node& node1, const Node& node2) {
    double dx = node1.position[0] - node2.position[0];
    double dy = node1.position[1] - node2.position[1];
    double dtheta = node1.position[2] - node2.position[2];
    dtheta = std::fmod((dtheta + M_PI), (2*M_PI)) - M_PI;
    double weight_theta = 0.5;
    return std::sqrt(dx*dx + dy*dy + weight_theta * dtheta*dtheta);
}

Node* nearest_node(const Node& sample, const std::vector<Node*>& nodes) {
    return *std::min_element(nodes.begin(), nodes.end(), [&sample](Node* a, Node* b) {
        return distance(*a, sample) < distance(*b, sample);
    });
}

Node* steer(Node* nearest, const Node& sample, const std::vector<std::pair<double, double>>& augmented_states_limits, double dt) {
    double x = nearest->position[0];
    double y = nearest->position[1];
    double theta = nearest->position[2];
    double v = nearest->position[3];
    double w = nearest->position[4];

    double x_desired = sample.position[0];
    double y_desired = sample.position[1];
    double theta_desired = sample.position[2];
    double v_desired = sample.position[3];
    double w_desired = sample.position[4];

    double dx = x_desired - x;
    double dy = y_desired - y;

    double desired_v_from_position = std::sqrt(dx*dx + dy*dy) / dt;
    double desired_w_from_position = (theta_desired - theta) / dt;
    double alpha = 0.8;

    double blended_v_desired = alpha * desired_v_from_position + (1.0 - alpha) * v_desired;
    double blended_w_desired = alpha * desired_w_from_position + (1.0 - alpha) * w_desired;

    double delta_v = blended_v_desired - v;
    double delta_w = blended_w_desired - w;

    delta_v = std::clamp(delta_v, augmented_states_limits[4].first * dt, augmented_states_limits[4].second * dt);
    delta_w = std::clamp(delta_w, augmented_states_limits[5].first * dt, augmented_states_limits[5].second * dt);

    double v_new = v + delta_v;
    double w_new = w + delta_w;

    v_new = std::clamp(v_new, augmented_states_limits[3].first, augmented_states_limits[3].second);
    w_new = std::clamp(w_new, augmented_states_limits[4].first, augmented_states_limits[4].second);

    double x_new = x + v_new * std::cos(theta) * dt;
    double y_new = y + v_new * std::sin(theta) * dt;

    if (x_new < augmented_states_limits[0].first || x_new > augmented_states_limits[0].second ||
        y_new < augmented_states_limits[1].first || y_new > augmented_states_limits[1].second) {
        return nullptr;
    }

    double theta_new = theta + w_new * dt;
    theta_new = std::fmod(theta_new + M_PI, 2.0 * M_PI) - M_PI;

    return new Node({ x_new, y_new, theta_new, v_new, w_new }, nearest);
}

Node* biased_sample(const Node& goal, double bias_probability, const std::vector<std::pair<double, double>>& state_limits) {
    static std::default_random_engine generator(std::chrono::system_clock::now().time_since_epoch().count());
    static std::uniform_real_distribution<double> distribution(0.0, 1.0);

    if (distribution(generator) < bias_probability) {
        return new Node(goal.position);
    }

    std::vector<double> random_position;
    for (const auto& dim_size : state_limits) {
        random_position.push_back(distribution(generator) * (dim_size.second - dim_size.first) + dim_size.first);

    }

    return new Node(random_position);
}

std::vector<Node*> rrt(Node* start, Node* goal, const std::vector<std::pair<double, double>>& state_limits, 
                       const std::vector<std::pair<double, double>>& augmented_states_limits, int num_nodes, 
                       double dt, double goal_threshold, double bias_probability) {
    
    std::vector<Node*> nodes = { start };
    int n = 0;
    int count = 0;
    static std::default_random_engine generator(std::chrono::system_clock::now().time_since_epoch().count());
    static std::uniform_real_distribution<double> distribution(0.0, 1.0);

    while (n < num_nodes) {
        if (count > 2 * num_nodes) {
            std::cout << "Stopped early" << std::endl;
            break;
        }
        count++;

        Node* sample = biased_sample(*goal, bias_probability, state_limits);
        Node* nearest = nearest_node(*sample, nodes);
        Node* new_node = steer(nearest, *sample, augmented_states_limits, dt);

        if (new_node) {
            nodes.push_back(new_node);
            n++;
        } else {
            delete sample; // Avoid memory leaks if new_node isn't created
            continue;
        }

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

void plot_rrt(const std::vector<Node*>& nodes, const std::pair<double, double>& max_coords) {
    int width = 500;
    int height = 500;
    cv::Mat image(height, width, CV_8UC3, cv::Scalar(255, 255, 255));

    // Draw nodes and edges
    for (auto node : nodes) {
        int x = static_cast<int>(node->position[0] * width / max_coords.first);
        int y = height - static_cast<int>(node->position[1] * height / max_coords.second);

        int px = node->parent ? static_cast<int>(node->parent->position[0] * width / max_coords.first) : 0;
        int py = node->parent ? height - static_cast<int>(node->parent->position[1] * height / max_coords.second) : 0;

        if (node->parent) {
            cv::line(image, cv::Point(px, py), cv::Point(x, y), cv::Scalar(255, 0, 0), 1);
        }
        cv::circle(image, cv::Point(x, y), 3, cv::Scalar(0, 0, 255), -1);
    }

    // Draw start and goal
    cv::circle(image, cv::Point(static_cast<int>(nodes[0]->position[0] * width / max_coords.first), height - static_cast<int>(nodes[0]->position[1] * height / max_coords.second)), 6, cv::Scalar(0, 0, 0), -1);
    cv::circle(image, cv::Point(static_cast<int>(nodes.back()->position[0] * width / max_coords.first), height - static_cast<int>(nodes.back()->position[1] * height / max_coords.second)), 6, cv::Scalar(0, 0, 0), -1);

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

    Node* start = new Node({ 0.0, 0.0, 0.0, 0.0, 0.0 });
    Node* goal = new Node({ 7.0, 5.0, 0.0, 0.0, 0.0 });
    std::pair<double, double> max_coords(10.0, 10.0); 
    std::vector<std::pair<double, double>> state_limits = {{0, max_coords.first}, {0, max_coords.second}, {-M_PI, M_PI}, {-3, 3}, {-3, 3}};
    std::vector<std::pair<double, double>> augmented_states_limits = {{0, max_coords.first}, {0, max_coords.second}, {-M_PI, M_PI}, {-3, 3}, {-3, 3}, {-0.5, 0.5}, {-0.5, 0.5}};
    double goal_threshold = 0.2;

    std::vector<Node*> nodes = rrt(start, goal, state_limits, augmented_states_limits, 20000, 0.3, goal_threshold, 0.35);

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    std::cout << "Time taken by function: " << duration << " milliseconds" << std::endl;
    
    plot_rrt(nodes, max_coords);

    if (distance(*nodes.back(), *goal) < goal_threshold) {
        std::vector<Node*> goal_path = trace_goal_path(nodes[nodes.size() - 2]);
        plot_rrt(goal_path, max_coords);
    } else {
        std::cout << "Goal not reached!" << std::endl;
    }

    for (auto node : nodes) {
        delete node;
    }
    return 0;
}
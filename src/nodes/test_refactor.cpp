#include "QTree_ptr.hpp"
#include <iostream>
#include <random>
#include <memory>
#include <opencv2/opencv.hpp>
#include <cmath> // for sqrt and pow functions
#include <limits> // for numeric_limits
#include <assert.h>

int generateRandomPoint(int range) {

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, range-1);
    int x = dis(gen);
    return x;
}

class Node {
    public:
        Node(int x, int y) : x(x), y(y) {}
        Node() : x(0), y(0) {}

        int x;
        int y;
};

std::shared_ptr<Node> bruteForceNearestNeighbor(const std::shared_ptr<Node>& target_node, const std::vector<std::shared_ptr<Node>>& all_nodes) {
    if (all_nodes.empty()) return nullptr;

    double min_distance = std::numeric_limits<double>::max();
    std::shared_ptr<Node> nearest_node;

    for (const auto& node : all_nodes) {
        double distance = pow(target_node->x - node->x, 2) + pow(target_node->y - node->y, 2);

        if (distance < min_distance) { // Update nearest node if a closer one is found
            min_distance = distance;
            nearest_node = node;
        }
    }

    return nearest_node;
}

int main() {
    QTree::Rectangle boundary(250, 250, 500, 500);
    std::shared_ptr<QTree::QuadTree<Node>> qtree = std::make_shared<QTree::QuadTree<Node>>(boundary, 1);

    cv::Mat img = cv::Mat::zeros(501, 501, CV_8UC3);
    cv::Mat backgroung_img = cv::Mat::zeros(501, 501, CV_8UC3);

    std::vector<std::shared_ptr<Node>> all_nodes;
    for (int i = 0; i < 1000; i++) {
        int x = generateRandomPoint(500);
        int y = generateRandomPoint(500);
        std::shared_ptr<Node> node = std::make_shared<Node>(x, y);
        qtree->insert(node);
        all_nodes.push_back(node);
        cv::circle(backgroung_img, cv::Point(x, y), 1, cv::Scalar(0, 255, 0), -1);
        img = backgroung_img.clone();
    }

    QTree::Rectangle range_test;
    std::shared_ptr<Node> node_test;
    std::vector<std::shared_ptr<Node>> nodes_test;

    for (int i = 0; i < 1000; i++){
        int x = generateRandomPoint(500);
        int y = generateRandomPoint(500);
        std::shared_ptr<Node> node = std::make_shared<Node>(x, y);

        QTree::Rectangle range_for_test;
        std::shared_ptr<Node> first_valid_node;
        std::vector<std::shared_ptr<Node>> nearest_nodes_for_test;

        std::shared_ptr<Node> nearest_node;
        nearest_node = qtree->nearest_neighbor_test(node, range_for_test, first_valid_node, nearest_nodes_for_test);
        std::shared_ptr<Node> brute_force_nearest = bruteForceNearestNeighbor(node, all_nodes);

        if (nearest_node != brute_force_nearest){
            std::cout << "Nearest node is not the same as the brute force nearest node" << std::endl;
            std::cout << "Target node: " << node->x << ", " << node->y << std::endl;
            std::cout << "Nearest node: " << nearest_node->x << ", " << nearest_node->y << std::endl;
            std::cout << "Brute force nearest node: " << brute_force_nearest->x << ", " << brute_force_nearest->y  <<std::endl;

            double distance_from_nearest = sqrt(pow(nearest_node->x - node->x, 2) + pow(nearest_node->y - node->y, 2));
            double distance_from_brute_force = sqrt(pow(brute_force_nearest->x - node->x, 2) + pow(brute_force_nearest->y - node->y, 2));

            std::cout << "Distance from nearest node: " << distance_from_nearest << std::endl;
            std::cout << "Distance from brute force nearest node: " << distance_from_brute_force << "\n" << std::endl;

            assert(distance_from_brute_force == distance_from_nearest);
        }

        cv::Point left_bottom(range_for_test.left, range_for_test.bottom);
        cv::Point right_top(range_for_test.right, range_for_test.top);

        cv::rectangle(img, left_bottom, right_top, cv::Scalar(255, 255, 255), 1);

        // for (auto node : nearest_nodes_for_test) {
        //     cv::circle(img, cv::Point(node->x, node->y), 1, cv::Scalar(0, 0, 255), -1);
        // }
        cv::circle(img, cv::Point(node->x, node->y), 1, cv::Scalar(255, 0, 0), -1);
        cv::circle(img, cv::Point(nearest_node->x, nearest_node->y), 2, cv::Scalar(255, 255, 255), -1);
        // cv::circle(img, cv::Point(first_valid_node->x, first_valid_node->y), 1, cv::Scalar(255, 0, 0), -1);

        // cv::resize(img, img, cv::Size(800, 800));
        // cv::imshow("QuadTree", img);
        // cv::waitKey(0);

        // assert(nearest_node->x == brute_force_nearest->x);
        // assert(nearest_node->y == brute_force_nearest->y);

        img = backgroung_img.clone();
    }
    return 0;
}
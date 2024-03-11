#include "QTree.hpp"
#include <iostream>
#include <random>
#include <opencv2/opencv.hpp>

int generateRandomPoint(int range) {

    std::random_device rd;
    std::mt19937 gen(rd()); // Mersenne Twister generator

    // Define the range for the coordinates
    std::uniform_int_distribution<> dis(0, range-1);

    // Generate random x and y values
    int x = dis(gen);

    // Return a new Point with these random coordinates
    return x;
}

void QTShow(cv::Mat img, const QTree::QuadTree& tree) {

    cv::Point left_bottom(tree.boundary.left, tree.boundary.bottom);
    cv::Point right_top(tree.boundary.right, tree.boundary.top);

    cv::rectangle(img, left_bottom, right_top, cv::Scalar(255, 255, 255), 1);

    if (tree.points.size() > 0) {
        for (auto point : tree.points) {
            cv::circle(img, cv::Point(point.x, point.y), 2, cv::Scalar(0, 255, 0), -1);
        }
    }

    if (tree.divided) {
        QTShow(img, *tree.northeast);
        QTShow(img, *tree.northwest);
        QTShow(img, *tree.southeast);
        QTShow(img, *tree.southwest);
    }
}

int main() {
    QTree::Rectangle boundary(250, 250, 500, 500);
    QTree::QuadTree tree(boundary, 1, 10);

    cv::Mat img = cv::Mat::zeros(501, 501, CV_8UC3);

    for (int i = 0; i < 300; i++) {
        int x = generateRandomPoint(500);
        int y = generateRandomPoint(500);
        QTree::Point point(x, y);
        tree.insert(point);
    }

    QTShow(img, tree);

    cv::resize(img, img, cv::Size(800, 800));
    cv::imshow("QuadTree", img);
    cv::waitKey(0);
    return 0;
}

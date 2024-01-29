#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

struct GridData {
    cv::Point startingGridCell;
    cv::Point goalGridCell;
    cv::Mat gridMap;
    // std::vector<cv::Point> obstacleCells;
    // cv::Size adjustedGridSize; // Changed on new code
};

GridData processImage(const cv::Mat& image, int desiredGridWidth, int desiredGridHeight);
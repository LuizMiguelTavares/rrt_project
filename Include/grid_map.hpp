#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

struct GridData {
    cv::Point startingGridCell;
    cv::Point goalGridCell;
    std::vector<cv::Point> obstacleCells;
    cv::Size adjustedGridSize;
};

GridData processImage(const cv::Mat& image, int desiredGridWidth, int desiredGridHeight);
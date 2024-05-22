#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

struct GridData {
    cv::Point startingGridCell;
    cv::Point goalGridCell;
    cv::Mat gridMap;
};

GridData processImage(const cv::Mat& image, int desiredGridWidth, int desiredGridHeight);
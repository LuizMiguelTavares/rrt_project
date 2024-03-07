// include libraries
#include <vector>
#include <algorithm>
#include <iostream>
#include <opencv2/opencv.hpp>

struct KdTree
{
    KdTree(const std::vector<Point>& points)
    {
        n = points.size();
        m = n // 2;

        // Sort points by x-coordinate
        std::vector<Point> sorted_points = points;
    }
};

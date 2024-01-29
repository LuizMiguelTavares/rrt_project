#include <opencv2/opencv.hpp>
#include "grid_map.hpp"
#include <iostream>
#include <vector>
#include <cmath>

GridData processImage(const cv::Mat& image, int desiredGridWidth, int desiredGridHeight) {
    GridData gridData;

    // Define the color ranges for gray, green, and red (in BGR format)
    cv::Scalar grayLower(145, 145, 145), grayUpper(210, 210, 210);
    cv::Scalar greenLower(0, 100, 0), greenUpper(0, 147, 0);
    cv::Scalar redLower(0, 0, 139), redUpper(75, 75, 255);

    // Vectors to hold the green and red pixels, and obstacle points
    std::vector<cv::Point> greenPixels, redPixels;

    // Create a white image for obstacles
    cv::Mat obstacleImage = cv::Mat::ones(image.size(), CV_8U) * 255;

    // Iterate over the image to find colors
    for(int y = 0; y < image.rows; y++) {
        for(int x = 0; x < image.cols; x++) {
            cv::Vec3b color = image.at<cv::Vec3b>(y, x);

            // Check for obstacles (gray)
            if (color[0] >= grayLower[0] && color[0] <= grayUpper[0] &&
                color[1] >= grayLower[1] && color[1] <= grayUpper[1] &&
                color[2] >= grayLower[2] && color[2] <= grayUpper[2]) {
                obstacleImage.at<uchar>(y, x) = 0;
            }

            // Check for starting point (green)
            if (color[0] >= greenLower[0] && color[0] <= greenUpper[0] &&
                color[1] >= greenLower[1] && color[1] <= greenUpper[1] &&
                color[2] >= greenLower[2] && color[2] <= greenUpper[2]) {
                greenPixels.push_back(cv::Point(x, y));
            }

            // Check for goal point (red)
            if (color[0] >= redLower[0] && color[0] <= redUpper[0] &&
                color[1] >= redLower[1] && color[1] <= redUpper[1] &&
                color[2] >= redLower[2] && color[2] <= redUpper[2]) {
                redPixels.push_back(cv::Point(x, y));
            }
        }
    }

    // Calculate the mean point for the starting point (green)
    cv::Point2f startingPoint;
    for(const auto& pt : greenPixels) {
        startingPoint += cv::Point2f(pt.x, pt.y);
    }
    startingPoint.x /= greenPixels.size();
    startingPoint.y /= greenPixels.size();

    // Calculate the mean point for the goal point (red)
    cv::Point2f goalPoint;
    for(const auto& pt : redPixels) {
        goalPoint += cv::Point2f(pt.x, pt.y);
    }
    goalPoint.x /= redPixels.size();
    goalPoint.y /= redPixels.size();

    // Resize the obstacle image to the desired grid size
    cv::Mat resizedObstacleImage;
    cv::resize(obstacleImage, resizedObstacleImage, cv::Size(desiredGridWidth, desiredGridHeight), 0, 0, cv::INTER_AREA);

    gridData.gridMap = resizedObstacleImage;

    // Scaling factors
    float scaleX = static_cast<float>(desiredGridWidth) / image.cols;
    float scaleY = static_cast<float>(desiredGridHeight) / image.rows;

    // Scale down the coordinates of the starting and goal points
    cv::Point startingGridCell = cv::Point(static_cast<int>(startingPoint.x * scaleX), static_cast<int>(startingPoint.y * scaleY));
    cv::Point goalGridCell = cv::Point(static_cast<int>(goalPoint.x * scaleX), static_cast<int>(goalPoint.y * scaleY));

    // Assign to gridData
    gridData.startingGridCell = startingGridCell;
    gridData.goalGridCell = goalGridCell;

    return gridData;
}

bool check_intersection(const cv::Mat& image, int xBegin, int yBegin, int xEnd, int yEnd) {
    int dx = abs(xEnd - xBegin), sx = xBegin < xEnd ? 1 : -1;
    int dy = -abs(yEnd - yBegin), sy = yBegin < yEnd ? 1 : -1; 
    int error = dx + dy, error2;

    while (true) {
        // Check if the point is within the image boundaries and is an obstacle
        if (xBegin >= 0 && xBegin < image.cols && yBegin >= 0 && yBegin < image.rows) {
            if (image.at<uchar>(yBegin, xBegin) < 255) { // Check for black pixel (obstacle)
                return true;
            }
        }

        if (xBegin == xEnd && yBegin == yEnd) break;
        error2 = 2 * error;
        if (error2 >= dy) { error += dy; xBegin += sx; }
        if (error2 <= dx) { error += dx; yBegin += sy; }
    }

    return false;
}

struct Point
{
    int x;
    int y;
};


std::vector<Point> drawLine(int x1, int y1, int x2, int y2) {
    std::vector<Point> linePoints;
    
    int dx = abs(x2 - x1), sx = x1 < x2 ? 1 : -1;
    int dy = -abs(y2 - y1), sy = y1 < y2 ? 1 : -1; 
    int error = dx + dy, error2;

    while (true) {
        linePoints.push_back({x1, y1});
        if (x1 == x2 && y1 == y2) break;
        error2 = 2 * error;
        if (error2 >= dy) { error += dy; x1 += sx; }
        if (error2 <= dx) { error += dx; y1 += sy; }
    }

    return linePoints;
}

// Macro that holds the directory of your project
#ifndef PROJECT_ROOT_DIR
#define PROJECT_ROOT_DIR ""
#endif

// int main() {
//     // Load the image
//     std::string imagePath = std::string(PROJECT_ROOT_DIR) + "/images/mipui.png";

//     cv::Mat image = cv::imread(imagePath);

//     // Check if the image was loaded
//     if(image.empty()) {
//         std::cerr << "Could not open or find the image" << std::endl;
//         return -1;
//     }

//     // Desired grid size (modify these values as needed)
//     int desiredGridWidth = 350; // Desired number of cells horizontally
//     int desiredGridHeight = 350; // Desired number of cells vertically

//     GridData gridData = processImage(image, desiredGridWidth, desiredGridHeight);

//     // Output the results
//     std::cout << "Starting Grid Cell: (" << gridData.startingGridCell.x << ", " << gridData.startingGridCell.y << ")" << std::endl;
//     std::cout << "Goal Grid Cell: (" << gridData.goalGridCell.x << ", " << gridData.goalGridCell.y << ")" << std::endl;

//     int x2 = 300, y2 = 300;
//     int x1 = 200, y1 = 300;

//     std::vector<Point> line = drawLine(x1, y1, x2, y2);

//     cv::Mat gridImage = gridData.gridMap;

//     bool intersects = check_intersection(gridImage, x1, y1, x2, y2);

//     std::cout << "Intersects: " << intersects << std::endl;
    
//     // Draw the points on the image
//     for (const Point& p : line) {
//         gridImage.at<uchar>(p.y, p.x) = 0;
//     }
//     // Draw the starting point as a green circle
//     cv::circle(gridImage, gridData.startingGridCell, 5, 0, cv::FILLED);
//     // Draw the goal point as a red circle
//     cv::circle(gridImage, gridData.goalGridCell, 5, 0, cv::FILLED);

//     // Optionally, display the images
//     cv::imshow("Original Image", image);
//     cv::imshow("Grid Image", gridImage);

//     cv::waitKey(0);

//     return 0;
// }
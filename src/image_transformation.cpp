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
    std::vector<cv::Point> greenPixels, redPixels, obstaclePoints;

    // Iterate over the image to find colors
    for(int y = 0; y < image.rows; y++) {
        for(int x = 0; x < image.cols; x++) {
            cv::Vec3b color = image.at<cv::Vec3b>(y, x);

            // Check for obstacles (gray)
            if (color[0] >= grayLower[0] && color[0] <= grayUpper[0] &&
                color[1] >= grayLower[1] && color[1] <= grayUpper[1] &&
                color[2] >= grayLower[2] && color[2] <= grayUpper[2]) {
                obstaclePoints.push_back(cv::Point(x, y));
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

    // Image dimensions
    int imageWidth = image.cols;
    int imageHeight = image.rows;

    // Calculate initial cell size
    float cellWidth_ = static_cast<float>(imageWidth) / desiredGridWidth;
    float cellHeight_ = static_cast<float>(imageHeight) / desiredGridHeight;

    // Find nearest multiple that is larger than or equal to the image dimensions
    cellWidth_ = std::ceil(cellWidth_);
    cellHeight_ = std::ceil(cellHeight_);

    // Recalculate the number of cells based on the adjusted cell size
    int gridWidth = std::ceil(static_cast<float>(imageWidth) / cellWidth_);
    int gridHeight = std::ceil(static_cast<float>(imageHeight) / cellHeight_);

    gridData.adjustedGridSize = cv::Size(gridWidth, gridHeight);

    // Populate the grid with obstacle data
    std::vector<std::vector<int>> grid(gridHeight, std::vector<int>(gridWidth, 0)); // Initialize grid

    // Calculate the size of each cell
    int cellWidth = image.cols / gridWidth;
    int cellHeight = image.rows / gridHeight;

    for(const auto& pt : obstaclePoints) {
        int gridX = pt.x / cellWidth;
        int gridY = pt.y / cellHeight;
        grid[gridY][gridX]++;
    }

    // Determine if a cell is an obstacle based on the percentage of gray pixels
    for(int y = 0; y < gridHeight; y++) {
        for(int x = 0; x < gridWidth; x++) {
            // If more than 40% of the cell is covered by obstacles, mark it as an obstacle cell
            if(grid[y][x] > (cellWidth * cellHeight * 0.4)) {
                gridData.obstacleCells.push_back(cv::Point(x, y));
            }
        }
    }

    // Determine the grid cell for the starting and goal points
    gridData.startingGridCell = cv::Point(static_cast<int>(startingPoint.x / cellWidth), static_cast<int>(startingPoint.y / cellHeight));
    gridData.goalGridCell = cv::Point(static_cast<int>(goalPoint.x / cellWidth), static_cast<int>(goalPoint.y / cellHeight));

    return gridData;
}

struct Point {
    int x, y;
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

int main() {
    // Load the image
    std::string imagePath = std::string(PROJECT_ROOT_DIR) + "/images/mipui.png";

    cv::Mat image = cv::imread(imagePath);

    // Check if the image was loaded
    if(image.empty()) {
        std::cerr << "Could not open or find the image" << std::endl;
        return -1;
    }

    // Desired grid size (modify these values as needed)
    int desiredGridWidth = 350; // Desired number of cells horizontally
    int desiredGridHeight = 350; // Desired number of cells vertically

    GridData gridData = processImage(image, desiredGridWidth, desiredGridHeight);

    // Output the results
    std::cout << "Starting Grid Cell: (" << gridData.startingGridCell.x << ", " << gridData.startingGridCell.y << ")" << std::endl;
    std::cout << "Goal Grid Cell: (" << gridData.goalGridCell.x << ", " << gridData.goalGridCell.y << ")" << std::endl;
    std::cout << "Number of Obstacle Cells: " << gridData.obstacleCells.size() << std::endl;
    std::cout << "Adjusted Grid Width (Cells): " << gridData.adjustedGridSize.width << std::endl;
    std::cout << "Adjusted Grid Height (Cells): " << gridData.adjustedGridSize.height << std::endl;

    cv::Mat gridImage(image.rows, image.cols, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::Mat newgridImage(gridData.adjustedGridSize.width, gridData.adjustedGridSize.height, CV_8UC3, cv::Scalar(255, 255, 255));

    int cellWidth = image.cols / gridData.adjustedGridSize.width;
    int cellHeight = image.rows / gridData.adjustedGridSize.height;

    // Draw the grid on the blank image
    for(int y = 0; y < gridData.adjustedGridSize.height; y++) {
        for(int x = 0; x < gridData.adjustedGridSize.width; x++) {
            int topLeftX = x * cellWidth;
            int topLeftY = y * cellHeight;
            // If the cell is an obstacle, draw it
            if(std::find(gridData.obstacleCells.begin(), gridData.obstacleCells.end(), cv::Point(x, y)) != gridData.obstacleCells.end()) {
                cv::rectangle(gridImage, cv::Point(topLeftX, topLeftY), cv::Point(topLeftX + cellWidth, topLeftY + cellHeight), cv::Scalar(50, 50, 50), cv::FILLED);
            }
        }
    }

     // Draw the grid on the blank image
    for(int y = 0; y < gridData.adjustedGridSize.height; y++) {
        for(int x = 0; x < gridData.adjustedGridSize.width; x++) {
            int topLeftX = x;
            int topLeftY = y;
            // If the cell is an obstacle, draw it
            if(std::find(gridData.obstacleCells.begin(), gridData.obstacleCells.end(), cv::Point(x, y)) != gridData.obstacleCells.end()) {
                newgridImage.at<cv::Vec3b>(topLeftY, topLeftX) = cv::Vec3b(50, 50, 50);
                // cv::rectangle(newgridImage, cv::Point(topLeftX, topLeftY), cv::Point(topLeftX + cellWidth, topLeftY + cellHeight), cv::Scalar(50, 50, 50), cv::FILLED);
            }
        }
    }

    cv::resize(newgridImage, newgridImage, cv::Size(image.cols, image.rows));

    int x2 = 300, y2 = 400;
    int x1 = 500, y1 = 500;

    std::vector<Point> line = drawLine(x1, y1, x2, y2);

    std::cout << "Point: " << newgridImage.at<cv::Vec3b>(y1, x1) << std::endl; 

    // Draw the points on the image
    for (const Point& p : line) {
        //cv::circle(image, cv::Point(p.x, p.y), 1, cv::Scalar(255, 0, 0), -1);
        //Alternatively, set the pixel directly: 
        newgridImage.at<cv::Vec3b>(p.y, p.x) = cv::Vec3b(255, 0, 0);
    }
    // Draw the starting point as a green circle
    cv::circle(gridImage, cv::Point(static_cast<int>(gridData.startingGridCell.x * cellWidth), static_cast<int>(gridData.startingGridCell.y * cellHeight)), 5, cv::Scalar(0, 255, 0), cv::FILLED);
    // Draw the goal point as a red circle
    cv::circle(gridImage, cv::Point(static_cast<int>(gridData.goalGridCell.x * cellWidth), static_cast<int>(gridData.goalGridCell.y * cellHeight)), 5, cv::Scalar(0, 0, 255), cv::FILLED);

    // Optionally, display the images
    cv::imshow("Original Image", image);
    cv::imshow("Grid Image", newgridImage);

    cv::waitKey(0);

    return 0;
}
#include <opencv2/opencv.hpp>
#include "grid_map.hpp"
#include <iostream>
#include <vector>
#include <cmath>

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

    int x2 = 300, y2 = 300;
    int x1 = 200, y1 = 300;

    std::vector<Point> line = drawLine(x1, y1, x2, y2);

    cv::Mat gridImage = gridData.gridMap;

    bool intersects = check_intersection(gridImage, x1, y1, x2, y2);

    std::cout << "Intersects: " << intersects << std::endl;
    
    // Draw the points on the image
    for (const Point& p : line) {
        gridImage.at<uchar>(p.y, p.x) = 0;
    }
    // Draw the starting point as a green circle
    cv::circle(gridImage, gridData.startingGridCell, 5, 0, cv::FILLED);
    // Draw the goal point as a red circle
    cv::circle(gridImage, gridData.goalGridCell, 5, 0, cv::FILLED);

    // Optionally, display the images
    cv::imshow("Original Image", image);
    cv::imshow("Grid Image", gridImage);

    cv::waitKey(0);

    return 0;
}
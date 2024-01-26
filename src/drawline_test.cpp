#include <iostream>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>

std::vector<cv::Point> drawLine(int x1, int y1, int x2, int y2) {
    std::vector<cv::Point> linePoints;
    
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

int main() {
    // Line coordinates
    int x2 = 1, y2 = 1;
    int x1 = 50, y1 = 30;
    
    // Create a black image
    cv::Mat image = cv::Mat::zeros(100, 100, CV_8UC3);

    y1 = image.rows - y1;
    y2 = image.rows - y2;

    std::vector<cv::Point> line = drawLine(x1, y1, x2, y2);

    // Draw the points on the image
    for (const cv::Point& p : line) {
        //cv::circle(image, cv::Point(p.x, p.y), 1, cv::Scalar(255, 0, 0), -1);
        //Alternatively, set the pixel directly: 
        image.at<cv::Vec3b>(p.y, p.x) = cv::Vec3b(255, 0, 0);
    }

    // Display the image
    cv::resize(image, image, cv::Size(500, 500));
    cv::imshow("Line", image);
    cv::waitKey(0); // Wait for a key press

    // Optionally save the image
    cv::imwrite("line.jpg", image);

    return 0;
}

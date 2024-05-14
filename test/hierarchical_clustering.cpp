// #include <mlpack/methods/hierarchical_clustering/hierarchical_clustering.hpp>
#include "/home/dev_container/mlpack-4.3.0/src/mlpack/methods/ann.hpp"
#include <mlpack/core.hpp>
#include <opencv2/opencv.hpp>
#include <random>
#include <iostream>

int main()
{
    // Generate random data
    const int points = 100;
    const int clusters = 3;
    const int dimension = 2;
    mlpack::arma::mat data(dimension, points);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> d(0,1);

    for (int i = 0; i < clusters; ++i) {
        for (int j = 0; j < points / clusters; ++j) {
            data(0, (i * (points / clusters)) + j) = d(gen) + i * 3;
            data(1, (i * (points / clusters)) + j) = d(gen) + i * 3;
        }
    }

    // Perform hierarchical clustering
    mlpack::hclust::HierarchicalClustering<> hclust(data);
    arma::Row<size_t> assignments;
    hclust.Cluster(assignments);

    // Visualize the result with OpenCV
    const int width = 500;
    const int height = 500;
    cv::Mat image(height, width, CV_8UC3, cv::Scalar(255, 255, 255));

    auto colors = std::vector<cv::Scalar>{
        cv::Scalar(255, 0, 0),
        cv::Scalar(0, 255, 0),
        cv::Scalar(0, 0, 255),
        cv::Scalar(0, 255, 255)
    };

    for (size_t i = 0; i < assignments.n_elem; ++i) {
        int x = static_cast<int>(data(0, i) * 50 + width / 2);
        int y = static_cast<int>(data(1, i) * 50 + height / 2);
        cv::circle(image, cv::Point(x, y), 5, colors[assignments[i] % colors.size()], CV_FILLED);
    }

    cv::imshow("Clusters", image);
    cv::waitKey(0);
    return 0;
}
#include "rrt.hpp"
#include "grid_map.hpp"
#include <vector>
#include <iostream>
#include <string>
#include <memory>

int main() {

    std::string imagePath = std::string(PROJECT_ROOT_DIR) + "/images/second_room.png";

    cv::Mat image = cv::imread(imagePath);

    double goal_threshold = 10;
    int num_nodes = 10000;
    double step_size = 10;
    double bias_probability = 0.05;

    // Process the image
    GridData grid_data = processImage(image, 300, 300);

    //QTree::Point start((double)grid_data.startingGridCell.x, (double)grid_data.startingGridCell.y);
    std::unique_ptr<motion_planning::Node> start = std::make_unique<motion_planning::Node>(std::vector<double>{(double)grid_data.startingGridCell.x, (double)grid_data.startingGridCell.y}, nullptr);
    std::unique_ptr<motion_planning::Node> goal = std::make_unique<motion_planning::Node>(std::vector<double>{(double)grid_data.goalGridCell.x, (double)grid_data.goalGridCell.y}, nullptr);

    cv::Mat grid_map = grid_data.gridMap;

    auto start_time = std::chrono::high_resolution_clock::now();
    motion_planning::RRT rrt(grid_map, start.get(), goal.get(), num_nodes, step_size, goal_threshold, bias_probability);
    std::vector<motion_planning::Node*> nodes = rrt.run(); 

    // Calculate time taken
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    std::cout << "Time taken by function: " << duration << " milliseconds" << std::endl;
    
    // Plot the RRT
    rrt.plot(nodes, start.get(), goal.get(), true);

    std::cout << "Finished!" << std::endl;
    return 0;
}
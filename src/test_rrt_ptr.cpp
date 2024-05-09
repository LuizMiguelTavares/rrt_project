#include "rrt_simple_ptr.hpp"
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
    int radius = 1;

    // Process the image
    GridData grid_data = processImage(image, 300, 300);

    //QTree::Point start((double)grid_data.startingGridCell.x, (double)grid_data.startingGridCell.y);
    std::shared_ptr<motion_planning::Node> start = std::make_unique<motion_planning::Node>(grid_data.startingGridCell.x, grid_data.startingGridCell.y, nullptr);
    std::shared_ptr<motion_planning::Node> goal = std::make_unique<motion_planning::Node>(grid_data.goalGridCell.x, grid_data.goalGridCell.y, nullptr);

    cv::Mat grid_map = grid_data.gridMap;

    auto start_time = std::chrono::high_resolution_clock::now();
    std::vector<std::shared_ptr<motion_planning::Node>> nodes;
    for(int i=0;i<1000;i++){
        nodes = motion_planning::rrt(grid_map, start, goal, num_nodes, step_size, goal_threshold, bias_probability, radius);

        // Calculate time taken
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        std::cout << "Time taken by function: " << duration << " milliseconds" << std::endl;
        
        // Plot the RRT
        if (nodes.empty()) {
            std::cout << "No path found!" << std::endl;
            return 0;
        }

        if (nodes.back()->x != goal->x || nodes.back()->y != goal->y) {
            std::cout << "No path found!" << std::endl;
            // motion_planning::plot_rrt(grid_map, start, goal, false, nodes);
            return 0;
        } else {
            
            // motion_planning::plot_rrt(grid_map, start, goal, true, nodes);
            nodes = motion_planning::trace_goal_path(nodes.back());
            // motion_planning::plot_rrt(grid_map, start, goal, true, nodes);
            std::cout << "Path found!" << std::endl;
            std::cout << "Number of nodes: " << nodes.size() << std::endl;
        }
    }
    std::cout << "Finished!" << std::endl;
    return 0;
}
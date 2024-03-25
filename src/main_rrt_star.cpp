/**
 *  This is the main file that calls the RRT* algorithm. 
 *  First, the algorithm generates a plan (vector of points) which is a first viable solution.
 *  Next, it calls the RRT* algorithm on the previouslly built plan to optimize it.
 */


// TODOs:

// MPI, OpenMP(Pthread), CUDA

// 0. Add openmp flag!
// 1. Measure time & Parallelize
// 2. Compare sequential time & Parallelize time
// 3. Generate different test cases. (Size of the map)
// 4. Visualize random exploration points. (Visible point)

#include"rrt_star.hpp"
#include "QTree.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>
#include "grid_map.hpp"
#include <chrono>
#include"omp.h"

#define TIME

int main(int argc, char** argv){
    #ifdef TIME 
        float total_time = 0;
        float starttime, endtime;
        starttime = omp_get_wtime();
    #endif

    std::string imagePath = std::string(PROJECT_ROOT_DIR) + "/images/second_room.png"; // min cost = 636.254

    cv::Mat image = cv::imread(imagePath);

    GridData grid_data = processImage(image, 300, 300);

    Point start_pos(grid_data.startingGridCell.x, grid_data.startingGridCell.y);
    Point end_pos(grid_data.goalGridCell.x, grid_data.goalGridCell.y);

    cv::Mat grid_map = grid_data.gridMap;

    Node* start_node = new Node();
    start_node->set_position(start_pos);

    QTree::Rectangle boundary(grid_map.cols/2, grid_map.rows/2, grid_map.cols, grid_map.rows);
    QTree::QuadTree<Node>* tree = new QTree::QuadTree<Node>(boundary, start_node, 4);

    //define the raduis for RRT* algorithm (Within a radius of r, RRT* will find all neighbour nodes of a new node).
    float rrt_radius = 5;
    //define the radius to check if the last node in the tree is close to the end position
    float end_thresh = 5;
    //
    float step_size = 5;
    int max_iter = 200000;

    //instantiate RRTSTAR class
    // Point start_pos, Point end_pos, float radius, float end_thresh, float step_size = 10, int max_iter = 5000, std::pair<float, float> map_size= std::make_pair(10.0f, 10.0f)
    RRTSTAR* rrtstar = new RRTSTAR(start_pos, end_pos, rrt_radius, end_thresh, grid_map, step_size, max_iter, tree);
    
    std::cout << "Starting RRT* Algorithm..." << std::endl;
    //search for the first viable solution
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<Point> initial_solution =rrtstar->planner();
    auto end = std::chrono::high_resolution_clock::now();
    
    //save initial solution
    // rrtstar->savePlanToFile(initial_solution, FIRST_PATH_FILE, "First viable solution . This file contains vector of points of the generated path.");
    if (!initial_solution.empty()) {
        std::cout << "First Viable Solution Obtained after " << rrtstar->getCurrentIterations() << " iterations" << std::endl;
        std::cout << "Cost is " << rrtstar->lastnode->cost << std::endl;
        std::cout << "Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
        std::cout << "Saving the generated plan (vector of points)" << std::endl;
        //rrtstar->plotBestPath();
    }
    std::vector<Point> optimized_solution;
    //search for the optimized paths
    while (rrtstar->getCurrentIterations() < rrtstar->getMaxIterations() && !initial_solution.empty())
    {
        std::cout << "=========================================================================" << std::endl;
        std::cout << "The algorithm continues iterating on the current plan to improve the plan" << std::endl;
        start = std::chrono::high_resolution_clock::now();
        optimized_solution = rrtstar->planner();
        end = std::chrono::high_resolution_clock::now();
        std::cout << "More optimal solution has obtained after " << rrtstar->getCurrentIterations() << " iterations" << std::endl;
        std::cout << "Cost is " << rrtstar->m_cost_bestpath << std::endl;
        std::cout << "Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
        //rrtstar->plotBestPath();
        
    }
    cv::waitKey(0);
    //save optimized solution
    // "Mfiles//Path_after_MAX_ITER.txt"
    // rrtstar->savePlanToFile(optimized_solution, OPTIMIZE_PATH_FILE, " Optimized Solution after maximum provided iteration.This file contains vector of points of the generated path.");
    if (!optimized_solution.empty()) {
        std::cout << "Exceeded max iterations!" << std::endl;
        std::cout << "Saving the generated plan (vector of points)" << std::endl;
    }
   
    // Save "Avalible point! .txt"
    // rrtstar->savePlanToFile(rrtstar->get_available_points(), AVAILABLE_PATH_FILE, "Saved a vector of reachable workspace points.");

    #ifdef TIME 
        endtime = omp_get_wtime();
        total_time = endtime - starttime;
        printf("Execution time: %.6f seconds\n",total_time);
    #endif

    //free up the memory
    delete rrtstar;
    delete start_node;
    delete tree;
}
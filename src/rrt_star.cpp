#include"rrt_star.hpp"
#include <iostream>
#include <cfloat>
#include <algorithm>
#include <vector>
#include <opencv2/opencv.hpp>
#include <thread>
#include <atomic>
#include <memory>
#include "omp.h"

namespace rrt_star{
    //Point class Constructors
    Point::Point()
        :m_x(FLT_MAX), m_y(FLT_MAX) {}
    Point::Point(float X, float Y)
        : m_x(X), m_y(Y) {}

    Node::Node() {
        parent= nullptr;
        cost=0;
    }

    void Node::set_position(Point pos){
        this->position = pos;
        this->x = pos.m_x;
        this->y = pos.m_y;
    }

    //RRTStar class Constructors
    RRTStar::RRTStar() : 
        m_step_size(DEFAULT_STEP_SIZE), m_max_iter(DEFAULT_MAX_ITER), m_destination_threshhold(DEFAULT_DESTINATION_THRESHHOLD), m_rrstar_radius(DEFAULT_RRTSTAR_RADIUS),
        m_num_itr(0), m_cost_bestpath(0), startPoint(Point()), destination(Point()) {}

    //RRTStar methods
    std::vector<Point> RRTStar::planner() {
        
        constexpr bool debug = false;
        
        if (this->m_map.rows == 0 || this->m_map.cols == 0) {
            std::cerr << "Map not set!" << std::endl;
            return {};
        }

        if (this->startPoint.m_x == FLT_MAX || this->startPoint.m_y == FLT_MAX) {
            std::cerr << "Start point not set!" << std::endl;
            return {};
        }

        if (this->destination.m_x == FLT_MAX || this->destination.m_y == FLT_MAX) {
            std::cerr << "Destination not set!" << std::endl;
            return {};
        }

        if (debug) {
            std::cout << "Map Size: " << this->m_map.rows << "x" << this->m_map.cols << std::endl;
            std::cout << "Start Point: " << this->startPoint.m_x << ", " << this->startPoint.m_y << std::endl;
            std::cout << "Destination: " << this->destination.m_x << ", " << this->destination.m_y <<std::endl;

            std::cout << "Step Size: " << this->m_step_size << std::endl;
            std::cout << "Max Iterations: " << this->m_max_iter << std::endl;
            std::cout << "Destination Threshold: " << this->m_destination_threshhold << std::endl;
            std::cout << "RRT* Radius: " << this->m_rrstar_radius << "\n" << std::endl;
        }

        if (m_num_itr == 0) {
            this->qtree->insert(this->root);
        }

        while (this->m_num_itr < this->m_max_iter)
        {   
            // std::cout << "Iteration: " << this->m_num_itr << std::endl;
            this->m_num_itr++;
            std::shared_ptr<Node> plan_n_rand = this->getRandomNode(); //Generate a random node

            if (debug) {
                std::cout << "Random Node: " << plan_n_rand->position.m_x << ", " << plan_n_rand->position.m_y << std::endl;
            }
            if (plan_n_rand->position.m_x!=FLT_MAX && plan_n_rand->position.m_y!=FLT_MAX) {
                    std::shared_ptr<Node> plan_n_nearest = this->qtree->nearest_neighbor(plan_n_rand);

                    if (debug){
                        std::cout << "Nearest Node: " << plan_n_nearest->position.m_x << ", " << plan_n_nearest->position.m_y << std::endl;
                    }

                    Point plan_p_new = this->steer(*plan_n_rand, plan_n_nearest); //Steer from "N_Nearest" towards "N_rand": interpolate if node is too far away.

                    if (debug) {
                        std::cout << "Steered Node: " << plan_p_new.m_x << ", " << plan_p_new.m_y << std::endl;
                    }

                    bool intersection = this->check_obstacle_intersection(this->m_map, plan_n_nearest->position.m_x, plan_n_nearest->position.m_y, plan_p_new.m_x, plan_p_new.m_y, this->m_rrstar_radius);
                
                if (!intersection) { // Check obstacle
                            std::shared_ptr<Node> plan_n_new = std::make_shared<Node>(); //create new node to store the position of the steered node.
                            plan_n_new->set_position(plan_p_new); //set the position of the new node

                            std::vector<std::shared_ptr<Node>> plan_v_n_near; //create a vector for neighbor nodes
                            QTree::Rectangle nn(plan_n_new->x, plan_n_new->y, m_rrstar_radius*2, m_rrstar_radius*2);
                            this->qtree->query(nn, plan_v_n_near);

                            if (debug) {
                                std::cout << "Number of Neighbors: " << plan_v_n_near.size() << std::endl;
                            }
                            
                            this->qtree->insert(plan_n_new); // Insert the new node to the quadtree

                            std::shared_ptr<Node> plan_n_parent=this->findParent(plan_v_n_near,plan_n_nearest,plan_n_new); //Find the parent of the given node (the node that is near and has the lowest path cost)
                            this->insertNode(plan_n_parent, plan_n_new);//Add N_new to node list.
                            this->reWire(plan_n_new, plan_v_n_near); //rewire the tree

                            if (this->reached() && this->bestpath.empty()) { //find the first viable path
                                return this->generatePlan(this->lastnode);
                            }

                            if (!this->bestpath.empty()) { //find more optimal paths
                                if (this->reached()) {
                                    // If we get a better path (lower cost)!
                                    if (this->lastnode->cost < this->m_cost_bestpath) {
                                        return this->generatePlan(this->lastnode);
                                    }
                                }
                                else {
                                    // Havent reach the goal, ??? Why here.
                                    std::shared_ptr<Node> Plan_NearNodeEnd = this->findNearest(this->destination);
                                    if (Plan_NearNodeEnd->cost < this->m_cost_bestpath) {
                                        return this->generatePlan(Plan_NearNodeEnd);
                                    }
                                }
                            } 
                    }            
            }
            if (debug) {
                std::cout << "\n" << "---------------------------------" << "\n" << std::endl;
            
            }
        }
        
        if (this->bestpath.empty()) {
            // if not reached yet, no solution has found
            std::cout << "Exceeded max iterations!" << std::endl;
            std::cout << "Error: No solution found" << std::endl;

            //Debugging
            // std::shared_ptr<Node> Plan_NearNodeEnd = this->findNearest(this->destination);
            // std::cout << "Number of nodes: " << this->nodes.size() << std::endl;
            // std::ofstream outfile("/home/miguel/cba_ws/src/rrt_project/src/nodes.csv");

            // if (outfile.is_open()) {
            //     outfile << "x,y" << std::endl;
            //     for (const auto& node : this->nodes) {
            //         outfile << node->position.m_x << "," << node->position.m_y << std::endl;
            //     }
            //     outfile.close();
            // } else {
            //     std::cerr << "Failed to open nodes.csv" << std::endl;
            // }
            // return this->generatePlan(Plan_NearNodeEnd);

            // Finish Debugging
            return {};
        }
        else {
            return RRTStar::planFromBestPath(); //after reaching the maximum iteration number retun the path that has the lowest cost.
        }     
    }

    std::shared_ptr<Node> RRTStar::getRandomNode() {
        std::random_device rand_rd;
        std::mt19937 rand_gen(rand_rd());
        std::uniform_real_distribution<> rand_unif(0, 1.0);
        
        // Generate random float coordinates
        float rand_x = rand_unif(rand_gen) * this->m_map.rows;
        float rand_y = rand_unif(rand_gen) * this->m_map.cols;
        
        // Convert to integer coordinates
        int rand_x_int = static_cast<int>(rand_x);
        int rand_y_int = static_cast<int>(rand_y);
        
        if (rand_x_int >= 0 && rand_x_int < this->m_map.rows && rand_y_int >= 0 && rand_y_int < this->m_map.cols) {
            std::shared_ptr<Node> rand_randomnode = std::make_shared<Node>();
            rand_randomnode->set_position(Point(rand_x_int, rand_y_int));
            return rand_randomnode;
        }
        return {};
    }

    float RRTStar::getBestPathCost() {
        return this->m_cost_bestpath;
    }

    std::shared_ptr<Node> RRTStar::findNearest(const Point point) {
        std::shared_ptr<Node> local_closest[omp_get_max_threads()]; 
        float local_minDist[omp_get_max_threads()];

        #pragma omp parallel
        {
            int tid = omp_get_thread_num();
            local_closest[tid] = NULL;
            local_minDist[tid] = FLT_MAX;

            #pragma omp for
            for (size_t i = 0; i < this->nodes.size(); i++) {
                float fn_dist = this->distance(point, this->nodes[i]->position);
                if (fn_dist < local_minDist[tid]) {
                    local_minDist[tid] = fn_dist;
                    local_closest[tid] = this->nodes[i];
                }
            }
        }

        std::shared_ptr<Node> fn_closest = NULL;
        float fn_minDist = FLT_MAX;
        for (int i = 0; i < omp_get_max_threads(); i++) {
            if (local_minDist[i] < fn_minDist) {
                fn_minDist = local_minDist[i];
                fn_closest = local_closest[i];
            }
        }

        return fn_closest;
    }

    void RRTStar::findNearNeighbors(const Point point, const float radius, std::vector<std::shared_ptr<Node>>& neighbor_nodes) { // Find neighbor nodes of the given node within the defined radius
        for (size_t i = 0; i < this->nodes.size(); i++) { //iterate through all nodes to see which ones fall inside the circle with the given radius.
            if (this->distance(point, this->nodes[i]->position) < radius) {
                neighbor_nodes.push_back(this->nodes[i]);
            }
        }
    }

    float RRTStar::distance(const Point p, const Point q) { //Find the distance between two points.
        Point dist_v = p - q;
        return sqrt(powf(dist_v.m_x, 2) + powf(dist_v.m_y, 2));
    }

    float RRTStar::getCost(const std::shared_ptr<Node> N) { //get the cost current node (traveling from the given node to the root)
        return N->cost;
    }

    float RRTStar::pathCost(const std::shared_ptr<Node> Np, const std::shared_ptr<Node> Nq) { //Compute the distance between the position of two nodes
        return this->distance(Nq->position, Np->position);
    }

    Point RRTStar::steer(const Node& n_rand, const std::shared_ptr<Node> n_nearest) { // Steer from new node towards the nearest neighbor and interpolate if the new node is too far away from its neighbor
        constexpr bool debug_steer = false;

        if (debug_steer) {
            std::cout << "Steering from " << n_nearest->position.m_x << ", " << n_nearest->position.m_y << " to " << n_rand.position.m_x << ", " << n_rand.position.m_y << std::endl;
        }

        if (this->distance(n_rand.position, n_nearest->position) >this->m_step_size) { //check if the distance between two nodes is larger than the maximum travel step size

            if (debug_steer) {
                std::cout << "Steering with step size: " << this->m_step_size << std::endl;
            }

            Point steer_p = n_rand.position - n_nearest->position;

            if (debug_steer) {
                std::cout << "Steer Vector: " << steer_p.m_x << ", " << steer_p.m_y << std::endl;
            }
            double steer_norm = this->distance(n_rand.position, n_nearest->position);

            if (debug_steer) {
                std::cout << "Steer Norm: " << steer_norm << std::endl;
            }

            steer_p = steer_p / steer_norm; //normalize the vector

            if (debug_steer) {
                std::cout << "Normalized Steer Vector: " << steer_p.m_x << ", " << steer_p.m_y << std::endl;
                // std::cout << "Stered Node: " << n_nearest->position + this->m_step_size * steer_p << std::endl;
            }

            Point steer_node = n_nearest->position;
            steer_node.m_x += this->m_step_size * steer_p.m_x;
            steer_node.m_y += this->m_step_size * steer_p.m_y;

            if (debug_steer) {
                std::cout << "Steered Node: " << steer_node.m_x << ", " << steer_node.m_y << std::endl;
            }

            return steer_node; //travel in the direction of line between the new node and the near node
            // return (n_nearest->position + this->m_step_size * steer_p); //travel in the direction of line between the new node and the near node
        }
        else {
            return  (n_rand.position);
        }
    }

    std::shared_ptr<Node> RRTStar::findParent(std::vector<std::shared_ptr<Node>> v_n_near,std::shared_ptr<Node> n_nearest, std::shared_ptr<Node> n_new) {
        std::shared_ptr<Node> fp_n_parent = n_nearest; //create new note to find the parent
        float fp_cmin = this->getCost(n_nearest) + this->pathCost(n_nearest, n_new); // Update cost of reaching "N_new" from "N_Nearest"
        for (size_t j = 0; j < v_n_near.size(); j++) { //In all members of "N_near", check if "N_new" can be reached from a different parent node with cost lower than Cmin, and without colliding with the obstacle.
            std::shared_ptr<Node> fp_n_near = v_n_near[j];
            if ((true) &&
                (this->getCost(fp_n_near) + this->pathCost(fp_n_near, n_new)) < fp_cmin) {
                fp_n_parent = fp_n_near; // a near node with minimun cost of path
                fp_cmin = this->getCost(fp_n_near) + this->pathCost(fp_n_near, n_new); //update the cost of path
            }
        }
        return fp_n_parent;
    }

    void RRTStar::insertNode(std::shared_ptr<Node> n_parent, std::shared_ptr<Node> n_new) { //Append the new node to the tree.
        n_new->parent = n_parent; //update the parent of new node
        n_new->cost = n_parent->cost + this->pathCost(n_parent, n_new);//update the cost of new node
        n_parent->children.push_back(n_new); //update the children of the nearest node to the new node
        this->nodes.push_back(n_new);//add the new node to the tree

        this->lastnode = n_new;//inform the tree which node is just added
    }

    void RRTStar::reWire(std::shared_ptr<Node> n_new, std::vector<std::shared_ptr<Node>>& neighbor_nodes) { // Rewire the tree to decrease the cost of the path. 
        for (size_t j = 0; j < neighbor_nodes.size(); j++) {  // Search through nodes in "N_near" and see if changing their parent to "N_new" lowers the cost of the path. Also check the obstacles
            std::shared_ptr<Node> rw_n_near = neighbor_nodes[j];
            if ((true) &&
                (this->getCost(n_new) + this->pathCost(n_new, rw_n_near)) < this->getCost(rw_n_near)) {
                std::shared_ptr<Node> rw_n_parent = rw_n_near->parent;
                float rw_costdifference = this->getCost(rw_n_near) - (this->getCost(n_new) + this->pathCost(n_new, rw_n_near)); //calculate the cost  by which the cost of all children of the near node must decrease
                // Remove branch between N_Parent and N_Near
                
                rw_n_parent->children.erase(std::remove(rw_n_parent->children.begin(), rw_n_parent->children.end(), rw_n_near), rw_n_parent->children.end());
                // Add branch between N_New and N_Near
                rw_n_near->cost = this->getCost(n_new) + this->pathCost(n_new, rw_n_near);
                rw_n_near->parent = n_new;
                n_new->children.push_back(rw_n_near);
                this->updateChildrenCost(rw_n_near, rw_costdifference);// Update the cost of all children of the near node 
            }
        }
    }

    // Here can I parallelize TOO.
    void RRTStar::updateChildrenCost(std::shared_ptr<Node> n, const float costdifference) {//Update the cost of all children of a node after rewiring 
        for (size_t i = 0; i < n->children.size(); i++)
        {
            n->children[i]->cost = n->children[i]->cost - costdifference;
            this->updateChildrenCost(n->children[i], costdifference); //recursive function. call it self to go through all children of the given node.
        }
    }

    bool RRTStar::reached() { //check if the last node in the tree is close to the end position.
        if (this->distance(this->lastnode->position, this->destination) < m_destination_threshhold) {
            return true;
        }
        return false;
    }

    void RRTStar::setStartPoint(const float x, const float y){
        if (this->startPoint.m_x != FLT_MAX && this->startPoint.m_y != FLT_MAX) {
            std::cerr << "Start point already set!" << std::endl;
            return;
        }

        this->root = std::make_shared<Node>();
        root->parent = nullptr;
        this->startPoint = Point(x, y);
        root->position = startPoint;
        root->cost = 0.0;
        this->lastnode = root;
        this->nodes.clear();
        nodes.push_back(this->root);
    }

    void RRTStar::setDestination(const float x, const float y) {
        if (this->destination.m_x != FLT_MAX && this->destination.m_y != FLT_MAX) {
            std::cerr << "Destination already set!" << std::endl;
            return;
        }

        this->destination = Point(x, y);
    }

    void RRTStar::setMap(const cv::Mat map) {
        if (this->m_map.rows != 0 && this->m_map.cols != 0) {
            std::cerr << "Map already set!" << std::endl;
            return;
        }

        this->m_map = map;

        QTree::Rectangle boundary((map.cols)/2, (map.rows)/2, map.cols, map.rows);
        qtree = std::make_shared<QTree::QuadTree<Node>>(boundary);
    }

    void RRTStar::setStepSize(const float step) { // set the step size (the maximum distance between two nodes) for the RRT* algorithm
        this->m_step_size = step;
    }

    void RRTStar::setRadius(const float radius) { // set the radius for the RRT* algorithm
        this->m_rrstar_radius = radius;
    }

    void RRTStar::setDestinationThreshold(const float thresh) { // set the destination threshold for the RRT* algorithm
        this->m_destination_threshhold = thresh;
    }

    float RRTStar::getStepSize() { // get the step size (the maximum distance between two nodes) of the RRT* algorithm
        return this->m_step_size;
    }

    void RRTStar::setMaxIterations(const int iter) { // set the maximum number of iteration for the RRT* algorithm
        this->m_max_iter = iter;
    }

    int RRTStar::getMaxIterations() { //get the maximum number of iteration of the RRT* algorithm
        return this->m_max_iter;
    }

    int RRTStar::getCurrentIterations() {
        return this->m_num_itr;
    }

    const std::vector<std::shared_ptr<Node>> RRTStar::getBestPath() const {
        return this->bestpath;
    }

    std::vector<Point> RRTStar::generatePlan(std::shared_ptr<Node> n) {// generate shortest path to destination.
        this->bestpath.clear();
        while (n != NULL) { // It goes from the given node to the root
            this->bestpath.push_back(n);
            n = n->parent;
        }

        this->m_cost_bestpath = this->bestpath[0]->cost;
        return this->planFromBestPath();
    }

    std::vector<Point> RRTStar::planFromBestPath() { // Generate plan (vector of points) from the best plan so far.
        std::vector<Point> pfb_generated_plan;
        // Accelerate I/O here. 
        for (size_t i = 0; i < this->bestpath.size(); i++) { // It goes from a node near destination to the root
            pfb_generated_plan.push_back(this->bestpath[i]->position);
        }
        return pfb_generated_plan;
    }

    void RRTStar::plotBestPath() {
        // Create a white image
        cv::Mat img(this->m_map.size(), CV_8UC3, cv::Scalar(255, 255, 255));

        // Overlay the grayscale map onto the white image
        cv::cvtColor(m_map, img, cv::COLOR_GRAY2BGR);

        // If we have a best path, draw it in red
        if (!bestpath.empty()) {
            for (size_t i = 1; i < bestpath.size(); i++) {
                cv::line(img, cv::Point(bestpath[i - 1]->position.m_x, bestpath[i - 1]->position.m_y),
                        cv::Point(bestpath[i]->position.m_x, bestpath[i]->position.m_y), cv::Scalar(0, 0, 255), 2);
            }
        }

        cv::imshow("RRT* Path", img);
        cv::waitKey(1);
    }

    // bool RRTStar::check_obstacle_intersection(const cv::Mat& image, int xBegin, int yBegin, int xEnd, int yEnd) {
    //     int dx = abs(xEnd - xBegin), sx = xBegin < xEnd ? 1 : -1;
    //     int dy = -abs(yEnd - yBegin), sy = yBegin < yEnd ? 1 : -1; 
    //     int error = dx + dy, error2;

    //     std::vector<cv::Point> line_points; // Vector to store line points

    //     while (true) {
    //         line_points.push_back(cv::Point(xBegin, yBegin)); // Add point to the vector

    //         // Check if the point is within the image boundaries and is an obstacle
    //         if (xBegin >= 0 && xBegin < image.cols && yBegin >= 0 && yBegin < image.rows) {
    //             if (image.at<uchar>(yBegin, xBegin) != 255) { 
    //                 return true;
    //             }
    //         }

    //         if (xBegin == xEnd && yBegin == yEnd) break;
    //         error2 = 2 * error;
    //         if (error2 >= dy) { error += dy; xBegin += sx; }
    //         if (error2 <= dx) { error += dx; yBegin += sy; }
    //     }

    //     // Drawing the line on the image for visualization
    //     cv::Mat img_with_line = image.clone();
    //     for (const auto& point : line_points) {
    //         cv::circle(img_with_line, point, 1, cv::Scalar(0, 0, 255), -1); // Drawing in red
    //     }

    //     return false;
    // }

    bool RRTStar::isBlack(const cv::Mat& image, int x, int y) {
        if (x >= 0 && x < image.cols && y >= 0 && y < image.rows) {
            return image.at<uchar>(x, y) == 0;
        }
        return false;
    }

    void RRTStar::_bresenhamCircleCollision(const cv::Mat& image, int yc, int xc, int radius, bool &collisionDetected) {
        int x = 0;
        int y = radius;
        int d = 3 - 2 * radius;
        while (y >= x) {
            // Check points using 8-way symmetry
            if (isBlack(image, xc + x, yc + y) || isBlack(image, xc - x, yc + y) ||
                isBlack(image, xc + x, yc - y) || isBlack(image, xc - x, yc - y) ||
                isBlack(image, xc + y, yc + x) || isBlack(image, xc - y, yc + x) ||
                isBlack(image, xc + y, yc - x) || isBlack(image, xc - y, yc - x)) {
                collisionDetected = true;
                return;
            }
            x++;
            if (d > 0) {
                y--;
                d += 4 * (x - y) + 10;
            } else {
                d += 4 * x + 6;
            }
        }
    }

    bool RRTStar::check_obstacle_intersection(const cv::Mat& image, int xBegin, int yBegin, const int xEnd, const int yEnd, const int radius) {
        int dx = abs(xEnd - xBegin), sx = xBegin < xEnd ? 1 : -1;
        int dy = -abs(yEnd - yBegin), sy = yBegin < yEnd ? 1 : -1; 
        int error = dx + dy, error2;

        while (true) {
            bool collisionDetected = false;
            _bresenhamCircleCollision(image, xBegin, yBegin, radius, collisionDetected);
            if (collisionDetected) {
                return true; }
            if (xBegin == xEnd && yBegin == yEnd) break;
            error2 = 2 * error;
            if (error2 >= dy) { error += dy; xBegin += sx; }
            if (error2 <= dx) { error += dx; yBegin += sy; }
        }
        return false;
    }
    
} // namespace rrt_star
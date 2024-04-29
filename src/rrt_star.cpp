#include"rrt_star.hpp"
#include "grid_map.hpp"
#include "QTree.hpp"
#include <iostream>
#include <cfloat>
#include <algorithm>
#include <vector>
#include <opencv2/opencv.hpp>
#include <thread>
#include <atomic>
#include "omp.h"

//Point class Constructors
Point::Point()
    :m_x(0.0f), m_y(0.0f) {}
Point::Point(float X, float Y)
    : m_x(X), m_y(Y) {}

Node::Node() {
    parent= nullptr;
    cost=0;
}
Node::~Node(){
}

void Node::set_position(Point pos){
    this->position = pos;
    this->x = pos.m_x;
    this->y = pos.m_y;
}

//RRTSTAR class Constructors
RRTSTAR::RRTSTAR(Point start_pos, Point end_pos, float radius, float end_thresh, cv::Mat map, float step_size, int max_iter, QTree::QuadTree<Node>*& qtree) {
    //set the default values and set the first node as the staring point
    startPoint = start_pos;
    destination = end_pos;
    root = new Node;
    root->parent = nullptr;
    root->position = startPoint;
    root->cost = 0.0;
    lastnode = root;
    nodes.push_back(root);
    Available_Points.push_back(root->position);

    m_step_size = step_size;
    m_max_iter = max_iter;
    m_map = map;
    m_rrstar_radius = radius;
    m_destination_threshhold = end_thresh;
    m_num_itr = 0;
    m_cost_bestpath = 0;
    this->qtree = qtree;
}

//Destructor
RRTSTAR::~RRTSTAR()
{
    deleteNodes(root);
}

//RRTSTAR methods
std::vector<Point> RRTSTAR::planner() {
    // while iter < MAX_Iterations
    while (this->m_num_itr < this->m_max_iter)
    {
        this->m_num_itr++;
        Node plan_n_rand = this->getRandomNode(); //Generate a random node
        if (plan_n_rand.position.m_x!=0 && plan_n_rand.position.m_y!=0) {
            
            Node* plan_n_nearest = this->qtree->nearest_neighbor(&plan_n_rand);
            // Node* plan_n_nearest = this->findNearest(plan_n_rand.position);  //Find the closest node to the new random node.
            Point plan_p_new = this->steer(plan_n_rand, plan_n_nearest); //Steer from "N_Nearest" towards "N_rand": interpolate if node is too far away.
            bool intersection = this->check_obstacle_intersection(this->m_map, plan_n_nearest->position.m_x, plan_n_nearest->position.m_y, plan_p_new.m_x, plan_p_new.m_y);
        
        if (!intersection) { // Check obstacle
                    Node* plan_n_new = new Node; //create new node to store the position of the steered node.
                    plan_n_new->set_position(plan_p_new); //set the position of the new node

                    std::vector<Node*> plan_v_n_near; //create a vector for neighbor nodes
                    // QTree::Rectangle boundary(grid_map.cols/2, grid_map.rows/2, grid_map.cols, grid_map.rows);
                    QTree::Rectangle nn(plan_n_new->x, plan_n_new->y, m_rrstar_radius*2, m_rrstar_radius*2);
                    this->qtree->query(nn, plan_v_n_near);
                    
                    this->qtree->insert(plan_n_new); // Insert the new node to the quadtree

                    // this->findNearNeighbors(plan_n_new->position, this->m_rrstar_radius, plan_v_n_near); // Find nearest neighbors with a given radius from new node.
                    Node* plan_n_parent=this->findParent(plan_v_n_near,plan_n_nearest,plan_n_new); //Find the parent of the given node (the node that is near and has the lowest path cost)
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
                            Node* Plan_NearNodeEnd = this->findNearest(this->destination);
                            if (Plan_NearNodeEnd->cost < this->m_cost_bestpath) {
                                return this->generatePlan(Plan_NearNodeEnd);
                            }
                        }
                    } 
            }            
        }
    }
    
    if (this->bestpath.empty()) {
        // if not reached yet, no solution has found
        std::cout << "Exceeded max iterations!" << std::endl;
        std::cout << "Error: No solution found" << std::endl;
        return {};
    }
    else {
        return RRTSTAR::planFromBestPath(); //after reaching the maximum iteration number retun the path that has the lowest cost.
    }     
}

Node RRTSTAR::getRandomNode() {
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
        Node rand_randomnode;
        rand_randomnode.set_position(Point(rand_x_int, rand_y_int)); // Create a Point with integer coordinates
        //rand_randomnode.position = Point(rand_x_int, rand_y_int); // Create a Point with integer coordinates
        return rand_randomnode;
    }
    return {};
}

Node* RRTSTAR::findNearest(const Point point) {
    Node* local_closest[omp_get_max_threads()]; 
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

    Node* fn_closest = NULL;
    float fn_minDist = FLT_MAX;
    for (int i = 0; i < omp_get_max_threads(); i++) {
        if (local_minDist[i] < fn_minDist) {
            fn_minDist = local_minDist[i];
            fn_closest = local_closest[i];
        }
    }

    return fn_closest;
}

void RRTSTAR::findNearNeighbors(const Point point, const float radius, std::vector<Node*>& neighbor_nodes) { // Find neighbor nodes of the given node within the defined radius
    for (size_t i = 0; i < this->nodes.size(); i++) { //iterate through all nodes to see which ones fall inside the circle with the given radius.
        if (this->distance(point, this->nodes[i]->position) < radius) {
            neighbor_nodes.push_back(this->nodes[i]);
        }
    }
}

float RRTSTAR::distance(const Point p, const Point q) { //Find the distance between two points.
    Point dist_v = p - q;
    return sqrt(powf(dist_v.m_x, 2) + powf(dist_v.m_y, 2));
}

float RRTSTAR::getCost(const Node* N) { //get the cost current node (traveling from the given node to the root)
    return N->cost;
}

float RRTSTAR::pathCost(const Node* Np, const Node* Nq) { //Compute the distance between the position of two nodes
    return this->distance(Nq->position, Np->position);
}

Point RRTSTAR::steer(const Node n_rand, const Node* n_nearest) { // Steer from new node towards the nearest neighbor and interpolate if the new node is too far away from its neighbor

    if (this->distance(n_rand.position, n_nearest->position) >this->m_step_size) { //check if the distance between two nodes is larger than the maximum travel step size
        Point steer_p = n_rand.position - n_nearest->position;
        double steer_norm = this->distance(n_rand.position, n_nearest->position);
        steer_p = steer_p / steer_norm; //normalize the vector
        return (n_nearest->position + this->m_step_size * steer_p); //travel in the direction of line between the new node and the near node
    }
    else {
        return  (n_rand.position);
    }
}

Node* RRTSTAR::findParent(std::vector<Node*> v_n_near,Node* n_nearest, Node* n_new) {
    Node* fp_n_parent = n_nearest; //create new note to find the parent
    float fp_cmin = this->getCost(n_nearest) + this->pathCost(n_nearest, n_new); // Update cost of reaching "N_new" from "N_Nearest"
    for (size_t j = 0; j < v_n_near.size(); j++) { //In all members of "N_near", check if "N_new" can be reached from a different parent node with cost lower than Cmin, and without colliding with the obstacle.
        Node* fp_n_near = v_n_near[j];
        if ((true) &&
            (this->getCost(fp_n_near) + this->pathCost(fp_n_near, n_new)) < fp_cmin) {
            fp_n_parent = fp_n_near; // a near node with minimun cost of path
            fp_cmin = this->getCost(fp_n_near) + this->pathCost(fp_n_near, n_new); //update the cost of path
        }
    }
    return fp_n_parent;
}

std::vector<Point> RRTSTAR::get_available_points(){
    return this->Available_Points;
}

void RRTSTAR::insertNode(Node* n_parent, Node* n_new) { //Append the new node to the tree.
    n_new->parent = n_parent; //update the parent of new node
    n_new->cost = n_parent->cost + this->pathCost(n_parent, n_new);//update the cost of new node
    n_parent->children.push_back(n_new); //update the children of the nearest node to the new node
    this->nodes.push_back(n_new);//add the new node to the tree
    this->Available_Points.push_back(n_new->position); //Add one more availble point! 

    this->lastnode = n_new;//inform the tree which node is just added
}

void RRTSTAR::reWire(Node* n_new, std::vector<Node*>& neighbor_nodes) { // Rewire the tree to decrease the cost of the path. 
    for (size_t j = 0; j < neighbor_nodes.size(); j++) {  // Search through nodes in "N_near" and see if changing their parent to "N_new" lowers the cost of the path. Also check the obstacles
        Node* rw_n_near = neighbor_nodes[j];
        if ((true) &&
            (this->getCost(n_new) + this->pathCost(n_new, rw_n_near)) < this->getCost(rw_n_near)) {
            Node* rw_n_parent = rw_n_near->parent;
            float rw_costdifference = this->getCost(rw_n_near) - (this->getCost(n_new) + this->pathCost(n_new, rw_n_near)); //calculate the cost  by which the cost of all children of the near node must decrease
            // Remove branch between N_Parent and N_Near
            
            rw_n_parent->children.erase(std::remove(rw_n_parent->children.begin(), rw_n_parent->children.end(), rw_n_near), rw_n_parent->children.end());
            // Add branch between N_New and N_Near
            rw_n_near->cost = this->getCost(n_new) + this->pathCost(n_new, rw_n_near);
            rw_n_near->parent = n_new;
            n_new->children.push_back(rw_n_near);
            // this->Available_Points.push_back(n_new->position);
            this->updateChildrenCost(rw_n_near, rw_costdifference);// Update the cost of all children of the near node 
        }
    }
}

// Here can I parallelize TOO.
void RRTSTAR::updateChildrenCost(Node* n, const float costdifference) {//Update the cost of all children of a node after rewiring 
    for (size_t i = 0; i < n->children.size(); i++)
    {
        n->children[i]->cost = n->children[i]->cost - costdifference;
        this->updateChildrenCost(n->children[i], costdifference); //recursive function. call it self to go through all children of the given node.
    }
}

bool RRTSTAR::reached() { //check if the last node in the tree is close to the end position.
    if (this->distance(this->lastnode->position, this->destination) < m_destination_threshhold) {
        return true;
    }
    return false;
}

void RRTSTAR::setStepSize(const float step) { // set the step size (the maximum distance between two nodes) for the RRT* algorithm
    this->m_step_size = step;
}

float RRTSTAR::getStepSize() { // get the step size (the maximum distance between two nodes) of the RRT* algorithm
    return this->m_step_size;
}

void RRTSTAR::setMaxIterations(const int iter) { // set the maximum number of iteration for the RRT* algorithm
    this->m_max_iter = iter;
}

int RRTSTAR::getMaxIterations() { //get the maximum number of iteration of the RRT* algorithm
    return this->m_max_iter;
}

int RRTSTAR::getCurrentIterations() {
    return this->m_num_itr;
}

const std::vector<Node*> RRTSTAR::getBestPath() const {
    return this->bestpath;
}

std::vector<Point> RRTSTAR::generatePlan(Node* n) {// generate shortest path to destination.
    while (n != NULL) { // It goes from the given node to the root
        this->path.push_back(n);
        n = n->parent;
    }
    this->bestpath.clear();
    this->bestpath = this->path;    //store the current plan as the best plan so far

    this->path.clear(); //clear the path as we have stored it in the Bestpath variable.
    this->m_cost_bestpath = this->bestpath[0]->cost; //store the cost of the generated path
    return this->planFromBestPath();
}

std::vector<Point> RRTSTAR::planFromBestPath() { // Generate plan (vector of points) from the best plan so far.
    std::vector<Point> pfb_generated_plan;
    // Accelerate I/O here. 
    for (size_t i = 0; i < this->bestpath.size(); i++) { // It goes from a node near destination to the root
        pfb_generated_plan.push_back(this->bestpath[i]->position);
    }
    return pfb_generated_plan;
}

void RRTSTAR::deleteNodes(Node* root){ //Free up memory when RRTSTAR destructor is called.

    for (auto& i : root->children) {
        deleteNodes(i);
    }
    delete root;
}

void RRTSTAR::plotBestPath() {
    // Create a white image
    cv::Mat img(this->m_map.size(), CV_8UC3, cv::Scalar(255, 255, 255));

    // Overlay the grayscale map onto the white image
    cv::cvtColor(m_map, img, cv::COLOR_GRAY2BGR);

    // Draw all available points in blue
    for (const Point& p : get_available_points()) {
        cv::circle(img, cv::Point(p.m_x, p.m_y), 1, cv::Scalar(255, 0, 0), -1);
    }

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

bool RRTSTAR::check_obstacle_intersection(const cv::Mat& image, int xBegin, int yBegin, int xEnd, int yEnd) {
    int dx = abs(xEnd - xBegin), sx = xBegin < xEnd ? 1 : -1;
    int dy = -abs(yEnd - yBegin), sy = yBegin < yEnd ? 1 : -1; 
    int error = dx + dy, error2;

    std::vector<cv::Point> line_points; // Vector to store line points

    while (true) {
        line_points.push_back(cv::Point(xBegin, yBegin)); // Add point to the vector

        // Check if the point is within the image boundaries and is an obstacle
        if (xBegin >= 0 && xBegin < image.cols && yBegin >= 0 && yBegin < image.rows) {
            if (image.at<uchar>(yBegin, xBegin) != 255) { 
                return true;
            }
        }

        if (xBegin == xEnd && yBegin == yEnd) break;
        error2 = 2 * error;
        if (error2 >= dy) { error += dy; xBegin += sx; }
        if (error2 <= dx) { error += dx; yBegin += sy; }
    }

    // Drawing the line on the image for visualization
    cv::Mat img_with_line = image.clone();
    for (const auto& point : line_points) {
        cv::circle(img_with_line, point, 1, cv::Scalar(0, 0, 255), -1); // Drawing in red
    }

    return false;
}
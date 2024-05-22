/**
 *  This file contains classes and methods to implement RRT* algorithm
 */

#pragma once
#include <random>
#include <optional>
#include <opencv2/opencv.hpp>
#include "QTree.hpp"

// ======= 3 Classes : Point, Node, RRTSTAR ============== //

/**
* @brief Class for storing position of a point in the space
*/
class Point {
public:
    //Data member
    float m_x;
    float m_y;
    //Constructors
    Point();
    Point(float X, float Y);
};

//Operators overloading for Point class
template <typename T>
inline Point operator *(const Point& left, T right)
{
    return Point(left.m_x * right, left.m_y * right);
}

template <typename T>
inline Point operator *(T left, const Point& right)
{
    return Point(right.m_x * left, right.m_y * left);
}

inline Point operator +(const Point& left, const Point& right)
{
    return Point(left.m_x + right.m_x, left.m_y + right.m_y);
}


inline Point operator -(const Point& left, const Point& right)
{
    return Point(left.m_x - right.m_x, left.m_y - right.m_y);
}

template <typename T>
inline Point operator /(const Point& left, T right)
{
    return Point(left.m_x / right, left.m_y / right);
}

/**
 * @brief Class for storing node data
 */
class Node {
public:
    // Data Members 
    std::vector<Node*> children;
    Node* parent;
    Point position;
    float x;
    float y;
    float cost;
    float cur_dist;

    //Constructor
    Node();
    //Destructor
    ~Node();

    void set_position(Point pos);
};

/**
 * @brief Class for RRT* objects
 */
class RRTSTAR
{
private:
    // Private Data Members 
    Node* root;
    Point startPoint;
    Point destination;
    QTree::QuadTree<Node>* qtree;
    std::vector<Node*> path;
    std::vector<Node*> bestpath;
    std::vector<Point> Available_Points;
    
    int m_num_itr;
    float m_rrstar_radius;
    float m_destination_threshhold;
    int m_max_iter;
    float m_step_size;
    cv::Mat m_map;

public:

    // Time.
    float IO_time, IO_starttime, IO_endtime;
    float Comm_time;
    float Compute_time;

    // public Data Members 
    std::vector<Node*> nodes;
    Node* lastnode;
    float m_cost_bestpath;

    /**
    * @brief construct the RRTSTAR class
    * @param Point start point
    * @param Point end point (i.e., destination point)
    * @param float radius. Within a radius of r, RRT* will find all neighbour nodes of a node
    * @param float end threshold. Check within the radius of the end threshold to find nodes near the end point
    * @param cv::Mat map. The map used for RRT* exploration
    * @param float step size. The step size for RRT* exploration
    * @param int max iterations. The maximum number of iterations for RRT* exploration
    * @param QTree::QuadTree* qtree. The quadtree used for RRT* exploration
     */
    RRTSTAR(Point start_pos, Point end_pos, float radius, float end_thresh, cv::Mat map, float step_size, int max_iter, QTree::QuadTree<Node>*& qtree);

    //Destructor
    ~RRTSTAR();
    //Methods

   /**
   * @brief For visualization of RRT* reachable workspace
   * @return std::vector<Point>  return available points.
   */   
    // return available points.
    std::vector<Point> get_available_points();

   /**
   * @brief Return rrt node vector Points
   * @return std::vector<Point> nodes->position;
   */   
    // return available points.
    std::vector<Point> get_nodes_points();

   /**
   * @brief RRT Exploration Tree.
   * @param int K : number of points desired to sample. 
   * @return std::vector<Point>  return available points.
   */   
    // return available points.
    std::vector<Point> RRT_Explore(int K);

   /**
   * @brief Main algorithm of RRT*
   * @return std::vector<Point>  path vector of nodes
   */
    std::vector<Point> planner();
    
   /**
   * @brief Pthread version: Main algorithm of RRT*
   * @return std::vector<Point>  path vector of nodes
   */    
    std::vector<Point> planner_pthread();

    /**
    * @brief Generates a random node
    *        if p > epsilon: random node = goal node (For faster convergence)
    *        else : not goal node.
    *        Note: global variable: epsilon ~= 0.85 
    * @param Void
    * @return Node Generated node
        */
    Node RandomNode_Epsilon();

    /**
    * @brief Generates a random node
    * @param Void
    * @return Node Generated node
     */
    Node getRandomNode();

    /**
     @brief Find the nearest Node to the new random node.
   * @param Point point (i.e., new random point)
   * @return Node* nearest node
    */
    Node* findNearest(const Point point);

    /**
    * @brief Find neighbor nodes of the given node within the defined radius
    * @param Point point (i.e., the given node) 
    * @param float radius (i.e.,  Within the radius, the method will find all existing neighbor nodes)
    * @param std::vector<Node*> neighbor_nodes (i.e, the found neighbor nodes)
    */
    void findNearNeighbors(const Point point, const float radius, std::vector<Node*>& neighbor_nodes);

    /**
    * @brief Find the distance between two points.
    * @param Point p (i.e., first point)
    * @param Point q (i.e., second point)
    * @return double distance 
    */
    float distance(const Point p, const Point q);

    /**
    * @brief Return the cost current node (traveling from the given node to the root)
    * @param Node* N (i.e., new random node) 
    * @return double cost
    */
    float getCost(const Node* N);

    /**
    * @brief Compute the distance between the position of two nodes
    * @param Node* Np (i.e., first node) 
    * @param Node* Nq (i.e., second node)
    * @return double cost
    */
    float pathCost(const Node* Np, const Node* Nq);

    /**
    * @brief Steer from new node towards the nearest neighbor and interpolate if the new node is too far away from its neighbor
    * @param Node N_rand (i.e., the new node)
    * @param Node* N_Nearest (i.e., the neighbor of the new node)
    * @return Point the position of the interpolated node.
    */
    Point steer(const Node n_rand, const Node* n_nearest);

    /**
    * @brief Append the new node to the tree.
    * @param Node* N_Nearest (i.e., the nearest node to the new node)
    * @param Node* N_New (i.e., the new node)
    * @return Void
    */
    void insertNode(Node* n_nearest, Node* n_new);

    /**
    * @brief Find the parent of the given node (the node that is near and has the lowest path cost)
    * @param Node* v_n_near (i.e., the vector of nodes that are in a circle around the new node)
    * @param Node* N_Nearest (i.e., the nearest node to the new node)
    * @param Node* N_New (i.e., the new node)
    * @return Node* parent
    */
    Node* findParent(std::vector<Node*> v_n_near, Node* n_nearest, Node* n_new);

    /**
    * @brief Rewire the tree to decrease the cost of the path. Search through nodes in "N_near" and see if changing their parent to "N_new" lowers the cost of the path. If so, rewire the tree and
    *add them as the children of "N_new" and update the cost of the path.
    * @param Node* qNew (i.e., the new node)
    * @param std::vector<Node*> neighbor_nodes (i.e, neighbor nodes within the RRTSTAR_RADIUS)
    * @return void
     */
    void reWire(Node* n_new, std::vector<Node*>& neighbor_nodes);

    /**
    * @brief Update the cost of all children of a node after rewiring 
    * @param Node* n (i.e., the given node)
    * @param double CostDifference: the amount by which the cost of all children of the given node must decrease.
    * @return void
    */
    void updateChildrenCost(Node* n, const float costdifference);

    /**
    * @brief Check if the last node in the tree is close to the end position. 
    * @param Void
    * @return bool
    */
    bool reached();

    /**
    * @brief set the step size (the maximum distance between two nodes) for the RRT* algorithm
    * @param int step
    * @return void
    */
    void setStepSize(const float step);

    /**
    * @brief return the step size (the maximum distance between two nodes) of the RRT* algorithm
    * @param void
    * @return int step size
    */
    float getStepSize();

    /**
   * @brief set the maximum number of iteration for the RRT* algorithm
   * @param int iter
   * @return void
   */
    void setMaxIterations(const int iter);

    /**
    * @brief Return the maximum number of iteration of the RRT* algorithm
    * @param void
    * @return int maximum number of iteration
    */
    int getMaxIterations();

    /**
    * @brief Return the current iteration number of the RRT* algorithm
    * @param void
    * @return int current iteration number
    */
    int getCurrentIterations();

    const std::vector<Node*> getBestPath() const;

    const std::pair<float, float> getMapSize() const;

    /**
    * @brief Generate plan (vector of points) from a point near the destination. Also, set the best plan so far into the RRTSTAR class.
    * @param Node* n (i.e., a point near the destination)
    * @return std::vector<Point> plan (vector of points)
    */
    std::vector<Point> generatePlan(Node* n);

    /**
    * @brief Generate plan (vector of points) from the best plan so far.
    * @param void
    * @return std::vector<Point> plan (vector of points)
    */
    std::vector<Point> planFromBestPath();

    /**
     * @brief Delete all nodes 
     * @param Node* root
     */
    void deleteNodes(Node* root);

    void plotBestPath();

    bool check_obstacle_intersection(const cv::Mat& image, int xBegin, int yBegin, int xEnd, int yEnd);
};
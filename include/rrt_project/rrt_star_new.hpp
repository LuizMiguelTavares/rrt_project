/**
 *  This file contains classes and methods to implement RRT* algorithm
 */

#pragma once
#include <random>
#include <optional>
#include <opencv2/opencv.hpp>
#include "QTree_ptr.hpp"
#include <memory>

// ======= 3 Classes : Point, Node, RRTStar ============== //


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
    std::vector<std::shared_ptr<Node>> children;
    std::shared_ptr<Node> parent;
    Point position;
    float x;
    float y;
    float cost;
    float cur_dist;

    //Constructor
    Node();

    void set_position(Point pos);
};

/**
 * @brief Class for RRT* objects
 */
class RRTStar
{
private:
    // Private Data Members 
    std::shared_ptr<Node> root;
    Point startPoint;
    Point destination;
    std::shared_ptr<QTree::QuadTree<Node>> qtree;
    std::vector<std::shared_ptr<Node>> bestpath;
    
    int m_num_itr;
    float m_rrstar_radius;
    float m_destination_threshhold;
    int m_max_iter;
    float m_step_size;
    cv::Mat m_map;

    const int DEFAULT_MAX_ITER = 5000;
    const float DEFAULT_STEP_SIZE = 10.0f;
    const float DEFAULT_RRTSTAR_RADIUS = 5.0f;
    const float DEFAULT_DESTINATION_THRESHHOLD = 5.0f;

public:

    // Time.
    float IO_time, IO_starttime, IO_endtime;
    float Comm_time;
    float Compute_time;

    // public Data Members 
    std::vector<std::shared_ptr<Node>> nodes;
    std::shared_ptr<Node> lastnode;
    float m_cost_bestpath;

    /**
    * @brief construct the RRTStar class
     */
    RRTStar();

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
    std::shared_ptr<Node> getRandomNode();

    /**
     @brief Find the nearest Node to the new random node.
   * @param Point point (i.e., new random point)
   * @return std::shared_ptr<Node> nearest node
    */
    std::shared_ptr<Node> findNearest(const Point point);

    /**
    * @brief Find neighbor nodes of the given node within the defined radius
    * @param Point point (i.e., the given node) 
    * @param float radius (i.e.,  Within the radius, the method will find all existing neighbor nodes)
    * @param std::vector<std::shared_ptr<Node>> neighbor_nodes (i.e, the found neighbor nodes)
    */
    void findNearNeighbors(const Point point, const float radius, std::vector<std::shared_ptr<Node>>& neighbor_nodes);

    /**
    * @brief Find the distance between two points.
    * @param Point p (i.e., first point)
    * @param Point q (i.e., second point)
    * @return double distance 
    */
    float distance(const Point p, const Point q);

    /**
    * @brief Return the cost current node (traveling from the given node to the root)
    * @param std::shared_ptr<Node> N (i.e., new random node) 
    * @return double cost
    */
    float getCost(const std::shared_ptr<Node> N);

    /**
    * @brief Compute the distance between the position of two nodes
    * @param std::shared_ptr<Node> Np (i.e., first node) 
    * @param std::shared_ptr<Node> Nq (i.e., second node)
    * @return double cost
    */
    float pathCost(const std::shared_ptr<Node> Np, const std::shared_ptr<Node> Nq);

    /**
    * @brief Steer from new node towards the nearest neighbor and interpolate if the new node is too far away from its neighbor
    * @param Node N_rand (i.e., the new node)
    * @param std::shared_ptr<Node> N_Nearest (i.e., the neighbor of the new node)
    * @return Point the position of the interpolated node.
    */
    Point steer(const std::shared_ptr<Node> n_rand, const std::shared_ptr<Node> n_nearest);

    /**
    * @brief Append the new node to the tree.
    * @param std::shared_ptr<Node> N_Nearest (i.e., the nearest node to the new node)
    * @param std::shared_ptr<Node> N_New (i.e., the new node)
    * @return Void
    */
    void insertNode(std::shared_ptr<Node> n_nearest, std::shared_ptr<Node> n_new);

    /**
    * @brief Find the parent of the given node (the node that is near and has the lowest path cost)
    * @param std::shared_ptr<Node> v_n_near (i.e., the vector of nodes that are in a circle around the new node)
    * @param std::shared_ptr<Node> N_Nearest (i.e., the nearest node to the new node)
    * @param std::shared_ptr<Node> N_New (i.e., the new node)
    * @return std::shared_ptr<Node> parent
    */
    std::shared_ptr<Node> findParent(std::vector<std::shared_ptr<Node>> v_n_near, std::shared_ptr<Node> n_nearest, std::shared_ptr<Node> n_new);

    /**
    * @brief Rewire the tree to decrease the cost of the path. Search through nodes in "N_near" and see if changing their parent to "N_new" lowers the cost of the path. If so, rewire the tree and
    *add them as the children of "N_new" and update the cost of the path.
    * @param std::shared_ptr<Node> qNew (i.e., the new node)
    * @param std::vector<std::shared_ptr<Node>> neighbor_nodes (i.e, neighbor nodes within the RRTStar_RADIUS)
    * @return void
     */
    void reWire(std::shared_ptr<Node> n_new, std::vector<std::shared_ptr<Node>>& neighbor_nodes);

    /**
    * @brief Update the cost of all children of a node after rewiring 
    * @param std::shared_ptr<Node> n (i.e., the given node)
    * @param double CostDifference: the amount by which the cost of all children of the given node must decrease.
    * @return void
    */
    void updateChildrenCost(std::shared_ptr<Node> n, const float costdifference);

    /**
    * @brief Check if the last node in the tree is close to the end position. 
    * @param Void
    * @return bool
    */
    bool reached();

    /**
     * @brief Set the start position of the RRT* algorithm
     * @param float x, float y
     * @return void
     */
    void setStartPoint(const float x, const float y);

    /**
     * @brief Set the destination position of the RRT* algorithm
     * @param float x, float y
     * @return void
     */
    void setDestination(const float x, const float y);

    /**
     * @brief Set the map of the RRT* algorithm
     * @param cv::Mat map
     * @return void
     */
    void setMap(const cv::Mat map);

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

    const std::vector<std::shared_ptr<Node>> getBestPath() const;

    const std::pair<float, float> getMapSize() const;

    /**
    * @brief Generate plan (vector of points) from a point near the destination. Also, set the best plan so far into the RRTStar class.
    * @param std::shared_ptr<Node> n (i.e., a point near the destination)
    * @return std::vector<Point> plan (vector of points)
    */
    std::vector<Point> generatePlan(std::shared_ptr<Node> n);

    /**
    * @brief Generate plan (vector of points) from the best plan so far.
    * @param void
    * @return std::vector<Point> plan (vector of points)
    */
    std::vector<Point> planFromBestPath();

    /**
     * @brief Delete all nodes 
     * @param std::shared_ptr<Node> root
     */
    void deleteNodes(std::shared_ptr<Node> root);

    void plotBestPath();

    bool check_obstacle_intersection(const cv::Mat& image, int xBegin, int yBegin, int xEnd, int yEnd);
};
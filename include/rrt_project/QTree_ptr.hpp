#pragma once

#include <vector>
#include <memory>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <iostream>
#include <assert.h>

namespace QTree {
    class Point {
    public:
        float x, y;

        Point() : empty(true) {}
        Point(float x, float y) : x(x), y(y), empty(false) {}

        float sqDistanceFrom(const Point& other) const {
            if (this->is_empty()) {
                throw std::runtime_error("Cannot calculate square distance from empty point");
            }

            if (other.is_empty()) {
                throw std::runtime_error("Cannot calculate square distance to empty point");
            }

            float dx = other.x - this->x;
            float dy = other.y - this->y;
            return dx * dx + dy * dy;
        }

        float distanceFrom(const Point& other) const {
            if (this->is_empty()) {
                throw std::runtime_error("Cannot calculate distance from empty point");
            }

            if (other.is_empty()) {
                throw std::runtime_error("Cannot calculate distance to empty point");
            }
            return std::sqrt(this->sqDistanceFrom(other));
        }

        bool is_empty() const {
            return empty;
        }

        void set_values(float x, float y) {
            this->x = x;
            this->y = y;
            empty = false;
        }
    
    private:
        bool empty;
    };

    class Rectangle {
    public:
        Rectangle() : empty(true) {
            left = x - w / 2;
            right = x + w / 2;
            top = y - h / 2;
            bottom = y + h / 2;
        }

        Rectangle(float x, float y, float w, float h) : x(x), y(y), w(w), h(h), empty(false) {
            left = x - w / 2;
            right = x + w / 2;
            top = y - h / 2;
            bottom = y + h / 2;
        }

        bool contains(const Point& point) const {
            if (this->is_empty()) {
                throw std::runtime_error("Rectangle is empty");
            }

            return (left <= point.x && point.x <= right &&
                    top <= point.y && point.y <= bottom);
        }

        bool intersects(const Rectangle& range) const {
            if (this->is_empty()) {
                throw std::runtime_error("Rectangle is empty");
            }

            if (range.is_empty()) {
                throw std::runtime_error("Input range is empty in intersects()");
            }

            return !(right < range.left || range.right < left ||
                    bottom < range.top || range.bottom < top);
        }

        void set_values(float x, float y, float w, float h) {
            this->x = x;
            this->y = y;
            this->w = w;
            this->h = h;
            left = x - w / 2;
            right = x + w / 2;
            top = y - h / 2;
            bottom = y + h / 2;
            empty = false;
        }

        bool is_empty() const {
            return empty;
        }

        float x, y, w, h;
        float left, right, top, bottom;
        Point top_left, top_right, bottom_left, bottom_right;

    private:
        bool empty;
    };

    template<typename Node>
    class QuadTree : public std::enable_shared_from_this<QuadTree<Node>> {
    public:
        QuadTree(const int capacity = DEFAULT_CAPACITY, const int max_depth = DEFAULT_MAX_DEPTH, int reserve_size = 0) 
                : parent(nullptr), divided(false), depth(0), position("root"), capacity(capacity), max_depth(max_depth) {
                    if (capacity < 1) {
                        throw std::range_error("Capacity must be greater than 0");
                    }

                    if (reserve_size > 0) {
                        this->nodes.reserve(reserve_size);
                    }
                }

        QuadTree(const Rectangle& boundary, const int capacity = DEFAULT_CAPACITY, const int max_depth = DEFAULT_MAX_DEPTH, int reserve_size = 0)
                : parent(nullptr), divided(false), depth(0), position("root"), capacity(capacity), max_depth(max_depth), boundary(boundary) {
                    if (capacity < 1) {
                        throw std::range_error("Capacity must be greater than 0");
                    }
                    if (reserve_size > 0) {
                        this->nodes.reserve(reserve_size);
                    }
                }
            
        
        QuadTree(const Rectangle& boundary, const int capacity, const int depth, std::shared_ptr<QTree::QuadTree<Node>> parent, const std::string position, const int max_depth = DEFAULT_MAX_DEPTH)
            : boundary(boundary), capacity(capacity), divided(false), depth(depth), parent(parent), position(position), max_depth(max_depth)  {
                if (capacity < 1) {
                    throw std::range_error("Capacity must be greater than 0");
                }
            }
        
        void set_boundary(const Rectangle& boundary) {
            if (boundary.is_empty()) {
                throw std::runtime_error("Can't set empty boundary");
            }

            this->boundary = boundary;
        }

        bool insert(std::shared_ptr<Node> node) {
            Point point(node->x, node->y);

            if (!boundary.contains(point)) {
                if (parent == nullptr) {
                    // throw std::runtime_error("Node is out of bounds of the QuadTree"); 
                    std::cerr << "Node is out of bounds of the QuadTree" << std::endl;
                }
                return false;
            }

            if (this->nodes.size() < capacity || depth == max_depth) {
                this->nodes.push_back(node);
                return true;
            }

            if (!divided) {
                subdivide();
            }

            if (northeast->insert(node) || northwest->insert(node) || 
                southeast->insert(node) || southwest->insert(node)) {
                return true;
            }

            // This should never happen
            return false;
        }

        std::shared_ptr<Node> nearest_neighbor(const std::shared_ptr<Node> node) {

            assert(this->boundary.contains(Point(node->x, node->y)) && "Can't find nearest neighbor for a node outside the boundary of the QuadTree");
            
            Point point(node->x, node->y);
            std::shared_ptr<QuadTree<Node>> current_quadtree = find_quadtree(this->shared_from_this(), point);

            std::shared_ptr<Node> nearest_node = find_first_valid_node(current_quadtree, point);
            float distance = Point(nearest_node->x, nearest_node->y).sqDistanceFrom(point);

            std::vector<std::shared_ptr<Node>> nearest_nodes;
            float distance_root = std::sqrt(distance);
            Rectangle nearest_nodes_range = Rectangle(point.x, point.y, distance_root * 2, distance_root * 2);
            query(nearest_nodes_range, nearest_nodes);
    
            for (const auto& p : nearest_nodes) {
                float new_distance = Point(p->x, p->y).sqDistanceFrom(point);
                if (new_distance < distance) {
                    distance = new_distance;
                    nearest_node = p;
                }
            }
            return nearest_node;
        }

        std::shared_ptr<Node> nearest_neighbor_test(const std::shared_ptr<Node> node, Rectangle& range_for_test, std::shared_ptr<Node>& first_valid_node, std::vector<std::shared_ptr<Node>>& nearest_nodes_for_test) {

            assert(this->boundary.contains(Point(node->x, node->y)) && "Can't find nearest neighbor for a node outside the boundary of the QuadTree");
            
            Point point(node->x, node->y);
            std::shared_ptr<QuadTree<Node>> current_quadtree = find_quadtree(this->shared_from_this(), point);

            std::shared_ptr<Node> nearest_node = find_first_valid_node(current_quadtree, point);
            float distance = Point(nearest_node->x, nearest_node->y).sqDistanceFrom(point);

            first_valid_node = nearest_node;

            std::vector<std::shared_ptr<Node>> nearest_nodes;
            float distance_root = std::sqrt(distance);
            Rectangle nearest_nodes_range = Rectangle(point.x, point.y, distance_root * 2, distance_root * 2);
            query(nearest_nodes_range, nearest_nodes);

            range_for_test = nearest_nodes_range;
            nearest_nodes_for_test = nearest_nodes;

            for (const auto& p : nearest_nodes) {
                float new_distance = Point(p->x, p->y).sqDistanceFrom(point);
                if (new_distance < distance) {
                    distance = new_distance;
                    nearest_node = p;
                }
            }
            return nearest_node;
        }

        void query(const Rectangle& range, std::vector<std::shared_ptr<Node>>& found) const {
            // Expecting the range to be not empty
            if (!boundary.intersects(range)) {
                return;
            }

            for (const auto& node : this->nodes) {
                Point point(node->x, node->y);
                if (range.contains(point)) {
                    found.push_back(node);
                }
            }

            if (divided) {
                this->northeast->query(range, found);
                this->northwest->query(range, found);
                this->southeast->query(range, found);
                this->southwest->query(range, found);
            }
            return;
        }

    private:
        static const int DEFAULT_CAPACITY = 4;
        static const int DEFAULT_MAX_DEPTH = 8;
        Rectangle boundary;
        const int capacity;
        std::vector<std::shared_ptr<Node>> nodes;
        bool divided;
        std::shared_ptr<QuadTree<Node>> northeast, northwest, southeast, southwest;
        std::shared_ptr<QuadTree<Node>> parent;
        const std::string position;
        const int depth;
        const int max_depth;

        std::shared_ptr<QuadTree<Node>> child(const std::string& pos) {
            if (pos == "northeast") {
                return northeast;
            } else if (pos == "northwest") {
                return northwest;
            } else if (pos == "southeast") {
                return southeast;
            } else if (pos == "southwest") {
                return southwest;
            }
            return nullptr;
        }

        void subdivide() {
        float x = boundary.x;
        float y = boundary.y;
        float w = boundary.w / 2.0;
        float h = boundary.h / 2.0;

        Rectangle neRect(x + w / 2, y - h / 2, w, h);
        Rectangle nwRect(x - w / 2, y - h / 2, w, h);
        Rectangle seRect(x + w / 2, y + h / 2, w, h);
        Rectangle swRect(x - w / 2, y + h / 2, w, h);

        this->northeast = std::make_shared<QuadTree>(neRect, capacity, depth + 1, this->shared_from_this(), "northeast");
        this->northwest = std::make_shared<QuadTree>(nwRect, capacity, depth + 1, this->shared_from_this(), "northwest");
        this->southeast = std::make_shared<QuadTree>(seRect, capacity, depth + 1, this->shared_from_this(), "southeast");
        this->southwest = std::make_shared<QuadTree>(swRect, capacity, depth + 1, this->shared_from_this(), "southwest");

        divided = true;
    }


        std::shared_ptr<QuadTree<Node>> find_quadtree(std::shared_ptr<QuadTree<Node>> quad_tree, const Point& point) const {
            // Assumes that the point is within the boundary of the quad_tree
            if (quad_tree->divided) {
                if (quad_tree->northeast->boundary.contains(point)) {
                    return find_quadtree(quad_tree->northeast, point);
                } else if (quad_tree->northwest->boundary.contains(point)) {
                    return find_quadtree(quad_tree->northwest, point);
                } else if (quad_tree->southeast->boundary.contains(point)) {
                    return find_quadtree(quad_tree->southeast, point);
                } else if (quad_tree->southwest->boundary.contains(point)) {
                    return find_quadtree(quad_tree->southwest, point);
                }
            }
            return quad_tree;
        }

        std::shared_ptr<Node> find_first_valid_node(const std::shared_ptr<const QuadTree>& quadtree, const Point& point) const {
            // If the current QuadTree has nodes, return the first one
            if (!quadtree->nodes.empty()) {
                return quadtree->nodes.front();
            }

            // If the QuadTree has no parent, then there are no siblings to check
            if (quadtree->parent == nullptr) {
                throw std::runtime_error("There are no valid nodes in the QuadTree!");
            }

            // Check siblings in a fixed order: northeast, northwest, southeast, southwest
            static const std::vector<std::string> positions = {"northeast", "northwest", "southeast", "southwest"};
            for (const auto& pos : positions) {
                if (pos != quadtree->position) {
                    auto sibling_quad = quadtree->parent->child(pos);
                    if (sibling_quad && !sibling_quad->nodes.empty()) {
                        return sibling_quad->nodes.front();
                    }
                }
            }
            
            // Should never reach here
            throw std::runtime_error("No valid nodes found in the QuadTree!");
        }

    };
} // namespace QTree
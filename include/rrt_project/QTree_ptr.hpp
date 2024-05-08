#pragma once

#include <vector>
#include <memory>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <iostream>

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

    private:
        bool empty;
        float x, y, w, h;
        float left, right, top, bottom;
        Point top_left, top_right, bottom_left, bottom_right;
    };

    template<typename Node>
    class QuadTree {
    public:
        QuadTree(const int capacity = DEFAULT_CAPACITY, const int max_depth = DEFAULT_MAX_DEPTH, int reserve_size = 0) 
                : parent(nullptr), divided(false), depth(0), position("root"), capacity(capacity), max_depth(max_depth), empty(true) {
                    if (capacity < 1) {
                        throw std::range_error("Capacity must be greater than 0");
                    }

                    if (reserve_size > 0) {
                        this->nodes.reserve(reserve_size);
                    }
                }

        QuadTree(const Rectangle& boundary, const int capacity = DEFAULT_CAPACITY, const int max_depth = DEFAULT_MAX_DEPTH, int reserve_size = 0)
                : parent(nullptr), divided(false), depth(0), position("root"), capacity(capacity), max_depth(max_depth), boundary(boundary), empty(true) {
                    if (capacity < 1) {
                        throw std::range_error("Capacity must be greater than 0");
                    }
                    if (reserve_size > 0) {
                        this->nodes.reserve(reserve_size);
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
                    throw std::runtime_error("Node is out of bounds of the QuadTree"); 
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

        // std::shared_ptr<Node> nearest_neighbor(const std::shared_ptr<Node> node) {

        //     assert(this->boundary.contains(Point(node->x, node->y)) && "Can't find nearest neighbor for a node outside the boundary of the QuadTree");
            
        //     Point point(node->x, node->y);
        //     QuadTree* current_quadtree = find_quadtree(this, point);

        //     Node* current_closest_node = nullptr;
        //     float distance = find_parent_distance(current_quadtree, point, current_closest_node);

        //     std::vector<Node*> nodes;

        //     Node* possible_nearest_node;

        //     if(distance == 0){
        //         nodes = this->nodes;
        //         possible_nearest_node = nodes[0];
        //     } else {
        //         possible_nearest_node = current_closest_node;
        //         Rectangle range = Rectangle(point.x, point.y, std::sqrt(distance)*2, std::sqrt(distance)*2);
        //         query(range, nodes);
        //     }

        //     Node* nearest_node  = possible_nearest_node;

        //     // Iterate over the nodes and find the nearest one to the given point
        //     for (const auto& p : nodes) {
        //         Point p_point(p->x, p->y);
        //         if (p_point.sqDistanceFrom(point) <= distance) {
        //             distance = p_point.sqDistanceFrom(point);
        //             nearest_node = p;
        //         }
        //     }
        //     return nearest_node;
        // }


    private:
        bool empty;
        static const int DEFAULT_CAPACITY = 4;
        static const int DEFAULT_MAX_DEPTH = 8;
        Rectangle boundary;
        const int capacity;
        std::vector<std::shared_ptr<Node>> nodes;
        bool divided;
        std::shared_ptr<QuadTree> northeast, northwest, southeast, southwest;
        std::shared_ptr<QuadTree> parent;
        const std::string position;
        const int depth;
        const int max_depth;

        std::shared_ptr<QuadTree> find_quadtree(std::shared_ptr<QuadTree> quad_tree, const Point& point) const {
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

        
    };
} // namespace QTree
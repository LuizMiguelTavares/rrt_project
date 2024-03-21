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
        double x, y;

        Point() : x(0), y(0) {}
        Point(double x, double y) : x(x), y(y) {}

        double sqDistanceFrom(const Point& other) const {
            double dx = other.x - this->x;
            double dy = other.y - this->y;
            return dx * dx + dy * dy;
        }

        double distanceFrom(const Point& other) const {
            return std::sqrt(this->sqDistanceFrom(other));
        }
    };

    class Rectangle {
    public:
        double x, y, w, h;
        double left, right, top, bottom;
        Point top_left, top_right, bottom_left, bottom_right;

        Rectangle() : x(0), y(0), w(0), h(0) {
            left = x - w / 2;
            right = x + w / 2;
            top = y - h / 2;
            bottom = y + h / 2;
        }

        Rectangle(double x, double y, double w, double h) : x(x), y(y), w(w), h(h) {
            left = x - w / 2;
            right = x + w / 2;
            top = y - h / 2;
            bottom = y + h / 2;
        }

        bool contains(const Point& point) const {
            return (left <= point.x && point.x <= right &&
                    top <= point.y && point.y <= bottom);
        }

        bool intersects(const Rectangle& range) const {
            return !(right < range.left || range.right < left ||
                    bottom < range.top || range.bottom < top);
        }
    };

    template<typename Node>
    class QuadTree {
    public:
        QuadTree(const Rectangle& boundary, Node* start, const int capacity = DEFAULT_CAPACITY)
                : boundary(boundary), capacity(capacity), divided(false), depth(0), parent(nullptr), position("root") {
                    if (capacity < 1) {
                        throw std::range_error("capacity must be greater than 0");
                    }

                    this->nodes.reserve(5000);
                    this->insert(start);
                }

        static std::unique_ptr<QuadTree> Create(const Rectangle& boundary, const int capacity, const int depth, const QuadTree* parent, const std::string& position) {
            return std::unique_ptr<QuadTree>(new QuadTree(boundary, capacity, depth, parent, position));
        }

        bool insert(Node* node) {
            Point point(node->x, node->y);

            if (!boundary.contains(point)) {
                return false;
            }

            if (this->nodes.size() < capacity || depth == MAX_DEPTH) {
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

        Point nearest_neighbor_test(const Point& point, Rectangle& range_for_test, Point& current_closest_point_for_test, std::vector<Point>& points_test) {
            QuadTree* current_quadtree = find_quadtree(this, point);

            Point current_closest_point = point;
            double distance = find_parent_distance(current_quadtree, point, current_closest_point);

            Point nearest_point = current_closest_point;
            current_closest_point_for_test = current_closest_point;
            Rectangle range = Rectangle(point.x, point.y, std::sqrt(distance)*2, std::sqrt(distance)*2);
            range_for_test = range;

            std::vector<Point> points;
            points.reserve(100);

            query(range, points);

            points_test = points;

            // Iterate over the points and find the nearest one to the given point
            for (const auto& p : points) {
                if (p.sqDistanceFrom(point) <= distance) {
                    distance = p.sqDistanceFrom(point);
                    nearest_point = p;
                }
            }
            return nearest_point;
        }

        Node* nearest_neighbor(const Node* node) {
            
            Point point(node->x, node->y);
            QuadTree* current_quadtree = find_quadtree(this, point);

            Node* current_closest_node = nullptr;
            double distance = find_parent_distance(current_quadtree, point, current_closest_node);

            std::vector<Node*> nodes;

            Node* possible_nearest_node;

            if(distance == 0){
                nodes = this->nodes;
                possible_nearest_node = nodes[0];
            } else {
                possible_nearest_node = current_closest_node;
                Rectangle range = Rectangle(point.x, point.y, std::sqrt(distance)*2, std::sqrt(distance)*2);
                query(range, nodes);
            }

             Node* nearest_node  = possible_nearest_node;

            // Iterate over the nodes and find the nearest one to the given point
            for (const auto& p : nodes) {
                Point p_point(p->x, p->y);
                if (p_point.sqDistanceFrom(point) <= distance) {
                    distance = p_point.sqDistanceFrom(point);
                    nearest_node = p;
                }
            }
            return nearest_node;
        }

        static const int DEFAULT_CAPACITY = 4;
        static const int MAX_DEPTH = 8;
        const Rectangle boundary;
        const int capacity;
        std::vector<Node*> nodes;
        bool divided;
        std::unique_ptr<QuadTree> northeast, northwest, southeast, southwest;
        const QuadTree *parent;
        const std::string position;
        const int depth;

    private:

        QuadTree(const Rectangle& boundary, const int capacity, const int depth, const QuadTree *parent, const std::string position)
            : boundary(boundary), capacity(capacity), divided(false), depth(depth), parent(parent), position(position) {
                this->nodes.reserve(5000);
            }

        void subdivide() {
            double x = boundary.x;
            double y = boundary.y;
            double w = boundary.w / 2.0;
            double h = boundary.h / 2.0;

            Rectangle neRect(x + w / 2, y - h / 2, w, h);
            Rectangle nwRect(x - w / 2, y - h / 2, w, h);
            Rectangle seRect(x + w / 2, y + h / 2, w, h);
            Rectangle swRect(x - w / 2, y + h / 2, w, h);

            this->northeast = QuadTree::Create(neRect, capacity, depth + 1, this, "northeast");
            this->northwest = QuadTree::Create(nwRect, capacity, depth + 1, this, "northwest");
            this->southeast = QuadTree::Create(seRect, capacity, depth + 1, this, "southeast");
            this->southwest = QuadTree::Create(swRect, capacity, depth + 1, this, "southwest");

            divided = true;
        }

        void query(const Rectangle& range, std::vector<Node*>& found) const {
            if (!boundary.intersects(range)) {
                return ;
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

            return ;
        }

        QuadTree* find_quadtree(QuadTree* quad_tree, const Point& point) const {
            if (quad_tree->divided) {
                if (quad_tree->northeast->boundary.contains(point)) {
                    return find_quadtree(quad_tree->northeast.get(), point);
                } else if (quad_tree->northwest->boundary.contains(point)) {
                    return find_quadtree(quad_tree->northwest.get(), point);
                } else if (quad_tree->southeast->boundary.contains(point)) {
                    return find_quadtree(quad_tree->southeast.get(), point);
                } else if (quad_tree->southwest->boundary.contains(point)) {
                    return find_quadtree(quad_tree->southwest.get(), point);
                }
            }
            return quad_tree;
        }

        double find_parent_distance(const QuadTree* quadtree, const Point& point, Node*& current_closest_node) const {

            if (quadtree->parent == nullptr) {
                return 0; // Code to calculate all the distances because there are not much points
                          // Mostly shouldn't happen
            }

            if (quadtree->nodes.size() > 0) {
                current_closest_node = quadtree->nodes[0];
                Point p(quadtree->nodes[0]->x, quadtree->nodes[0]->y);
                return p.sqDistanceFrom(point);
            } else {
                if (quadtree->position == "northeast"){
                    if (quadtree->parent->northwest->nodes.size() > 0) {
                        current_closest_node = quadtree->parent->northwest->nodes[0];
                        Point p(quadtree->parent->northwest->nodes[0]->x, quadtree->parent->northwest->nodes[0]->y);
                        return p.sqDistanceFrom(point);
                    } else if (quadtree->parent->southwest->nodes.size() > 0) {
                        current_closest_node = quadtree->parent->southwest->nodes[0];
                        Point p(quadtree->parent->southwest->nodes[0]->x, quadtree->parent->southwest->nodes[0]->y);
                        return p.sqDistanceFrom(point);
                    } else if (quadtree->parent->southeast->nodes.size() > 0) {
                        current_closest_node = quadtree->parent->southeast->nodes[0];
                        Point p(quadtree->parent->southeast->nodes[0]->x, quadtree->parent->southeast->nodes[0]->y);
                        return p.sqDistanceFrom(point);
                    }
                } else if (quadtree->position == "northwest"){
                    if (quadtree->parent->northeast->nodes.size() > 0) {
                        current_closest_node = quadtree->parent->northeast->nodes[0];
                        Point p(quadtree->parent->northeast->nodes[0]->x, quadtree->parent->northeast->nodes[0]->y);
                        return p.sqDistanceFrom(point);
                    } else if (quadtree->parent->southwest->nodes.size() > 0) {
                        current_closest_node = quadtree->parent->southwest->nodes[0];
                        Point p(quadtree->parent->southwest->nodes[0]->x, quadtree->parent->southwest->nodes[0]->y);
                        return p.sqDistanceFrom(point);
                    } else if (quadtree->parent->southeast->nodes.size() > 0) {
                        current_closest_node = quadtree->parent->southeast->nodes[0];
                        Point p(quadtree->parent->southeast->nodes[0]->x, quadtree->parent->southeast->nodes[0]->y);
                        return p.sqDistanceFrom(point);
                    }
                } else if (quadtree->position == "southwest"){
                    if (quadtree->parent->northeast->nodes.size() > 0) {
                        current_closest_node = quadtree->parent->northeast->nodes[0];
                        Point p(quadtree->parent->northeast->nodes[0]->x, quadtree->parent->northeast->nodes[0]->y);
                        return p.sqDistanceFrom(point);
                    } else if (quadtree->parent->northwest->nodes.size() > 0) {
                        current_closest_node = quadtree->parent->northwest->nodes[0];
                        Point p(quadtree->parent->northwest->nodes[0]->x, quadtree->parent->northwest->nodes[0]->y);
                        return p.sqDistanceFrom(point);
                    } else if (quadtree->parent->southeast->nodes.size() > 0) {
                        current_closest_node = quadtree->parent->southeast->nodes[0];
                        Point p(quadtree->parent->southeast->nodes[0]->x, quadtree->parent->southeast->nodes[0]->y);
                        return p.sqDistanceFrom(point);
                    }
                } else if (quadtree->position == "southeast"){
                    if (quadtree->parent->northeast->nodes.size() > 0) {
                        current_closest_node = quadtree->parent->northeast->nodes[0];
                        Point p(quadtree->parent->northeast->nodes[0]->x, quadtree->parent->northeast->nodes[0]->y);
                        return p.sqDistanceFrom(point);
                    } else if (quadtree->parent->northwest->nodes.size() > 0) {
                        current_closest_node = quadtree->parent->northwest->nodes[0];
                        Point p(quadtree->parent->northwest->nodes[0]->x, quadtree->parent->northwest->nodes[0]->y);
                        return p.sqDistanceFrom(point);
                    } else if (quadtree->parent->southwest->nodes.size() > 0) {
                        current_closest_node = quadtree->parent->southwest->nodes[0];
                        Point p(quadtree->parent->southwest->nodes[0]->x, quadtree->parent->southwest->nodes[0]->y);
                        return p.sqDistanceFrom(point);
                    }
                }
            }

            // This should never happen
            return 0;
        }
    };
}  // namespace QTree


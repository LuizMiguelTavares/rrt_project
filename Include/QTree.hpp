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

    class QuadTree {
    public:
        QuadTree(const Rectangle& boundary, const int capacity = DEFAULT_CAPACITY)
                : boundary(boundary), capacity(capacity), divided(false), depth(0), parent(nullptr), position("root") {
                    if (capacity < 1) {
                        throw std::range_error("capacity must be greater than 0");
                    }

                    this->points.reserve(5000);
                }

        static std::unique_ptr<QuadTree> Create(const Rectangle& boundary, const int capacity, const int depth, const QuadTree* parent, const std::string& position) {
            return std::unique_ptr<QuadTree>(new QuadTree(boundary, capacity, depth, parent, position));
        }

        bool insert(const Point& point) {
            if (!boundary.contains(point)) {
                return false;
            }

            if (points.size() < capacity || depth == MAX_DEPTH) {
                points.push_back(point);
                return true;
            }

            if (!divided) {
                subdivide();
            }

            if (northeast->insert(point) || northwest->insert(point) || 
                southeast->insert(point) || southwest->insert(point)) {
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

        Point nearest_neighbor(const Point& point) {
            QuadTree* current_quadtree = find_quadtree(this, point);

            Point current_closest_point = point;
            double distance = find_parent_distance(current_quadtree, point, current_closest_point);

            std::vector<Point> points;

            Point possible_nearest_point;

            if(distance == 0){
                points = this->points;
                possible_nearest_point = points[0];
            } else {
                possible_nearest_point = current_closest_point;
                Rectangle range = Rectangle(point.x, point.y, std::sqrt(distance)*2, std::sqrt(distance)*2);
                query(range, points);
            }

            Point nearest_point = possible_nearest_point;

            // Iterate over the points and find the nearest one to the given point
            for (const auto& p : points) {
                if (p.sqDistanceFrom(point) <= distance) {
                    distance = p.sqDistanceFrom(point);
                    nearest_point = p;
                }
            }
            return nearest_point;
        }

        static const int DEFAULT_CAPACITY = 4;
        static const int MAX_DEPTH = 8;
        const Rectangle boundary;
        const int capacity;
        std::vector<Point> points;
        bool divided;
        std::unique_ptr<QuadTree> northeast, northwest, southeast, southwest;
        const QuadTree *parent;
        const std::string position;
        const int depth;

    private:

        QuadTree(const Rectangle& boundary, const int capacity, const int depth, const QuadTree *parent, const std::string position)
            : boundary(boundary), capacity(capacity), divided(false), depth(depth), parent(parent), position(position) {
                this->points.reserve(5000);
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

        void query(const Rectangle& range, std::vector<Point>& found) const {
            if (!boundary.intersects(range)) {
                return ;
            }

            for (const auto& point : points) {
                if (range.contains(point)) {
                    found.push_back(point);
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

        double find_parent_distance(const QuadTree* quadtree, const Point& point, Point& current_closest_point) const {

            if (quadtree->parent == nullptr) {
                return 0; // Code to calculate all the distances because there are not much points
                          // Mostly shouldn't happen
            }

            if (quadtree->points.size() > 0) {
                current_closest_point = quadtree->points[0];
                return quadtree->points[0].sqDistanceFrom(point);
            } else {
                if (quadtree->position == "northeast"){
                    if (quadtree->parent->northwest->points.size() > 0) {
                        current_closest_point = quadtree->parent->northwest->points[0];
                        return quadtree->parent->northwest->points[0].sqDistanceFrom(point);
                    } else if (quadtree->parent->southwest->points.size() > 0) {
                        current_closest_point = quadtree->parent->southwest->points[0];
                        return quadtree->parent->southwest->points[0].sqDistanceFrom(point);
                    } else if (quadtree->parent->southeast->points.size() > 0) {
                        current_closest_point = quadtree->parent->southeast->points[0];
                        return quadtree->parent->southeast->points[0].sqDistanceFrom(point);
                    }
                } else if (quadtree->position == "northwest") {
                    if (quadtree->parent->northeast->points.size() > 0) {
                        current_closest_point = quadtree->parent->northeast->points[0];
                        return quadtree->parent->northeast->points[0].sqDistanceFrom(point);
                    } else if (quadtree->parent->southwest->points.size() > 0) {
                        current_closest_point = quadtree->parent->southwest->points[0];
                        return quadtree->parent->southwest->points[0].sqDistanceFrom(point);
                    } else if (quadtree->parent->southeast->points.size() > 0) {
                        current_closest_point = quadtree->parent->southeast->points[0];
                        return quadtree->parent->southeast->points[0].sqDistanceFrom(point);
                    }
                } else if (quadtree->position == "southeast") {
                    if (quadtree->parent->northeast->points.size() > 0) {
                        current_closest_point = quadtree->parent->northeast->points[0];
                        return quadtree->parent->northeast->points[0].sqDistanceFrom(point);
                    } else if (quadtree->parent->southwest->points.size() > 0) {
                        current_closest_point = quadtree->parent->southwest->points[0];
                        return quadtree->parent->southwest->points[0].sqDistanceFrom(point);
                    } else if (quadtree->parent->northwest->points.size() > 0) {
                        current_closest_point = quadtree->parent->northwest->points[0];
                        return quadtree->parent->northwest->points[0].sqDistanceFrom(point);
                    }
                } else if (quadtree->position == "southwest") {
                    if (quadtree->parent->northeast->points.size() > 0) {
                        current_closest_point = quadtree->parent->northeast->points[0];
                        return quadtree->parent->northeast->points[0].sqDistanceFrom(point);
                    } else if (quadtree->parent->northwest->points.size() > 0) {
                        current_closest_point = quadtree->parent->northwest->points[0];
                        return quadtree->parent->northwest->points[0].sqDistanceFrom(point);
                    } else if (quadtree->parent->southeast->points.size() > 0) {
                        current_closest_point = quadtree->parent->southeast->points[0];
                        return quadtree->parent->southeast->points[0].sqDistanceFrom(point);
                    }
                }
            }

            return 0;
        }
    };
}  // namespace QTree


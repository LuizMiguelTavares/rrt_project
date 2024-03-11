#pragma once

#include <vector>
#include <cmath>
#include <stdexcept>
#include <algorithm>

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

        Rectangle(double x, double y, double w, double h) : x(x), y(y), w(w), h(h) {
            left = x - w / 2;
            right = x + w / 2;
            top = y - h / 2;
            bottom = y + h / 2;
            
            top_left = Point(left, top);
            top_right = Point(right, top);
            bottom_left = Point(left, bottom);
            bottom_right = Point(right, bottom);
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
            static const int DEFAULT_CAPACITY = 4;
            static const int MAX_DEPTH = 8;
            Rectangle boundary;
            int capacity;
            std::vector<Point> points;
            bool divided;
            QuadTree *northeast, *northwest, *southeast, *southwest;
            QuadTree *parent;
            QuadTree *me = this;
            int depth;


        QuadTree(const Rectangle& boundary, int capacity = DEFAULT_CAPACITY, int depth = 0, QuadTree *parent = nullptr)
            : boundary(boundary), capacity(capacity), divided(false), depth(depth),
            northeast(nullptr), northwest(nullptr), southeast(nullptr), southwest(nullptr), parent(parent) {
                if (capacity < 1) {
                    throw std::range_error("capacity must be greater than 0");
                }
            }

        ~QuadTree() {
                delete northeast;
                delete northwest;
                delete southeast;
                delete southwest;
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

            northeast = new QuadTree(neRect, capacity, depth + 1, this);
            northwest = new QuadTree(nwRect, capacity, depth + 1, this);
            southeast = new QuadTree(seRect, capacity, depth + 1, this);
            southwest = new QuadTree(swRect, capacity, depth + 1, this);

            for (const auto& point : points) {
                if (northeast->insert(point) || northwest->insert(point) || 
                    southeast->insert(point) || southwest->insert(point)) {
                    continue;
                }
            }

            divided = true;
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

            points.push_back(point);

            if (northeast->insert(point) || northwest->insert(point) || 
                southeast->insert(point) || southwest->insert(point)) {
                return true;
            }

            // This should never happen
            return false;
        }

        std::vector<Point> query(const Rectangle& range, std::vector<Point>& found) const {
            if (!boundary.intersects(range)) {
                return found;
            }

            for (const auto& point : points) {
                if (range.contains(point)) {
                    found.push_back(point);
                }
            }

            if (divided) {
                northeast->query(range, found);
                northwest->query(range, found);
                southeast->query(range, found);
                southwest->query(range, found);
            }

            return found;
        }
    
    private:

        QuadTree* find_quadtree(const Point& point) {
            if (divided) {
                if (northeast->boundary.contains(point)) {
                    return northeast->find_quadtree(point);
                } else if (northwest->boundary.contains(point)) {
                    return northwest->find_quadtree(point);
                } else if (southeast->boundary.contains(point)) {
                    return southeast->find_quadtree(point);
                } else if (southwest->boundary.contains(point)) {
                    return southwest->find_quadtree(point);
                }
            }
            return this;
        }

    };
}  // namespace QTree
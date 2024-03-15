#pragma once

#include <vector>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <iostream>

namespace QTree {

//     struct Contact {
//     public:
//         bool check_contact(const QuadTree* main_quadtree, const QuadTree* secondary_quadtree) {
//             bool contact = false;
//             contact = check_boundary(main_quadtree, secondary_quadtree);

//             if (contact) {
//                 if (this->direction == "left" || this->direction == "right") {
//                     double start1 = main_quadtree->boundary.bottom;
//                     double end1 = main_quadtree->boundary.top;
//                     double start2 = secondary_quadtree->boundary.bottom;
//                     double end2 = secondary_quadtree->boundary.top;

//                     if(rangesOverlap(start1, end1, start2, end2)) {
//                         return true;
//                     }
//                 } else if (this->direction == "top" || this->direction == "bottom") {
//                     double start1 = main_quadtree->boundary.left;
//                     double end1 = main_quadtree->boundary.right;
//                     double start2 = secondary_quadtree->boundary.left;
//                     double end2 = secondary_quadtree->boundary.right;

//                     if(rangesOverlap(start1, end1, start2, end2)) {
//                         return true;
//                     }
//                 }
//             }
//             return false;
//         }

//         std::string get_direction(){
//             return this->direction;
//             }

//     private:
//         std::string direction;
//         QuadTree* quadtree;

//         bool check_boundary(const QuadTree* main_quadtree, const QuadTree* secondary_quadtree) {
//             if (main_quadtree->boundary.left == secondary_quadtree->boundary.right) {
//                 this->direction = "left";
//                 return true;
//             } else if (main_quadtree->boundary.right == secondary_quadtree->boundary.left) {
//                 this->direction = "right";
//                 return true;
//             } else if (main_quadtree->boundary.top == secondary_quadtree->boundary.bottom) {
//                 this->direction = "top";
//                 return true;
//             } else if (main_quadtree->boundary.bottom == secondary_quadtree->boundary.top) {
//                 this->direction = "bottom";
//                 return true;
//             } else {
//                 return false;
//             }
//         }

//         bool rangesOverlap(double start1, double end1, double start2, double end2) {
//             return (start1 <= end2) && (start2 <= end1);
//     }

// };

    // struct NearestNeighbor {
    // public:
    //     NearestNeighbor(QuadTree* main_quadtree) : main_quadtree(main_quadtree) {}

    //     Point find_nearest_neighbor(const Point& point) {
    //         Point nearest_point;

    //         this->set_current_point(point);
    //         this->set_current_quadtree();
    //         this->set_closest_quadtrees();

    //         return nearest_point;
    //     }

    // private:
    //     QuadTree* main_quadtree;
    //     QuadTree* current_quadtree;
    //     Point current_point;
    //     std::vector<const QuadTree*> closest_quadtrees;
    //     std::vector<const QuadTree*> second_look_quadtrees; // Quadtrees that the closest quadtrees are empty

    //     std::vector<double> get_current_distances(const QuadTree*& quadtree, const Point& point) const {
    //         std::vector<double> distances;

    //         distances.push_back(std::abs(quadtree->boundary.left - point.x));
    //         distances.push_back(std::abs(quadtree->boundary.right - point.x));
    //         distances.push_back(std::abs(quadtree->boundary.top - point.y));
    //         distances.push_back(std::abs(quadtree->boundary.bottom - point.y));
    //         return distances;
    //     }

    //     double get_distance(const Point& point, const QuadTree*& quadtree, const std::string& direction) const {
    //         double distance;

    //         if (direction == "left") {
    //             distance = std::abs(quadtree->boundary.left - point.x);
    //         } else if (direction == "right") {
    //             distance = std::abs(quadtree->boundary.right - point.x);
    //         } else if (direction == "top") {
    //             distance = std::abs(quadtree->boundary.top - point.y);
    //         } else if (direction == "bottom") {
    //             distance = std::abs(quadtree->boundary.bottom - point.y);
    //         } else {
    //             throw std::invalid_argument("Invalid direction");
    //         }
    //         return distance;
    //     }

    //     QuadTree* find_quadtree(QuadTree*& quad_tree) const {
    //         if (quad_tree->divided) {
    //             if (quad_tree->northeast->boundary.contains(this->current_point)) {
    //                 return find_quadtree(quad_tree->northeast);
    //             } else if (quad_tree->northwest->boundary.contains(this->current_point)) {
    //                 return find_quadtree(quad_tree->northwest);
    //             } else if (quad_tree->southeast->boundary.contains(this->current_point)) {
    //                 return find_quadtree(quad_tree->southeast);
    //             } else if (quad_tree->southwest->boundary.contains(this->current_point)) {
    //                 return find_quadtree(quad_tree->southwest);
    //             }
    //         }
    //         return quad_tree;
    //     }

    //     // Setters

    //     void set_current_quadtree() {
    //         this->current_quadtree = find_quadtree(this->main_quadtree);
    //     }

    //     void set_current_point(const Point& point) {
    //         this->current_point = point;
    //     }

    //     void set_closest_quadtrees(QuadTree* quad_tree) {

    //         if
            
    //     }
    // };

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
                : boundary(boundary), capacity(capacity), divided(false), depth(0),
                northeast(nullptr), northwest(nullptr), southeast(nullptr), southwest(nullptr), parent(nullptr), position("root"), number_of_points(0) {
                    if (capacity < 1) {
                        throw std::range_error("capacity must be greater than 0");
                    }
                }
        
        // QuadTree(const Rectangle& boundary, const int capacity = DEFAULT_CAPACITY, std::vector<Point> points)
        //         : boundary(boundary), capacity(capacity), divided(false), depth(0),
        //         northeast(nullptr), northwest(nullptr), southeast(nullptr), southwest(nullptr), parent(nullptr), position("root"), number_of_points(0) {
        //             if (capacity < 1) {
        //                 throw std::range_error("capacity must be greater than 0");
        //             }

        //             for (const auto& point : points) {
        //                 insert(point);
        //             }
        //         }

        ~QuadTree() {
                delete northeast;
                delete northwest;
                delete southeast;
                delete southwest;
            }

        bool insert(const Point& point) {
            if (!boundary.contains(point)) {
                return false;
            }

            if (points.size() < capacity || depth == MAX_DEPTH) {
                points.push_back(point);
                this->number_of_points++;
                return true;
            }

            if (!divided) {
                subdivide();
            }

            points.push_back(point);
            this->number_of_points++;

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

            std::cout << "Range: " << range.left << " " << range.right << " " << range.top << " " << range.bottom << std::endl;

            std::vector<Point> points;
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
        Rectangle boundary;
        int capacity;
        std::vector<Point> points;
        int number_of_points;
        bool divided;
        QuadTree *northeast, *northwest, *southeast, *southwest;
        QuadTree *parent;
        std::string position;
        int depth;

    private:

        QuadTree(const Rectangle& boundary, const int capacity, int depth, QuadTree *parent, std::string position)
            : boundary(boundary), capacity(capacity), divided(false), depth(depth),
            northeast(nullptr), northwest(nullptr), southeast(nullptr), southwest(nullptr), parent(parent), position(position), number_of_points(0) {
                if (capacity < 1) {
                    throw std::range_error("capacity must be greater than 0");
                }
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

            this->northeast = new QuadTree(neRect, capacity, depth + 1, this, "northeast");
            this->northwest = new QuadTree(nwRect, capacity, depth + 1, this, "northwest");
            this->southeast = new QuadTree(seRect, capacity, depth + 1, this, "southeast");
            this->southwest = new QuadTree(swRect, capacity, depth + 1, this, "southwest");

            for (const auto& point : points) {
                if (this->northeast->insert(point) || this->northwest->insert(point) || 
                    this->southeast->insert(point) || this->southwest->insert(point)) {
                    continue;
                }
            }

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
                northeast->query(range, found);
                northwest->query(range, found);
                southeast->query(range, found);
                southwest->query(range, found);
            }

            return ;
        }

        QuadTree* find_quadtree(QuadTree* quad_tree, const Point& point) const {
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
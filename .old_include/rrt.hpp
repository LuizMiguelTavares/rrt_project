#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#include "grid_map.hpp"
#include "QTree.hpp"

namespace motion_planning {
    class Node {
        public:
            std::vector<double> position;
            Node* parent;
            int x;
            int y;
            bool is_empty;

            Node(std::vector<double> pos, Node* par = nullptr); 
            Node(); 

            ~Node();

            bool empty();

            void setParent(Node* par);

            void setPosition(std::vector<double> pos);
    };

    class RRT {
        public:
            RRT(const cv::Mat& map, Node* start, Node* goal, int num_nodes, double step_size, double goal_threshold, double bias_probability);
            std::vector<Node*> run();
            void plot(const std::vector<Node*>& nodes, const Node* start, const Node* end, bool reached);
            std::vector<Node*> traceGoalPath(Node* goal_node);
            virtual ~RRT();

        private:
            cv::Mat map_;
            Node* start_;
            Node* goal_;
            int num_nodes_;
            double step_size_;
            double goal_threshold_;
            double bias_probability_;

            Node* biasedSample();
            Node* steer(Node* nearest, const Node& sample) const;
            double distance(const Node& node1, const Node& node2) const;
            bool checkObstacleIntersection(const Node* start, const Node* end) const;
    };
} // namespace motion_planning
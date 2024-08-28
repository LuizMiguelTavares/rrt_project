#pragma once

#include <vector>
#include <Eigen/Dense>

namespace potential{
    class ObstacleAvoidance {
    private:
        double n, a, b, k, lambda;
        Eigen::Vector2d J;
        double V;

    public:
        ObstacleAvoidance(double n, double a, double b, double k, double lambda=0.1);

        double calculate_V(double x_diff, double y_diff);

        Eigen::Vector2d j_ob(double v, double x_diff, double y_diff);

        std::pair<double, double> obstacle_avoidance(const Eigen::Vector2d& robot_point, const Eigen::Vector2d& obstacle_points);

        std::pair<double, double> calculate_diff(double x_dot, double y_dot, const Eigen::Vector2d& combined_gradient);
        
        Eigen::Vector2d get_J() const;

        double get_v() const;
    };
} // namespace potential
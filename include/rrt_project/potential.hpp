#pragma once

#include <vector>
#include <Eigen/Dense>

namespace potential{
    class ObstacleAvoidance {
    private:
        double n, a, b, k;
        Eigen::MatrixXd J;

    public:
        ObstacleAvoidance(double n, double a, double b, double k);

        double calculate_V(double x_diff, double y_diff);

        Eigen::Vector2d j_ob(double v, double x_diff, double y_diff);

        std::pair<double, double> obstacle_avoidance(const Eigen::Vector2d& robot_point, const std::vector<Eigen::Vector2d>& obstacle_points);

        Eigen::MatrixXd get_J() const;
    };
} // namespace potential
#include "potential.hpp"
#include <cmath>
#include <stdexcept>

namespace potential {
    ObstacleAvoidance::ObstacleAvoidance(double n, double a, double b, double k, double lambda) : n(n), a(a), b(b), k(k), lambda(lambda) {
        if (n == 0 || a == 0 || b == 0 || k == 0) {
            throw std::invalid_argument("The obstacle detection constants must be declared");
        }
        J.resize(2, 1);
    }

    double ObstacleAvoidance::calculate_V(double x_diff, double y_diff) {
        return std::exp(-std::pow(x_diff, n)/a) * std::exp(-std::pow(y_diff, n)/b);
    }

    Eigen::Vector2d ObstacleAvoidance::j_ob(double v, double x_diff, double y_diff) {
        Eigen::Vector2d result;
        result(0) = -v * n * std::pow(x_diff, n - 1) / a;
        result(1) = -v * n * std::pow(y_diff, n - 1) / b;
        return result;
    }

    std::pair<double, double> ObstacleAvoidance::obstacle_avoidance(const Eigen::Vector2d& robot_point, const Eigen::Vector2d& obstacle_point) {
        double x_diff = robot_point(0) - obstacle_point(0);
        double y_diff = robot_point(1) - obstacle_point(1);
    
        double v = calculate_V(x_diff, y_diff);
        V = v;

        Eigen::Vector2d J_ob = j_ob(v, x_diff, y_diff);
        J = J_ob;
        double v_ref = k * (-v);

        Eigen::Matrix2d lambda_Job = lambda * Eigen::Matrix2d::Identity();

        Eigen::Matrix2d J_ob_T_J_ob = J_ob * J_ob.transpose() + lambda_Job;
        Eigen::Vector2d pseudo_inv_term = J_ob.transpose() * J_ob_T_J_ob.inverse() * v_ref;

        double x_dot = pseudo_inv_term(0);
        double y_dot = pseudo_inv_term(1);

        return std::make_pair(x_dot, y_dot);
    }

    Eigen::Vector2d ObstacleAvoidance::get_J() const {
        return J;
    }

    double ObstacleAvoidance::get_v() const {
        return V;
    }
} // namespace potential
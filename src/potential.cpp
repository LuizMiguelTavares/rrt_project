// #include "potential.hpp"
// #include <cmath>
// #include <stdexcept>

// namespace potential{
//     ObstacleAvoidance::ObstacleAvoidance(double n, double a, double b, double k) : n(n), a(a), b(b), k(k) {
//         if (n == 0 || a == 0 || b == 0 || k == 0) {
//             throw std::invalid_argument("The obstacle detection constants must be declared");
//         }
//         J.resize(2, 1); // Ensure J is properly sized
//     }

//     double ObstacleAvoidance::calculate_V(double x_diff, double y_diff) {
//         return std::exp(-std::pow(x_diff / a, n)) * std::exp(-std::pow(y_diff / b, n));
//     }

//     Eigen::Vector2d ObstacleAvoidance::j_ob(double v, double x_diff, double y_diff) {
//         Eigen::Vector2d result;
//         result(0) = -v * n * std::pow(x_diff, n - 1) / std::pow(a, n);
//         result(1) = -v * n * std::pow(y_diff, n - 1) / std::pow(b, n);
//         return result;
//     }

//     std::pair<double, double> ObstacleAvoidance::obstacle_avoidance(const Eigen::Vector2d& robot_point, const std::vector<Eigen::Vector2d>& obstacle_points) {
//         double x_dot = 0, y_dot = 0;

//         for (const auto& obstacle_point : obstacle_points) {
//             double x_diff = robot_point(0) - obstacle_point(0);
//             double y_diff = robot_point(1) - obstacle_point(1);

//             double v = calculate_V(x_diff, y_diff);

//             Eigen::Vector2d J_ob = j_ob(v, x_diff, y_diff);
//             J = J_ob;
//             double v_ref = k * (-v);

//             Eigen::MatrixXd J_ob_reshaped(2, 1);
//             J_ob_reshaped << J_ob(0), J_ob(1);

//             Eigen::MatrixXd v_ref_matrix(1, 1);
//             v_ref_matrix << v_ref;

//             Eigen::Vector2d x_dot_aux = J_ob_reshaped.completeOrthogonalDecomposition().pseudoInverse() * v_ref_matrix;

//             x_dot += x_dot_aux(0);
//             y_dot += x_dot_aux(1);
//         }

//         return std::make_pair(x_dot, y_dot);
//     }

//     Eigen::MatrixXd ObstacleAvoidance::get_J() const {
//         return J;
//     }
// }

#include "potential.hpp"
#include <cmath>
#include <stdexcept>

namespace potential {
    ObstacleAvoidance::ObstacleAvoidance(double n, double a, double b, double k, double lambda) : n(n), a(a), b(b), k(k), lambda(lambda) {
        if (n == 0 || a == 0 || b == 0 || k == 0) {
            throw std::invalid_argument("The obstacle detection constants must be declared");
        }
        J.resize(2, 1); // Ensure J is properly sized
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

        Eigen::Vector2d J_ob = j_ob(v, x_diff, y_diff);
        J = J_ob;
        double v_ref = k * (-v);

        // Convert lambda_Job to an Eigen matrix
        Eigen::Matrix2d lambda_Job = lambda * Eigen::Matrix2d::Identity();

        // Perform the matrix operations
        Eigen::Matrix2d J_ob_T_J_ob = J_ob * J_ob.transpose() + lambda_Job;
        Eigen::Vector2d pseudo_inv_term = J_ob.transpose() * J_ob_T_J_ob.inverse() * v_ref;

        double x_dot = pseudo_inv_term(0);
        double y_dot = pseudo_inv_term(1);

        return std::make_pair(x_dot, y_dot);
    }

    Eigen::MatrixXd ObstacleAvoidance::get_J() const {
        return J;
    }
}

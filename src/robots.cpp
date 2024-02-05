#include "robots.hpp"
#include <cmath>

Eigen::Vector3f DifferentialRobot::computeDynamics()
{
    // Limit linear acceleration
    if (std::abs(acceleration[0]) > maxAcceleration[0]) {
        acceleration[0] = maxAcceleration[0] * std::copysign(1, acceleration[0]);
    }

    // Update linear velocity within limits
    velocity += acceleration * dt;
    if (std::abs(velocity[0]) > maxVelocity[0]) {
        velocity[0] = maxVelocity[0] * std::copysign(1, velocity[0]);
    }

    // Limit angular acceleration
    if (std::abs(angularAcceleration[2]) > maxAngularAcceleration[2]) {
        angularAcceleration[2] = maxAngularAcceleration[2] * std::copysign(1, angularAcceleration[2]);
    }

    // Update angular velocity within limits
    angularVelocity[2] += angularAcceleration[2] * dt;
    if (std::abs(angularVelocity[2]) > maxAngularVelocity[2]) {
        angularVelocity[2] = maxAngularVelocity[2] * std::copysign(1, angularVelocity[2]);
    }

    // Calculate change in orientation
    float deltaTheta = angularVelocity[2] * dt;

    // Check for non-zero angular velocity to update position based on the arc
    if (std::abs(angularVelocity[2]) > std::numeric_limits<float>::epsilon()) {
        float R = velocity[0] / angularVelocity[2]; // Calculate radius of the curvature
        position[0] += -R * sin(orientation[2]) + R * sin(orientation[2] + deltaTheta);
        position[1] += R * cos(orientation[2]) - R * cos(orientation[2] + deltaTheta);
    } else {
        // If angular velocity is effectively zero, move in a straight line
        position[0] += velocity[0] * cos(orientation[2]) * dt;
        position[1] += velocity[0] * sin(orientation[2]) * dt;
    }

    // Update orientation based on angular velocity
    orientation[2] += deltaTheta;

    // Normalize orientation to keep it within a valid range
    orientation[2] = std::fmod(orientation[2], 2 * M_PI);
    if (orientation[2] < -M_PI) orientation[2] += 2 * M_PI;
    else if (orientation[2] > M_PI) orientation[2] -= 2 * M_PI;

    return Eigen::Vector3f::Zero();
}

Eigen::Vector3f DifferentialRobot::computeDynamicsSimple(){
    return Eigen::Vector3f::Zero();
}

Eigen::Vector3f DifferentialRobot::closestPointOnPath(Eigen::Vector3f point)
{
    return point;
}
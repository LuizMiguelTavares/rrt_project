# pragma once
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

class RobotBase {
public:
    RobotBase()
    : position(Eigen::Vector3f::Zero()), orientation(Eigen::Vector3f::Zero()), velocity(Eigen::Vector3f::Zero()), acceleration(Eigen::Vector3f::Zero()),
      angularVelocity(Eigen::Vector3f::Zero()), angularAcceleration(Eigen::Vector3f::Zero()), maxVelocity(Eigen::Vector3f::Zero()),
      maxAcceleration(Eigen::Vector3f::Zero()), maxAngularVelocity(Eigen::Vector3f::Zero()), dt(0.01f)
    {}

    virtual ~RobotBase() {}

    // Common attributes
    Eigen::Vector3f position;
    Eigen::Vector3f orientation;
    Eigen::Vector3f velocity;
    Eigen::Vector3f acceleration;
    Eigen::Vector3f angularVelocity;
    Eigen::Vector3f angularAcceleration;
    Eigen::Vector3f maxVelocity;
    Eigen::Vector3f maxAcceleration;
    Eigen::Vector3f maxAngularVelocity;
    Eigen::Vector3f maxAngularAcceleration;
    float dt;

    // Common methods
    virtual Eigen::Vector3f closestPointOnPath(Eigen::Vector3f point) = 0;
    virtual Eigen::Vector3f computeDynamics() = 0;

    // Setters
    void setPosition(const Eigen::Vector3f& pos) { position = pos; }
    void setOrientation(const Eigen::Vector3f& ori) { orientation = ori; }
    void setVelocity(const Eigen::Vector3f& vel) { velocity = vel; }
    void setAcceleration(const Eigen::Vector3f& acc) { acceleration = acc; }
    void setAngularVelocity(const Eigen::Vector3f& angVel) { angularVelocity = angVel; }
    void setAngularAcceleration(const Eigen::Vector3f& angAcc) { angularAcceleration = angAcc; }
    void setMaxVelocity(const Eigen::Vector3f& maxVel) { maxVelocity = maxVel; }
    void setMaxAcceleration(const Eigen::Vector3f& maxAcc) { maxAcceleration = maxAcc; }
    void setMaxAngularVelocity(const Eigen::Vector3f& maxAngVel) { maxAngularVelocity = maxAngVel; }
    void setMaxAngularAcceleration(const Eigen::Vector3f& maxAngAcc) { maxAngularAcceleration = maxAngAcc; }
    void setDt(float timeStep) { dt = timeStep; }

    // Getters
    const Eigen::Vector3f& getPosition() const { return position; }
    const Eigen::Vector3f& getOrientation() const { return orientation; }
    const Eigen::Vector3f& getVelocity() const { return velocity; }
    const Eigen::Vector3f& getAcceleration() const { return acceleration; }
    const Eigen::Vector3f& getAngularVelocity() const { return angularVelocity; }
    const Eigen::Vector3f& getAngularAcceleration() const { return angularAcceleration; }
    const Eigen::Vector3f& getMaxVelocity() const { return maxVelocity; }
    const Eigen::Vector3f& getMaxAcceleration() const { return maxAcceleration; }
    const Eigen::Vector3f& getMaxAngularVelocity() const { return maxAngularVelocity; }
    const Eigen::Vector3f& getMaxAngularAcceleration() const { return maxAngularAcceleration; }
    float getDt() const { return dt; }
};

class DifferentialRobot : public RobotBase 
{
public:
    DifferentialRobot() : RobotBase() {}

    DifferentialRobot(const Eigen::Vector3f& pos, const Eigen::Vector3f& ori, const Eigen::Vector3f& vel, const Eigen::Vector3f& acc,
                  const Eigen::Vector3f& angVel, const Eigen::Vector3f& angAcc,const Eigen::Vector3f& maxVel, const Eigen::Vector3f& maxAcc,
                  const Eigen::Vector3f& maxAngVel, const Eigen::Vector3f& maxAngAcc,float timeStep, cv::Mat shape) 
    {
    position = pos;
    orientation = ori;
    velocity = vel;
    acceleration = acc;
    angularVelocity = angVel;
    angularAcceleration = angAcc;
    maxVelocity = maxVel;
    maxAcceleration = maxAcc;
    maxAngularVelocity = maxAngVel;
    maxAngularAcceleration = maxAngAcc;
    dt = timeStep;
    robot_shape = shape;
    }

    virtual ~DifferentialRobot() {}

    // Attributes
    cv::Mat robot_shape;

    // Methods
    virtual Eigen::Vector3f computeDynamics() override;
    virtual Eigen::Vector3f computeDynamicsSimple();
    virtual Eigen::Vector3f closestPointOnPath(Eigen::Vector3f point) override;
};
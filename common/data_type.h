// Author: Van Thanh Nguyen


#pragma once
#include <Eigen/Dense>

// alias for convinient uses
// Vector types
using VectorXd  = Eigen::VectorXd;
using Vector2d  = Eigen::Vector2d;
using Vector3d  = Eigen::Vector3d;
using Vector4d  = Eigen::Vector4d;

// Matrix types
using MatrixXd  = Eigen::MatrixXd;
using Matrix2d  = Eigen::Matrix2d;
using Matrix3d  = Eigen::Matrix3d;
using Matrix4d  = Eigen::Matrix4d;

// Rotation / Transform
using Quat  = Eigen::Quaterniond; // q = w + xi + yj +zk;
using AngleAxis = Eigen::AngleAxisd;

// Jacobian matrix
// Pinocchio frame Jacobian: 6 rows (3 linear + 3 angular), nv cols (dynamic)
using Jacobian6 = Eigen::Matrix<double, 6, Eigen::Dynamic>; // 6 x nv
using JacobianX = Eigen::MatrixXd;                          // general dynamic Jacobian

struct JointState
{
    int robot_na; // number of actuated joint

    VectorXd qj; // joint position vector
    VectorXd dqj; // joint velocity vector
    VectorXd torquej; // joint torque 

    // constructor
    JointState (int naIn) : robot_na(naIn)
    {
        // init member variables
        qj = VectorXd::Zero(robot_na);
        dqj = VectorXd::Zero(robot_na);
        torquej = VectorXd::Zero(robot_na);
    }
};

struct IMUSensor
{
    Vector3d imu_accel_L = Vector3d::Zero(); // imu acceleration, in local imu frame
    Vector3d imu_gyro_L = Vector3d::Zero(); // imu angular velocity/ gyroscope in local frame
    Quat  imu_quat_ = Quat::Identity(); // imu quaternion
};

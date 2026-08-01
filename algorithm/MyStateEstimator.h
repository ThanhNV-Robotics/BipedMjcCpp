#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

#include "data_bus.h"

class StateEstimator
{
    public:
        
        StateEstimator(double dt, bool verbose ); //Constructor, input: sampling time dt
        void getSensorMeansurement (DataBus& Data); // get sensor measurement from mujoco simulator

        void update();

        // in our biped 12-dof robot, measurement includes:
        // joint states: position, velocity, torque
        // imu: quaternion, local acceleration, local angular velocity  

        Eigen::Matrix<double, 4,1> getImuquaternion();
        Eigen::Matrix<double, 12,1> get_qj(); // return joint position
        Eigen::Matrix<double, 12,1> get_qjd(); // return joint velocity
        // Eigen::Matrix<double, 6,1> get_qb(); // return base pose (position, rpy)
        // Eigen::Matrix<double, 6,1> get_qbd(); // return base linear velocity and angular velocity

    private:
        double dt_;
        // int model_nv = 12;
        // measurement variables
        Eigen::Matrix<double, 3,1> imu_acceleration_mea_, imu_angular_vel_mea_; // w.r.t imu/base's local frame
        Eigen::Matrix<double, 12,1> joint_pos_mea_, joint_vel_mea_, joint_tor_mea_;
        Eigen::Matrix<double, 4,1> imu_quaternion_; // our robot imu itself can estimate its orientation w.r.t world frame
        Eigen::Matrix<double, 3,1> imu_rpy_; // imu roll, pitch, yaw
        
        
        Eigen::Matrix<double, 3,1> base_pos_est_, base_linearVel_est_;  
        
        // Kalman Filter for base position and linear velocity estimation
        // Prediction model:
        // x_k+1 = A_*x_k + B_*a_k
        // y_k = C_*x_k + v
        // where a_k is the imu acceleration w.r.t world frame
        // v process noise
        Eigen::Matrix<double, -1, -1> A_, B_, C_, Q_, P_, R_;
        Eigen::Matrix<double, -1, -1> x_hat_, ps_, vs_;
        const int numContact_ = 2; // 2 contact leg
        const int dimContact_ = 3*this->numContact_; // 6, dim 3 is the feet cartein position

// observation is the foot position and velocity measurement  w.r.t the base frame and foot height w.r.t world frame consider that the foot is in contact to the ground.
        const int dimObserve_ = 2*this->dimContact_ + this->numContact_; // 12 + 2 = 14

        const int dimState_ = 6 + this->dimContact_; // base pose + 2 foot position = 6 +3 +3 = 12
        Eigen::Matrix<double, -1, 1> feetHeights_; //

};
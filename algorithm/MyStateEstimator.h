#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

#include "data_bus.h"

class StateEstimator
{
    public:
        
        StateEstimator(double dt); //Constructor, input: sampling time dt
        void getSensorMeansurement (DataBus& Data); //
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

};
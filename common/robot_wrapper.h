#pragma once

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>


#include <pinocchio/multibody/fwd.hpp>
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/aba.hpp"


#include "data_bus.h"
#include <string>
#include "json/json.h"
#include <vector>

#include "data_type.h"
#include <exception>

class RobotWrapper {
    private:
        // must be declared (and therefore constructed) before the const model_*
        // attributes below, since those are initialized from pin_model_/pin_data_
        // in the constructor's member-initializer list.
        pinocchio::Model pin_model_;
        pinocchio::Data pin_data_;

    public:
        const int model_nq_; // position vector dim
        const int model_nv_; // velocity vector dim
        const int model_njoint_; // number of joint
        const int model_na_; // number of actuated joint
        void printModelInfo ();

        void updateJointState (JointState& joint_state); // update q from input robot sensor

        Jacobian6 computeLeftFeetJointJacobianGlobal(JointState& joint_state, IMUSensor& imu_sensor);
        //Constructor
        RobotWrapper(const std::string& urdf_path);

    private:


        std::vector<pinocchio::JointIndex> left_leg_joint_ids_;  // joint IDs in left leg subtree
        std::vector<pinocchio::JointIndex> right_leg_joint_ids_; // joint IDs in right leg subtree

        // std::vector<double> min_joint_pos_;
        // std::vector<double> max_joint_pos_;
        // std::vector<double> min_joint_vel_;
        // std::vector<double> max_joint_vel_;
        // std::vector<double> min_joint_torque_;
        // std::vector<double> min_joint_torque_;

        JointState joint_state_;

        // VectorXd ddq; // joint acceleration
        Vector3d imu_accel_L_; // imu acceleration, in local imu frame
        Vector3d imu_gyro_L_; // imu angular velocity/ gyroscope in local frame
        Quat  imu_quat_W_; // imu quaternion, w.r.t global frame

        std::string robot_urdf_path_;
};
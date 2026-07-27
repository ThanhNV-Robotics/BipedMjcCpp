/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024-2025 Humanoid Robot (Shanghai) Co., Ltd.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#pragma once

#include "pinocchio/parsers/urdf.hpp"
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

class Pin_KinDyn
{
public:
    std::vector<bool> motorReachLimit;
    const std::vector<std::string> motorName = {"right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
                                                "right_knee_pitch_joint", "right_ankle_roll_joint", "right_ankle_pitch_joint",
                                                "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
                                                "left_knee_pitch_joint","left_ankle_roll_joint", "left_ankle_pitch_joint"}; // joint name in urdf and jason config files
    Eigen::VectorXd motorMaxTorque;
    Eigen::VectorXd motorMaxPos;
    Eigen::VectorXd motorMinPos;

    Eigen::VectorXd tauJointOld;
    std::string urdf_path;
    pinocchio::Model model_biped;
    pinocchio::Model model_biped_fixed;
    int model_nv;
    pinocchio::JointIndex r_ankle_joint, l_ankle_joint, base_joint, r_hip_joint, l_hip_joint, r_hip_roll_joint, l_hip_roll_joint;
    pinocchio::JointIndex r_ankle_joint_fixed, l_ankle_joint_fixed, r_hip_joint_fixed, l_hip_joint_fixed;
    Eigen::VectorXd q, dq, ddq;
    Eigen::Matrix3d Rcur;
    Eigen::Quaternion<double> quatCur;
    Eigen::Matrix<double, 6, -1> J_r, J_l, J_base, J_r_body, J_l_body;
    Eigen::Matrix<double, 6, -1> dJ_r, dJ_l, dJ_base;
    Eigen::Matrix<double, 3, -1> Jcom;
    Eigen::Vector3d fe_r_pos, fe_l_pos, base_pos; // foot-end position in world frame
    Eigen::Vector3d fe_r_pos_body, fe_l_pos_body; // foot-end position in body frame
    Eigen::Vector3d fe_r_vel_body, fe_l_vel_body; // foot-end velcity in body frame
    Eigen::Vector3d hip_r_pos, hip_l_pos;
    Eigen::Vector3d hip_r_pos_body, hip_l_pos_body;
    Eigen::Matrix3d fe_r_rot, fe_l_rot, base_rot;
    Eigen::Matrix3d fe_r_rot_body, fe_l_rot_body;
    Eigen::MatrixXd dyn_M, dyn_M_inv, dyn_C, dyn_G, dyn_Ag, dyn_dAg;
    Eigen::VectorXd dyn_Non;
    Eigen::Vector3d CoM_pos;
    Eigen::Matrix3d inertia;
    enum legIdx
    {
        left,
        right
    };
    struct IkRes
    {
        int status;
        int itr;
        Eigen::VectorXd err;
        Eigen::VectorXd jointPosRes;
    };

    Pin_KinDyn(std::string urdf_pathIn);
    void dataBusRead(DataBus const &robotState);
    void dataBusWrite(DataBus &robotState);
    void computeJ_dJ();
    void computeDyn();
    IkRes computeInK_Leg(const Eigen::Matrix3d &Rdes_L, const Eigen::Vector3d &Pdes_L, const Eigen::Matrix3d &Rdes_R, const Eigen::Vector3d &Pdes_R);
    Eigen::VectorXd integrateDIY(const Eigen::VectorXd &qI, const Eigen::VectorXd &dqI);
    static Eigen::Quaterniond intQuat(const Eigen::Quaterniond &quat, const Eigen::Matrix<double, 3, 1> &w);
    void workspaceConstraint(Eigen::VectorXd &qFT, Eigen::VectorXd &tauJointFT);

private:
    pinocchio::Data data_biped, data_biped_fixed;
};

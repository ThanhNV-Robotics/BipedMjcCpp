/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024-2025 Humanoid Robot (Shanghai) Co., Ltd.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/

//-------------------------------------------NOTE------------------------------------//
//
// The damping(Kd) in the joint_ctrl_config.yaml is relatively large, and they may not match the real ones.
//
//-----------------------------------------------------------------------------------//
#pragma once
#include <yaml-cpp/yaml.h>
#include <string>
#include "LPF_fst.h"
#include <vector>
#include <cmath>
#include "data_type.h"
#include "robot_wrapper.h"
#include "KinWBC.h"

// PVT: Position Velocity Torque (control)
// This class is for joint low-level 
class PVT_Ctr {
public:
    int jointNum;

    // Current feedback state (from getFeedbackMotorState)
    VectorXd motor_pos_cur;     // current joint positions  [jointNum]
    VectorXd motor_vel;         // current joint velocities [jointNum]

    // Desired references
    VectorXd motor_pos_des;     // P desired [jointNum]
    VectorXd motor_vel_des;     // V desired [jointNum]
    VectorXd motor_tor_des;     // T feedforward [jointNum]
    VectorXd motor_pos_des_old; // previous P desired (for delta-limit overload) [jointNum]

    // Torque outputs — kept as std::vector<double> for MJ_Interface::setMotorsTorque() compatibility
    std::vector<double> motor_tor_out_link;  // output torque at link  [jointNum]
    std::vector<double> motor_tor_out_motor; // output torque at motor [jointNum]

    // PD gains and limits
    VectorXd pvt_Kp;   // proportional gain [jointNum]
    VectorXd pvt_Kd;   // derivative gain   [jointNum]
    VectorXd maxTor;   // torque limit      [jointNum]
    VectorXd maxVel;   // velocity limit    [jointNum]
    VectorXd maxPos;   // max joint pos     [jointNum]
    VectorXd minPos;   // min joint pos     [jointNum]
    VectorXd gear;     // gear ratio        [jointNum]

    PVT_Ctr(double timeStepIn, const char * yamlPath);
    void calMotorsPVT();
    void calMotorsPVT(double deltaP_Lim);
    void calMotorsPVT(VectorXd ref_pos, VectorXd ref_vel, VectorXd tau_ff);
    void calMotorsPVT(KinWBC &kin_wbc);
    void enablePV();                        // enable PV for all joints
    void disablePV();                       // disable PV for all joints
    void enablePV(int jtId);
    void disablePV(int jtId);
    void setJointPD(double kp, double kd, const char * jointName);

    void getFeedbackMotorState(RobotWrapper &robot_wrapper);

    void printPVTinfo();
    void printTorqueOut();

    void genTestTrajectory(double t);

    // Joint order matching motor_pos_des etc. -- callers must map by name.
    const std::vector<std::string> &getMotorNames() const { return motorName; }

private:
    std::vector<LPF_Fst> tau_out_lpf;
    std::vector<LPF_Fst> traj_pos_lpf;
    std::vector<LPF_Fst> traj_vel_lpf;
    Eigen::VectorXi PV_enable;  // 1 = PV active, 0 = disabled [jointNum]
    double sign(double in);
    std::vector<std::string> motorName;
};

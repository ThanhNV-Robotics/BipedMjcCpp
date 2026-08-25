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
#include "data_bus.h"
#include "data_type.h"
#include "robot_wrapper.h"

// PVT: Position Velocity Torque (control)
// This class is for joint low-level 
class PVT_Ctr {
public:
    int jointNum;
    std::vector<double> motor_pos_cur;
    std::vector<double> motor_pos_des_old;
    std::vector<double> motor_vel;
    std::vector<double> motor_tor_out_link; // final tau output
    std::vector<double> motor_tor_out_motor; // final tau output
    PVT_Ctr(double timeStepIn, const char * yamlPath);
    void calMotorsPVT();
    void calMotorsPVT(double deltaP_Lim);
    void calMotorsPVT (VectorXd ref_pos, VectorXd ref_vel, VectorXd tau_ff);
    void enablePV(); // enable PV control item
    void disablePV(); // disable PV control item
    void enablePV(int jtId); // enable PV control item
    void disablePV(int jtId); // disable PV control item
    void setJointPD(double kp, double kd, const char * jointName);
    void dataBusRead(DataBus &busIn);
    void dataBusWrite(DataBus &busIn);

    void getFeedbackMotorState (RobotWrapper &robot_wrapper);

    void printPVTinfo();
    void printTorqueOut();

    void genTestTrajectory(double t);

    // Joint order matching motor_pos_des/motor_vel_des/motor_tor_des and every
    // other per-joint vector below -- callers must map onto this order by
    // name, not assume it matches any other model's/file's joint order (this
    // one is the config file's top-level keys, sorted alphabetically in the
    // constructor since yaml-cpp otherwise preserves the file's own order).
    const std::vector<std::string> &getMotorNames() const { return motorName; }

    std::vector<double> motor_pos_des; // P des
    std::vector<double> motor_vel_des; // V des
    std::vector<double> motor_tor_des; // T des

    std::vector<double> pvt_Kp;
    std::vector<double> pvt_Kd;
    std::vector<double> maxTor;
    std::vector<double> maxVel;
    std::vector<double> maxPos;
    std::vector<double> minPos;
    std::vector<double> gear;

private:
    std::vector<LPF_Fst> tau_out_lpf;
    std::vector<LPF_Fst> traj_pos_lpf; // smooths genTestTrajectory's reference position
    std::vector<LPF_Fst> traj_vel_lpf; // smooths genTestTrajectory's reference velocity
    std::vector<int> PV_enable;
    double sign(double in);
    std::vector<std::string> motorName; // joint names, populated from yamlPath's top-level keys in the constructor
};



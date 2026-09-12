/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024-2025 Humanoid Robot (Shanghai) Co., Ltd.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/

#include "PVT_ctrl.h"
#include <algorithm>

PVT_Ctr::PVT_Ctr(double timeStepIn, const char *yamlPath) {
    YAML::Node root_read = YAML::LoadFile(yamlPath);

    for (const auto &kv : root_read)
        motorName.push_back(kv.first.as<std::string>());
    jointNum = motorName.size();

    // Resize LPF vectors
    tau_out_lpf.assign(jointNum, LPF_Fst());
    traj_pos_lpf.assign(jointNum, LPF_Fst());
    traj_vel_lpf.assign(jointNum, LPF_Fst());

    // Resize all Eigen vectors to zero
    motor_pos_cur     = VectorXd::Zero(jointNum);
    motor_vel         = VectorXd::Zero(jointNum);
    motor_pos_des     = VectorXd::Zero(jointNum);
    motor_vel_des     = VectorXd::Zero(jointNum);
    motor_tor_des     = VectorXd::Zero(jointNum);
    motor_pos_des_old = VectorXd::Zero(jointNum);
    pvt_Kp            = VectorXd::Zero(jointNum);
    pvt_Kd            = VectorXd::Zero(jointNum);
    maxTor            = VectorXd::Constant(jointNum, 400.0);
    maxVel            = VectorXd::Constant(jointNum, 50.0);
    maxPos            = VectorXd::Constant(jointNum, 3.14);
    minPos            = VectorXd::Constant(jointNum, -3.14);
    gear              = VectorXd::Ones(jointNum);
    PV_enable         = Eigen::VectorXi::Ones(jointNum);

    // std::vector outputs (MJ_Interface boundary)
    motor_tor_out_link .assign(jointNum, 0.0);
    motor_tor_out_motor.assign(jointNum, 0.0);

    // Load per-joint parameters from YAML
    for (int i = 0; i < jointNum; i++) {
        const YAML::Node &jt = root_read[motorName[i]];
        pvt_Kp(i)  = jt["kp"].as<double>();
        pvt_Kd(i)  = jt["kd"].as<double>();
        maxTor(i)  = jt["maxTorque"].as<double>();
        maxVel(i)  = jt["maxSpeed"].as<double>();
        maxPos(i)  = jt["maxPos"].as<double>();
        minPos(i)  = jt["minPos"].as<double>();
        gear(i)    = jt["gear"].as<double>();

        double fc = jt["PVT_LPF_Fc"].as<double>();
        tau_out_lpf[i].setPara(fc, timeStepIn);
        tau_out_lpf[i].ftOut(0);

        const double trajLPF_Fc = 1.0;
        traj_pos_lpf[i].setPara(trajLPF_Fc, timeStepIn);
        traj_vel_lpf[i].setPara(trajLPF_Fc, timeStepIn);
        traj_pos_lpf[i].ftOut(0);
        traj_vel_lpf[i].ftOut(0);
    }
}

void PVT_Ctr::getFeedbackMotorState(RobotWrapper &robot_wrapper)
{
    // q = [base_pos(3), base_quat(4), joints(n)]  → joints start at index 7
    // dq = [base_lin(3), base_ang(3), joints(n)]   → joints start at index 6
    motor_pos_cur = robot_wrapper.getQ().segment(7, jointNum);
    motor_vel     = robot_wrapper.getDq().segment(6, jointNum);
}

void PVT_Ctr::setJointPD(double kp, double kd, const char *jointName) {
    auto it = std::find(motorName.begin(), motorName.end(), jointName);
    if (it == motorName.end()) { std::cout << jointName << " NOT found!" << std::endl; return; }
    int id = std::distance(motorName.begin(), it);
    pvt_Kp(id) = kp;
    pvt_Kd(id) = kd;
}

// Basic PD control using motor_pos_des / motor_vel_des / motor_tor_des
void PVT_Ctr::calMotorsPVT() {
    for (int i = 0; i < jointNum; i++) {
        double tauDes = PV_enable(i) * pvt_Kp(i) * (motor_pos_des(i) - motor_pos_cur(i))
                      + PV_enable(i) * pvt_Kd(i) * (motor_vel_des(i) - motor_vel(i));
        tauDes = tau_out_lpf[i].ftOut(tauDes) + motor_tor_des(i);
        if (std::fabs(tauDes) >= std::fabs(maxTor(i)))
            tauDes = sign(tauDes) * maxTor(i);
        motor_tor_out_motor[i] = tauDes / gear(i);
        motor_tor_out_link[i]  = tauDes;
        motor_pos_des_old(i)   = motor_pos_des(i);
    }
}

// PD control with per-step delta-position clamping
void PVT_Ctr::calMotorsPVT(double deltaP_Lim) {
    for (int i = 0; i < jointNum; i++) {
        double delta = motor_pos_des(i) - motor_pos_des_old(i);
        if (std::fabs(delta) >= std::fabs(deltaP_Lim))
            delta = deltaP_Lim * sign(delta);
        double pDes = delta + motor_pos_des_old(i);
        double tauDes = PV_enable(i) * pvt_Kp(i) * (pDes - motor_pos_cur(i))
                      + PV_enable(i) * pvt_Kd(i) * (motor_vel_des(i) - motor_vel(i));
        tauDes = tau_out_lpf[i].ftOut(tauDes) + motor_tor_des(i);
        if (std::fabs(tauDes) >= std::fabs(maxTor(i)))
            tauDes = sign(tauDes) * maxTor(i);
        motor_tor_out_motor[i] = tauDes / gear(i);
        motor_tor_out_link[i]  = tauDes;
        motor_pos_des_old(i)   = pDes;
    }
}

// Explicit reference override (bypasses motor_pos_des/vel_des/tor_des members)
void PVT_Ctr::calMotorsPVT(VectorXd ref_pos, VectorXd ref_vel, VectorXd tau_ff)
{
    for (int i = 0; i < jointNum; i++) {
        double tauDes = PV_enable(i) * pvt_Kp(i) * (ref_pos(i) - motor_pos_cur(i))
                      + PV_enable(i) * pvt_Kd(i) * (ref_vel(i) - motor_vel(i))
                      + tau_ff(i);
        if (std::fabs(tauDes) >= std::fabs(maxTor(i)))
            tauDes = sign(tauDes) * maxTor(i);
        motor_tor_out_motor[i] = tauDes / gear(i);
        motor_tor_out_link[i]  = tauDes;
    }
}

// KinWBC-driven: update des from IK output then run PD
void PVT_Ctr::calMotorsPVT(KinWBC &kin_wbc)
{
    motor_pos_des = motor_pos_cur + kin_wbc.out_delta_q.tail(jointNum);
    motor_vel_des = kin_wbc.out_dq.tail(jointNum);   // KinWBC output velocity
    calMotorsPVT();                                  // reuse base overload
}

double PVT_Ctr::sign(double in) { return (in >= 0) ? 1.0 : -1.0; }

void PVT_Ctr::enablePV()              { PV_enable.setOnes(); }
void PVT_Ctr::disablePV()             { PV_enable.setZero(); }
void PVT_Ctr::enablePV(int jtId)      { PV_enable(jtId) = 1; }
void PVT_Ctr::disablePV(int jtId)     { PV_enable(jtId) = 0; }

void PVT_Ctr::genTestTrajectory(double t) {
    const double start_time    = 2.0;
    const double ramp_duration = 2.0;
    const double omega         = 2 * 3.14159265358979 / 4.0;

    motor_tor_des = VectorXd::Zero(jointNum);

    if (t <= start_time) {
        motor_pos_des = VectorXd::Zero(jointNum);
        motor_vel_des = VectorXd::Zero(jointNum);
        for (int i = 0; i < jointNum; i++) {
            traj_pos_lpf[i].ftOut(0);
            traj_vel_lpf[i].ftOut(0);
        }
        return;
    }

    double traj_time = t - start_time;
    for (int i = 0; i < jointNum; i++) {
        double joint_Center    = (maxPos(i) + minPos(i)) / 2.0;
        double joint_Amplitude = std::max(0.0, 0.9 * (maxPos(i) - minPos(i)) / 2.0);

        double raw_pos, raw_vel;
        if (traj_time < ramp_duration) {
            raw_pos = joint_Center * (traj_time / ramp_duration);
            raw_vel = joint_Center / ramp_duration;
        } else {
            double osc_time = traj_time - ramp_duration;
            raw_pos = joint_Center + joint_Amplitude * std::sin(omega * osc_time);
            raw_vel = joint_Amplitude * omega * std::cos(omega * osc_time);
        }
        motor_pos_des(i) = traj_pos_lpf[i].ftOut(raw_pos);
        motor_vel_des(i) = traj_vel_lpf[i].ftOut(raw_vel);
    }
}

void PVT_Ctr::printPVTinfo() {
    std::printf("Number of joint: %d\n", jointNum);
    std::printf("List of joints:\n");
    for (int i = 0; i < jointNum; i++)
        std::printf("  [%2d] %-25s  kp=%.3f  kd=%.3f  maxTor=%.3f  maxVel=%.3f  minPos=%.5f  maxPos=%.5f  gear=%.3f\n",
                     i, motorName[i].c_str(), pvt_Kp(i), pvt_Kd(i), maxTor(i), maxVel(i), minPos(i), maxPos(i), gear(i));
}

void PVT_Ctr::printTorqueOut() {
    for (int i = 0; i < jointNum; i++)
        std::printf("Torqued at Joint %-25s is: %.4f\n", motorName[i].c_str(), motor_tor_out_motor[i]);
}
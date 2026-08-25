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
    // read joint pvt parameters
    YAML::Node root_read = YAML::LoadFile(yamlPath);

    // joint names come from the config file's top-level keys, in file order
    // (yaml-cpp preserves it) -- this order must match the URDF's joint
    // declaration order (see the note at the top of the yaml file), so it
    // agrees with RobotWrapper's Pinocchio joint order without any reordering
    for (const auto &kv : root_read) {
        motorName.push_back(kv.first.as<std::string>());
    }
    jointNum=motorName.size();

    tau_out_lpf.assign(jointNum,LPF_Fst());
    traj_pos_lpf.assign(jointNum,LPF_Fst());
    traj_vel_lpf.assign(jointNum,LPF_Fst());
    motor_vel.assign(jointNum,0);
    motor_pos_cur.assign(jointNum,0);
    motor_pos_des_old.assign(jointNum,0);
    motor_tor_out_link.assign(jointNum,0);
    motor_tor_out_motor.assign(jointNum,0);
    pvt_Kp.assign(jointNum,0);
    pvt_Kd.assign(jointNum,0);
    maxTor.assign(jointNum,400);
    maxVel.assign(jointNum,50);
    maxPos.assign(jointNum,3.14);
    minPos.assign(jointNum,-3.14);
    PV_enable.assign(jointNum,1);
    gear.assign(jointNum,1.0);

    // update controller parameter from yaml file
    for (int i=0;i<jointNum;i++){
        const YAML::Node &jt = root_read[motorName[i]];
        pvt_Kp[i]=jt["kp"].as<double>();
        pvt_Kd[i]=jt["kd"].as<double>();
        maxTor[i]=jt["maxTorque"].as<double>();
        maxVel[i]=jt["maxSpeed"].as<double>();
        maxPos[i]=jt["maxPos"].as<double>();
        minPos[i]=jt["minPos"].as<double>();
        double fc=jt["PVT_LPF_Fc"].as<double>();
        gear[i] = jt["gear"].as<double>();
        tau_out_lpf[i].setPara(fc, timeStepIn);
        tau_out_lpf[i].ftOut(0);

        const double trajLPF_Fc = 1.0; // cutoff (Hz) for smoothing genTestTrajectory's reference
        traj_pos_lpf[i].setPara(trajLPF_Fc, timeStepIn);
        traj_vel_lpf[i].setPara(trajLPF_Fc, timeStepIn);
        traj_pos_lpf[i].ftOut(0);
        traj_vel_lpf[i].ftOut(0);
    }
}

void PVT_Ctr::dataBusRead(DataBus &busIn) {
    for (int i=0;i<jointNum;i++)
    {
        motor_pos_cur[i]=busIn.motors_pos_cur[i];
        motor_vel[i]=busIn.motors_vel_cur[i];
    }
    motor_pos_des=busIn.motors_pos_des;
    motor_vel_des=busIn.motors_vel_des;
    motor_tor_des=busIn.motors_tor_des;
}

void PVT_Ctr::getFeedbackMotorState (RobotWrapper &robot_wrapper)
{
    // robot_wrapper.q = [base_pos(3), base_quat(4), joint_pos(jointNum)], so
    // joint positions start at index 7 (see RobotWrapper::updateRobotState).
    // motor_pos_cur is std::vector<double>, so Eigen's block expression needs
    // to be copied element-by-element via Eigen::Map, not assigned directly.
    Eigen::VectorXd::Map(motor_pos_cur.data(), motor_pos_cur.size()) = robot_wrapper.q.segment(7, jointNum);

    // robot_wrapper.dq = [base_lin_vel(3), base_ang_vel(3), joint_vel(jointNum)],
    // so joint velocities start at index 6 here (no quaternion in dq).
    Eigen::VectorXd::Map(motor_vel.data(), motor_vel.size()) = robot_wrapper.dq.segment(6, jointNum);
}

void PVT_Ctr::dataBusWrite(DataBus &busIn) {
    busIn.motors_tor_out=motor_tor_out_motor;
    busIn.motors_tor_cur=motor_tor_out_link;
}

void PVT_Ctr::setJointPD(double kp, double kd, const char *jointName) {
    auto it = std::find(motorName.begin(), motorName.end(), jointName);

    int id=-1;
    if (it != motorName.end()) {
        id = std::distance(motorName.begin(), it);
    } else {
        std::cout << jointName << " NOT found!" << std::endl;
    }
    pvt_Kp[id]=kp;
    pvt_Kd[id]=kd;
}

// joint pvt control
void PVT_Ctr::calMotorsPVT() {
    for (int i=0;i<jointNum;i++)
    {
        double tauDes{0};
        tauDes=PV_enable[i]*pvt_Kp[i]*(motor_pos_des[i]-motor_pos_cur[i])+PV_enable[i]*pvt_Kd[i]*(motor_vel_des[i]-motor_vel[i]);
        tauDes=tau_out_lpf[i].ftOut(tauDes)+motor_tor_des[i];
        if (fabs(tauDes)>=fabs(maxTor[i]))
            tauDes= sign(tauDes)*maxTor[i];
        motor_tor_out_motor[i]=tauDes/gear[i];
        motor_tor_out_link[i]=tauDes;
        motor_pos_des_old[i]=motor_pos_des[i];
    }
}

// joint pvt control with delta position limit
void PVT_Ctr::calMotorsPVT(double deltaP_Lim) {
    for (int i=0;i<jointNum;i++)
    {
        double tauDes{0};
        double delta=motor_pos_des[i]-motor_pos_des_old[i];
        if (fabs(delta)>= fabs(deltaP_Lim))
            delta=deltaP_Lim * sign(delta);
        double pDes=delta+motor_pos_des_old[i];
        tauDes=PV_enable[i]*pvt_Kp[i]*(pDes-motor_pos_cur[i])+PV_enable[i]*pvt_Kd[i]*(motor_vel_des[i]-motor_vel[i]);
        tauDes=tau_out_lpf[i].ftOut(tauDes)+motor_tor_des[i];
        if (fabs(tauDes)>=fabs(maxTor[i]))
            tauDes= sign(tauDes)*maxTor[i];
        motor_tor_out_motor[i]=tauDes/gear[i];
        motor_tor_out_link[i]=tauDes;
        motor_pos_des_old[i]=pDes;
    }
}

// joint PD impedance control from explicit reference vectors (bypasses
// motor_pos_des/motor_vel_des/motor_tor_des, unlike the other overloads)
void PVT_Ctr::calMotorsPVT (VectorXd ref_pos, VectorXd ref_vel, VectorXd tau_ff)
{
    for (int i=0;i<jointNum;i++)
    {
        double tauDes = PV_enable[i]*pvt_Kp[i]*(ref_pos(i)-motor_pos_cur[i])
                       + PV_enable[i]*pvt_Kd[i]*(ref_vel(i)-motor_vel[i])
                       + tau_ff(i);
        if (fabs(tauDes)>=fabs(maxTor[i]))
            tauDes = sign(tauDes)*maxTor[i];
        motor_tor_out_motor[i]=tauDes/gear[i];
        motor_tor_out_link[i]=tauDes;
    }
}

double PVT_Ctr::sign(double in) {
    if (in>=0)
        return 1.0;
    else
        return -1.0;
}

// Enable PV control item for all joints. Note the PV control item is default enabled, no need for calling this function
void PVT_Ctr::enablePV() {
    PV_enable.assign(jointNum,1);
}

void PVT_Ctr::enablePV(int jtId) {
    PV_enable[jtId]=1;
}

// Disable PV control item for all joints.
void PVT_Ctr::disablePV() {
    PV_enable.assign(jointNum,0);
}

void PVT_Ctr::disablePV(int jtId) {
    PV_enable[jtId]=0;
}

void PVT_Ctr::genTestTrajectory(double t){
    const double start_time = 2; // starting time of the trajectory
    const double sim_duration = 10; // duration of the trajectory
    const double ramp_duration = 2.0; // time to ramp from 0 up to the joint center
    const double omega = 2 * 3.14159265358979 / 4.0; // angular frequency, 2-second period
    double traj_time{0};
    this->motor_tor_des.assign(this->jointNum, 0); // Feedforward torque

    if (t <= start_time) // before starting, just stay at 0
    {
        this->motor_pos_des.assign(this->jointNum, 0);
        this->motor_vel_des.assign(this->jointNum, 0);
        for (int i = 0; i<this->jointNum; i++)
        {
            traj_pos_lpf[i].ftOut(0); // keep the filters primed at 0 while idle
            traj_vel_lpf[i].ftOut(0);
        }
    }
    else
    {
        traj_time = t - start_time;

        for (int i = 0; i<this->jointNum; i++)
        {
            double joint_Amplitude = std::max(0.0, 0.9 * (this->maxPos[i] - this->minPos[i]) / 2.0);
            double joint_Center = (this->maxPos[i] + this->minPos[i]) / 2.0;

            double raw_pos_des, raw_vel_des;
            if (traj_time < ramp_duration)
            {
                // Ramp linearly from 0 up to the joint center over ramp_duration,
                // instead of jumping straight into an off-center oscillation
                raw_pos_des = joint_Center * (traj_time / ramp_duration);
                raw_vel_des = joint_Center / ramp_duration;
            }
            else
            {
                // Generate a sinusoidal trajectory for each joint
                // amplitude is taken at 0.9 of the joint motion range
                double osc_time = traj_time - ramp_duration;
                raw_pos_des = joint_Center + joint_Amplitude * sin(omega * osc_time);
                raw_vel_des = joint_Amplitude * omega * cos(omega * osc_time);
            }

            // Low-pass filter the raw ramp/sinusoid reference so the switch between
            // the two phases (and the trajectory's start) doesn't show up as a kink
            this->motor_pos_des[i] = traj_pos_lpf[i].ftOut(raw_pos_des);
            this->motor_vel_des[i] = traj_vel_lpf[i].ftOut(raw_vel_des);
        }
    }
}

//------------------------------
// Printing function
//------------------------------
void PVT_Ctr::printPVTinfo()
{
    std::printf("Number of joint: %d\n", this->jointNum);
    std::printf("List of joints:\n");

    for (int i = 0; i < jointNum; i++)
    {
        std::printf("  [%2d] %-25s  kp=%.3f  kd=%.3f  maxTor=%.3f  maxVel=%.3f  minPos=%.5f  maxPos=%.5f  gear=%.3f\n",
                     i, motorName[i].c_str(), pvt_Kp[i], pvt_Kd[i], maxTor[i], maxVel[i], minPos[i], maxPos[i], gear[i]);
    }
}

void PVT_Ctr::printTorqueOut()
{
    for (int i = 0; i < jointNum; i++)
    {
        std::printf("Torqued at Joint %-25s is: %.4f\n", this->motorName[i].c_str(), this->motor_tor_out_motor[i]);
    }
}
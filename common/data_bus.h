/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024-2025 Humanoid Robot (Shanghai) Co., Ltd.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#pragma once

#include "Eigen/Dense"
#include <iostream>
#include <vector>
#include "iomanip"

struct DataBus
{
    const int model_nv; // number of dq

    // const values for frame mismatch
    const Eigen::Matrix3d fe_L_rot_L_off = (Eigen::MatrixXd(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1).finished(); // left foot-end R w.r.t to the body frame in offset posture (currently identity/no-op, unused in WBC math)
    const Eigen::Matrix3d fe_R_rot_L_off = (Eigen::MatrixXd(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1).finished(); // right foot-end frame-offset rotation, same as above (identity/no-op)

    // joint, sensors and states feedback
    double rpy[3];      // base-link roll/pitch/yaw (rad), from IMU quaternion, world frame
    double fL[3];       // left foot-end contact force (N); currently unpopulated stub, always 0
    double fR[3];       // right foot-end contact force (N); currently unpopulated stub, always 0
    double basePos[3];  // base-link position, world frame (m)
    double baseLinVel[3]; // velocity of the basePos
    double baseAcc[3];    // baseAcc of the base link
    double baseAngVel[3]; // angular velocity of the base link
    std::vector<double> joint_pos_cur; // measured joint positions (rad), size model_nv-6
    std::vector<double> joint_vel_cur; // measured joint velocities (rad/s)
    std::vector<double> joint_tor_cur; // PVT-controller commanded joint torque (N.m), used as applied-torque proxy
    Eigen::VectorXd FL_est, FR_est;     // estimated left/right foot-end 6D wrench (force+moment)
    bool isdqIni;                       // vestigial "dq initialized" flag; not read/written elsewhere

    // PVT controls
    std::vector<double> joint_pos_des; // desired joint position setpoint (rad) for PD control
    std::vector<double> joint_vel_des; // desired joint velocity setpoint (rad/s) for PD control
    std::vector<double> joint_tor_des; // desired feed-forward joint torque (N.m) added to PD term
    std::vector<double> joint_tor_out; // final motor-side torque command sent to actuators

    // states and key variables
    Eigen::VectorXd q, dq, ddq; // q=[base_pos(3),base_quat(4),joint_pos]; dq/ddq=[base_lin_vel(3),base_ang_vel(3),joint_vel/acc]
    Eigen::VectorXd qOld;       // q from the previous timestep, cached in updateQ()
    Eigen::MatrixXd J_base, J_l, J_r, J_hd_l, J_hd_r, J_hip_link; // 6xN Jacobians of base, left/right foot-end, left/right hand, hip link
    Eigen::MatrixXd dJ_base, dJ_l, dJ_r, dJ_hd_l, dJ_hd_r;        // time-derivatives of the Jacobians above
    Eigen::MatrixXd Jcom_W; // jacobian of CoM, in world frame
    Eigen::Vector3d pCoM_W; // center-of-mass position, world frame (m)
    Eigen::Vector3d fe_r_pos_W, fe_l_pos_W, base_pos, base_vel; // right/left foot-end pos, base pos, base linear vel (world frame)
    Eigen::Matrix3d fe_r_rot_W, fe_l_rot_W, base_rot; // in world frame
    Eigen::Vector3d fe_r_pos_L, fe_l_pos_L;           // in Body frame
    Eigen::Vector3d fe_r_vel_L, fe_l_vel_L;           // linear velocity in Body frame
    Eigen::Vector3d hip_link_pos;                     // waist/pelvis link position, world frame (m)
    Eigen::Vector3d hip_r_pos_L, hip_l_pos_L;         // right/left hip-joint position, body frame (m)
    Eigen::Vector3d hip_r_pos_W, hip_l_pos_W;         // right/left hip-joint position, world frame (m)
    Eigen::Matrix3d fe_r_rot_L, fe_l_rot_L;           // right/left foot-end rotation, body frame
    Eigen::Matrix3d hip_link_rot;                     // waist/pelvis link rotation, world frame
    Eigen::Vector3d fe_r_pos_L_cmd, fe_l_pos_L_cmd;   // reserved, unused: foot-end position command, body frame
    Eigen::Matrix3d fe_r_rot_L_cmd, fe_l_rot_L_cmd;   // reserved, unused: foot-end rotation command, body frame

    Eigen::Vector3d hd_r_pos_W, hd_l_pos_W; // in world frame
    Eigen::Matrix3d hd_r_rot_W, hd_l_rot_W; // right/left hand rotation, world frame
    Eigen::Vector3d hd_r_pos_L, hd_l_pos_L; // in body frame
    Eigen::Matrix3d hd_r_rot_L, hd_l_rot_L; // right/left hand rotation, body frame
    Eigen::VectorXd qCmd, dqCmd;            // reserved, unused: commanded q/dq
    Eigen::VectorXd tauJointCmd;            // reserved, unused: commanded joint torque
    Eigen::MatrixXd dyn_M, dyn_M_inv, dyn_C, dyn_Ag, dyn_dAg; // mass/inertia matrix (+inverse), Coriolis matrix, centroidal momentum matrix (+derivative)
    Eigen::VectorXd dyn_G, dyn_Non;         // gravity generalized force; nonlinear bias term (C*dq + G)
    Eigen::Vector3d base_omega_L, base_omega_W, base_rpy; // base angular velocity in body/world frame, base roll-pitch-yaw

    Eigen::Vector3d slop;                   // ground slope (roll,pitch,yaw) used by MPC for stance-foot orientation on uneven terrain
    Eigen::Matrix<double, 3, 3> inertia;    // centroidal/base rotational inertia tensor, body frame

    // state EST
    Eigen::Matrix<double, 3, 1> base_pos_est, base_vel_est; // EKF-estimated base/CoM position and linear velocity, world frame
    Eigen::Matrix<double, 3, 1> eul_est, omegaW_est;        // EKF-filtered roll/pitch/yaw (no yaw offset) and angular velocity, world frame
    Eigen::Matrix<double, 3, 1> fe_l_pos_W_est, fe_r_pos_W_est; // EKF-estimated left/right foot-end position, world frame
    Eigen::Matrix<double, 3, 1> delta_acc; // EKF-estimated accelerometer bias/correction term
    Eigen::Matrix<double, 3, 1> freeAcc;   // IMU acceleration with gravity removed, EKF process-model input

    Eigen::Matrix<double, 15, 1> AX, BU; // EKF state vector X=[pCoM,vCoM,pFootL,pFootR,accBias]; AX=predicted A*X, BU=process input B*freeAcc
    Eigen::Matrix<double, 14, 1> CX;     // EKF predicted measurement C*X
    Eigen::Matrix<double, 14, 1> Y;      // EKF actual measurement vector (relative foot pos/vel/height), for innovation Y-C*X
    Eigen::Matrix<double, 6, 1> pbW;     // left/right foot-end position relative to base, rotated into world-aligned frame

    // cmd value from the joystick interpreter
    Eigen::Vector3d js_eul_des;   // desired base roll/pitch/yaw, world frame (rad)
    Eigen::Vector3d js_pos_des;   // desired base position, world frame (m)
    Eigen::Vector3d js_omega_des; // desired base angular velocity (yaw-rate populated)
    Eigen::Vector3d js_vel_des;   // desired base linear velocity, world frame (m/s)

    // cmd values for MPC
    Eigen::VectorXd Xd;    // desired centroidal state trajectory over the horizon, 12*10, built from js_*_des
    Eigen::VectorXd X_cur; // current 12-dim centroidal state [base_rpy, base_pos, base_ang_vel_W, base_lin_vel_W]
    //    Eigen::Vector3d     mpc_eul_des;
    //    Eigen::Vector3d     mpc_pos_des;
    //    Eigen::Vector3d     mpc_omega_des;
    //    Eigen::Vector3d     mpc_vel_des;
    Eigen::VectorXd X_cal;  // single-step-ahead predicted centroidal state from MPC solution
    Eigen::VectorXd dX_cal; // predicted centroidal state derivative from MPC solution, seeds WBC des_ddq
    Eigen::VectorXd fe_react_tau_cmd; // MPC-optimized foot reaction wrench decision vector, 13*3

    int qp_nWSR_MPC;         // number of qpOASES working-set recalculations for MPC solve
    double qp_cpuTime_MPC;   // CPU time (s) of MPC QP solve
    int qpStatus_MPC;        // MPC QP solver return status (0 = success)

    // cmd values for WBC
    Eigen::Vector3d base_rpy_des;   // desired base roll/pitch/yaw for WBC tracking task (rad)
    Eigen::Vector3d base_pos_des;   // desired base position for WBC tracking task, world frame (m)
    Eigen::Vector3d base_vel_des;   // desired base linear velocity, world frame (m/s)
    Eigen::Vector3d base_omega_des; // desired base angular velocity (rad/s)
    Eigen::VectorXd des_ddq, des_dq, des_delta_q, des_q; // desired generalized accel/vel/delta-pos feed-forward references for WBC (des_q unused)
    Eigen::Vector3d swing_fe_pos_des_W; // desired swing-foot-end position, world frame (m)
    Eigen::Vector3d swing_fe_rpy_des_W; // desired swing-foot-end orientation, world frame
    Eigen::Vector3d stance_fe_pos_cur_W; // current stance-foot-end position, world frame (m)
    Eigen::Matrix3d stance_fe_rot_cur_W; // current stance-foot-end rotation, world frame
    Eigen::VectorXd wbc_delta_q_final, wbc_dq_final, wbc_ddq_final; // WBC kinematic-task solution: incremental joint pos/vel/accel
    Eigen::VectorXd wbc_tauJointRes; // final joint torque command from WBC QP, size model_nv-6
    Eigen::VectorXd wbc_FrRes;       // WBC QP-optimized foot ground-reaction force/torque correction, 12x1
    Eigen::VectorXd Fr_ff;           // feed-forward foot reaction force/torque bias for WBC QP, from MPC
    int qp_nWSR;        // number of qpOASES working-set recalculations for WBC torque QP
    double qp_cpuTime;  // CPU time (s) of WBC torque QP solve
    int qp_status;      // WBC QP solver return status (0 = success)

    // values for foot-placement
    Eigen::Vector3d swingStartPos_W;   // swing-foot-end position at start of current swing phase, world frame
    Eigen::Vector3d swingDesPosCur_W;  // current-instant desired swing-foot-end position (cycloid trajectory), world frame
    Eigen::Vector3d swingDesPosCur_L;  // reserved, unused: body-frame counterpart of swingDesPosCur_W
    Eigen::Vector3d swingDesPosFinal_W; // predicted final swing-foot landing position for this step, world frame
    Eigen::Vector3d stanceDesPos_W;    // stance-foot-end position captured at start of current stance phase, world frame
    Eigen::Vector3d posHip_W, posST_W; // hip position opposite the stance leg (placement ref); current stance-foot-end position, world frame
    Eigen::Vector3d desV_W; // desired linear velocity
    double desWz_W;         // desired angular velocity
    double theta0;          // offset yaw angle of the swing leg, w.r.t body frame
    double width_hips;      // distance between the left and right hip
    double tSwing;          // swing-phase duration (s)
    double phi;             // gait phase variable in [0,1] for current swing/stance cycle
    enum MotionState
    {
        Stand,      // standing/balance mode
        Walk,       // continuous walking mode
        Walk2Stand  // transition from walking to standing
    };
    enum LegState
    {
        LSt, // left leg is stance/support leg
        RSt, // right leg is stance/support leg
        DSt // no use but reserverd
    };
    bool leg_contact[2];             // foot contact indicators: [0]=left, [1]=right
    double thetaZ_des{0};            // reserved, unused: desired yaw offset
    LegState legState{DataBus::DSt}; // current support-leg state
    LegState legStateNext{DataBus::DSt}; // predicted support-leg state for next step, used to schedule MPC contacts
    MotionState motionState{DataBus::Stand}; // high-level FSM behavior mode

    // for jump
    Eigen::Vector3d base_pos_stand;             // base position snapshot during pre-jump standing posture, world frame
    Eigen::Matrix<double, 6, 1> pfeW_stand, pfeW0; // left+right foot-end positions (world frame) captured at pre-jump stand / at each jump-phase transition
    // Eigen::Vector3d mpc_eul_des, mpc_omega_des, mpc_vel_des, mpc_pos_des;

    DataBus(int model_nvIn) : model_nv(model_nvIn)
    {
        joint_pos_cur.assign(model_nv - 6, 0);
        joint_vel_cur.assign(model_nv - 6, 0);
        joint_tor_out.assign(model_nv - 6, 0);
        joint_tor_cur.assign(model_nv - 6, 0);
        joint_tor_des.assign(model_nv - 6, 0);
        joint_vel_des.assign(model_nv - 6, 0);
        joint_pos_des.assign(model_nv - 6, 0);
        q = Eigen::VectorXd::Zero(model_nv + 1);
        qOld = Eigen::VectorXd::Zero(model_nv + 1);
        dq = Eigen::VectorXd::Zero(model_nv);
        ddq = Eigen::VectorXd::Zero(model_nv);
        qCmd = Eigen::VectorXd::Zero(model_nv + 1);
        dqCmd = Eigen::VectorXd::Zero(model_nv);
        tauJointCmd = Eigen::VectorXd::Zero(model_nv - 6);
        FL_est = Eigen::VectorXd::Zero(6);
        FR_est = Eigen::VectorXd::Zero(6);
        Xd = Eigen::VectorXd::Zero(12 * 10);
        X_cur = Eigen::VectorXd::Zero(12);
        X_cal = Eigen::VectorXd::Zero(12);
        dX_cal = Eigen::VectorXd::Zero(12);
        fe_react_tau_cmd = Eigen::VectorXd::Zero(13 * 3);
        Fr_ff = Eigen::VectorXd::Zero(12);
        des_ddq = Eigen::VectorXd::Zero(model_nv);
        des_dq = Eigen::VectorXd::Zero(model_nv);
        des_delta_q = Eigen::VectorXd::Zero(model_nv);
        base_rpy_des.setZero();
        base_pos_des.setZero();
        base_vel_des.setZero();
        base_omega_des.setZero();
        js_eul_des.setZero();
        js_pos_des.setZero();
        js_omega_des.setZero();
        js_vel_des.setZero();
        motionState = Stand;
        base_vel << 0, 0, 0;
    };

    // update q according to sensor values, must update sensor values before
    void updateQ()
    {
        base_omega_W << baseAngVel[0], baseAngVel[1], baseAngVel[2];
        auto Rcur = eul2Rot(rpy[0], rpy[1], rpy[2]);
        base_omega_W = Rcur * base_omega_W;

        //  q = [global_base_position, global_base_quaternion, joint_positions]
        //  dq = [global_base_velocity_linear, global_base_velocity_angular, joint_velocities]

        auto quatNow = eul2quat(rpy[0], rpy[1], rpy[2]);
        q(0) = basePos[0];
        q(1) = basePos[1];
        q(2) = basePos[2];
        q(3) = quatNow.x();
        q(4) = quatNow.y();
        q(5) = quatNow.z();
        q(6) = quatNow.w();
        for (int i = 0; i < model_nv - 6; i++)
            q(i + 7) = joint_pos_cur[i];

        Eigen::Vector3d vCoM_W;
        vCoM_W << baseLinVel[0], baseLinVel[1], baseLinVel[2];
        dq.block<3, 1>(0, 0) = vCoM_W;
        dq.block<3, 1>(3, 0) << base_omega_W[0], base_omega_W[1], base_omega_W[2];
        //        dq.block<3,1>(3,0) << baseAngVel[0],baseAngVel[1],baseAngVel[2];
        for (int i = 0; i < model_nv - 6; i++)
        {
            dq(i + 6) = joint_vel_cur[i];
        }

        base_pos << q(0), q(1), q(2);
        base_rpy << rpy[0], rpy[1], rpy[2];
        base_rot = Rcur;
        qOld = q;
    }

    static void printdq(const Eigen::VectorXd &q)
    {
        std::cout << std::setprecision(5) << q.block<6, 1>(0, 0).transpose() << std::endl;
        std::cout << std::setprecision(5) << q.block<7, 1>(6, 0).transpose() << std::endl;
        std::cout << std::setprecision(5) << q.block<7, 1>(13, 0).transpose() << std::endl;
        std::cout << std::setprecision(5) << q.block<4, 1>(20, 0).transpose() << std::endl;
        std::cout << std::setprecision(5) << q.block<6, 1>(24, 0).transpose() << std::endl;
        std::cout << std::setprecision(5) << q.block<6, 1>(30, 0).transpose() << std::endl;
    }

    static void printq(const Eigen::VectorXd &q)
    {
        std::cout << std::setprecision(5) << q.block<7, 1>(0, 0).transpose() << std::endl;
        std::cout << std::setprecision(5) << q.block<7, 1>(7, 0).transpose() << std::endl;
        std::cout << std::setprecision(5) << q.block<7, 1>(14, 0).transpose() << std::endl;
        std::cout << std::setprecision(5) << q.block<4, 1>(21, 0).transpose() << std::endl;
        std::cout << std::setprecision(5) << q.block<6, 1>(25, 0).transpose() << std::endl;
        std::cout << std::setprecision(5) << q.block<6, 1>(31, 0).transpose() << std::endl;
    }

    Eigen::Matrix<double, 3, 3> eul2Rot(double roll, double pitch, double yaw)
    {
        Eigen::Matrix<double, 3, 3> Rx, Ry, Rz;
        Rz << cos(yaw), -sin(yaw), 0,
            sin(yaw), cos(yaw), 0,
            0, 0, 1;
        Ry << cos(pitch), 0, sin(pitch),
            0, 1, 0,
            -sin(pitch), 0, cos(pitch);
        Rx << 1, 0, 0,
            0, cos(roll), -sin(roll),
            0, sin(roll), cos(roll);
        return Rz * Ry * Rx;
    }

    Eigen::Quaterniond eul2quat(double roll, double pitch, double yaw)
    {
        Eigen::Matrix3d R = eul2Rot(roll, pitch, yaw);
        Eigen::Quaternion<double> quatCur;
        quatCur = R; // rotation matrix converted to quaternion
        Eigen::Quaterniond resQuat;
        resQuat = quatCur;
        return resQuat;
    }
};

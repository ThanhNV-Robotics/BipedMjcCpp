/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024-2025 Humanoid Robot (Shanghai) Co., Ltd.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#include "foot_placement.h"
#include "bezier_1D.h"
#include "data_type.h"
#include <cmath>
#include <yaml-cpp/yaml.h>

FootPlacement::FootPlacement(const std::string &yamlPath)
{
    YAML::Node root = YAML::LoadFile(yamlPath);
    const YAML::Node &fp = root["foot_placement"];
    stepHeight = fp["stepHeight"].as<double>();
    hip_width  = fp["hip_width"].as<double>();
    xOff_L     = fp["x_offset"].as<double>();
    yOff_L     = fp["y_offset"].as<double>();
    zOff_W     = fp["z_offset"].as<double>();
}

void FootPlacement::updateFromRobot(const RobotWrapper &rb_wrapper, const MyGaitScheduler &gait_scheduler, const JoyStickInterpreter &joyStick)
{
    LegState curLegState = gait_scheduler.legState;

    // Snapshot the swing foot's liftoff position the instant the swing leg
    // changes (stance-leg transition) -- getSwingPos()'s cycloid/Trajectory
    // blends start from here.
    Eigen::Vector3d curSwingFootPos_W = (curLegState == LegState::LSt) ? rb_wrapper.pos_R_feet_W : rb_wrapper.pos_L_feet_W;
    if (!swingInit_ || curLegState != swingLegPrev_)
    {
        posStart_W = curSwingFootPos_W;
        swingInit_ = true;
    }
    swingLegPrev_ = curLegState;
    legState = curLegState;

    phi = gait_scheduler.phi;
    tSwing = gait_scheduler.tSwing;
    // baseline swing-arc angle for the current stance side -- mirrors
    // MyGaitScheduler::step()'s own (private) theta0 computation.
    theta0 = (curLegState == LegState::LSt) ? -M_PI / 2.0 : M_PI / 2.0;

    base_pos = rb_wrapper.pos_base_W;

    // yaw + world-frame yaw rate, extracted from the base's orientation/
    // velocity state (Pinocchio's q.segment<4>(3) coeffs order is x,y,z,w;
    // dq's angular part is base-local, rotated to world here same as
    // KinWBC::updateCurrent() does for task_base_rpy).
    Eigen::Quaterniond quat_base_W(rb_wrapper.q(6), rb_wrapper.q(3), rb_wrapper.q(4), rb_wrapper.q(5));
    Eigen::Matrix3d Rcur_base = quat_base_W.toRotationMatrix();
    yawCur = std::atan2(Rcur_base(1, 0), Rcur_base(0, 0));
    omegaZ_W = (Rcur_base * rb_wrapper.dq.segment<3>(3))(2);

    curV_W = rb_wrapper.vel_base_W;
    desV_W = Eigen::Vector3d(joyStick.vx_W, joyStick.vy_W, 0.0);
    desWz_W = joyStick.wz_L; // pure z rotation -- body-frame and world-frame rates coincide

    // hipPos_W: RobotWrapper doesn't expose hip frames directly, so
    // approximate the swinging leg's hip as the base position offset
    // laterally by half the hip width (no fore/aft or vertical offset).
    // Swap in an exact hip Jacobian/frame here if RobotWrapper ever adds one.
    Eigen::Matrix3d Rz;
    Rz << std::cos(yawCur), -std::sin(yawCur), 0,
        std::sin(yawCur), std::cos(yawCur), 0,
        0, 0, 1;
    double side = (curLegState == LegState::LSt) ? -1.0 : 1.0; // LSt (left stance) -> swing leg is right -> right hip sits on the -y side
    hipPos_W = base_pos + Rz * Eigen::Vector3d(0, side * hip_width / 2.0, 0);

    if (gait_scheduler.motionState == MotionState::WARM_UP)
    {
        this->inPlaceOnly = true;
    }
    else if (gait_scheduler.motionState == MotionState::WALK) {
        this->inPlaceOnly = false;
    }
}

void FootPlacement::StepSwingPlanning(const RobotWrapper &rb_wrapper,
                                        const MyGaitScheduler &gait_scheduler,
                                         const JoyStickInterpreter &joyStick,
                                        const CP_Planning &cp_planner)
{
    updateFromRobot(rb_wrapper, gait_scheduler, joyStick);


    // Eigen::Matrix<double, 4, 1> b;
    // b.setZero();
    // Eigen::Matrix<double, 1, 4> xNow;
    // xNow << 1, phi, pow(phi, 2), pow(phi, 3);

    // Eigen::Matrix3d KP, Rz;
    // KP.setZero();
    // KP(0, 0) = kp_vx;
    // KP(1, 1) = kp_vy;
    // KP(2, 2) = 0;
    // Rz << cos(yawCur), -sin(yawCur), 0,
    //     sin(yawCur), cos(yawCur), 0,
    //     0, 0, 1;
    // KP = Rz * KP * Rz.transpose();

    // Standard Raibert heuristic: p_foot = p_hip + 0.5*T*v_des + Kp*(v_cur - v_des)
    // hipPos_W is re-evaluated every tick from the current base position, so
    // hip drift during the remaining swing is already captured — the extra
    // curV_W*(1-phi)*tSwing look-ahead that was here caused double-counting
    // and made the foot target 3× larger than the CP step advance.

    // posDes_W = hipPos_W + KP * (curV_W - desV_W) + 0.5 * tSwing * desV_W;

    double step_length = cp_planner.step_length;

    // posDes_W[0] = hipPos_W[0] + cp_planner.cxi_x_;
    // posDes_W[1] = hipPos_W[1] + cp_planner.cxi_y_;

    posDes_W[0] = cp_planner.cxi_x_;
    posDes_W[1] = hipPos_W[1] ;

    // // for angular veloctity
    // double thetaF;
    // thetaF = yawCur + theta0 + omegaZ_W * (1 - phi) * tSwing + 0.5 * omegaZ_W * tSwing + kp_wz * (omegaZ_W - desWz_W);
    // posDes_W(0) += 0.5 * hip_width * (cos(thetaF) - cos(yawCur + theta0));
    // posDes_W(1) += 0.5 * hip_width * (sin(thetaF) - sin(yawCur + theta0));

    // foot-end placement offsets (loaded from YAML via constructor)

    posDes_W(2) = base_pos(2) - legLength + zOff_W;

    double xOff_W(0), yOff_W(0);
    if (legState == LegState::LSt)
    {
        xOff_W = cos(yawCur) * xOff_L - sin(yawCur) * yOff_L;
        yOff_W = sin(yawCur) * xOff_L + cos(yawCur) * yOff_L;
        posDes_W[1] = - this->hip_width/2;
        // yOff_W = 0.05;
    }
    else if (legState == LegState::RSt)
    {
        xOff_W = cos(yawCur) * xOff_L - sin(yawCur) * (-yOff_L);
        yOff_W = sin(yawCur) * xOff_L + cos(yawCur) * (-yOff_L);
        posDes_W[1] = this->hip_width/2;
        // yOff_W = -0.05;
    }

    // posDes_W(0) += xOff_W;
    // posDes_W(1) += yOff_W;

    //
    //    double yOff=0.005; // positive for moving the leg inside
    //    if (legState==LegState::LSt)
    //        posDes_W(1)+=yOff;
    //    else if (legState==LegState::RSt)
    //        posDes_W(1)-=yOff;

    // cycloid trajectories -- unless inPlaceOnly, which pins x,y at the
    // liftoff position instead (lift/lower straight up and down, no
    // forward stepping).
    if (inPlaceOnly)
    {
        pDesCur[0] = posStart_W(0);
        pDesCur[1] = posStart_W(1);
    }
    else if (phi <= 1.0)
    {
        
        pDesCur[0] = posStart_W(0) + (posDes_W(0) - posStart_W(0)) / (2 * 3.1415) * (2 * 3.1415 * phi - sin(2 * 3.1415 * phi));
        pDesCur[1] = posStart_W(1) + (posDes_W(1) - posStart_W(1)) / (2 * 3.1415) * (2 * 3.1415 * phi - sin(2 * 3.1415 * phi));
    }

    // if (phi >= 1)
    // {
    //     zStretch += -0.002;

    //     // std::cout << "---------------- " << zStretch << std::endl;
    // }
    // else
    //     zStretch = 0;
    // if (zStretch < -0.05)
    // {
    //     zStretch = -0.05;
    //     // finish_Stretch = true;
    // }

    // if (phi < 1e-3)
    // {
    //     finish_Stretch = false;
    // }

    pDesCur[2] = posStart_W(2) + stepHeight * 0.5 * (1 - cos(2 * 3.1415 * phi)) + (posDes_W(2) - posStart_W(2)) / (2 * 3.1415) * (2 * 3.1415 * phi - sin(2 * 3.1415 * phi))+ zStretch;
    // double zBeforeStretch = posStart_W(2) + Trajectory(0.2, stepHeight, posDes_W(2) - posStart_W(2));
    // Never plan the swing foot below ground -- clamp zStretch itself
    // (rather than the final sum) so it can't dig past zBeforeStretch's own
    // floor. Clamping the sum post-hoc instead would flatten pDesCur[2] at 0
    // while zStretch keeps accumulating underneath unseen, then jump when
    // the next cycle's reset suddenly exposes that accumulated offset --
    // clamping zStretch here keeps it (and so the whole curve) continuous.
    // zStretch = std::max(zStretch, -zBeforeStretch);
    // pDesCur[2] = zBeforeStretch + zStretch;
}

double FootPlacement::Trajectory(double phase, double hei, double len)
{
    Bezier_1D Bswpid;
    double para0 = 5, para1 = 3;
    for (int i = 0; i < para0; i++)
    {
        Bswpid.P.push_back(0.0);
    }
    for (int i = 0; i < para1; i++)
    {
        Bswpid.P.push_back(1.0);
    }

    double output;
    if (phi < phase)
    {
        output = hei * Bswpid.getOut(phi / phase);
    }
    else
    {
        double s = Bswpid.getOut((1.4 - phi) / (1.4 - phase));
        if (s > 0)
        {
            output = hei * s + len * (1.0 - s);
        }
        else
        {
            output = len;
        }
    }
    return output;
}

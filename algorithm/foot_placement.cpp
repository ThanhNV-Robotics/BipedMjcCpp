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
#include <algorithm>
#include <cmath>
#include <yaml-cpp/yaml.h>

FootPlacement::FootPlacement(const std::string &yamlPath, RobotWrapper &robot_wrapper)
{
    YAML::Node root = YAML::LoadFile(yamlPath);
    const YAML::Node &fp = root["foot_placement"];
    stepHeight = fp["stepHeight"].as<double>();
    hip_width  = robot_wrapper.hip_width_;
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
    // Keyed on RSt (not LSt) so DSt falls into the same "right swinging"
    // bucket as LSt -- must match KinWBC.cpp's task_lift_foot/static_contact
    // assignment (if (legState==RSt) {left...} else {right...}), which is
    // ALSO keyed on RSt. With plain LSt/RSt cycling this made no observable
    // difference (DSt never persisted), but the newer leg_state_ cycle
    // (DSt->LSt->RSt->DSt->...) holds DSt for a full t_swing each time --
    // keying this on LSt instead disagreed with KinWBC for that entire
    // phase (FootPlacement computing a target for the left foot while
    // KinWBC applied it to the right foot), producing a persistent
    // full-hip-width error every cycle.
    Eigen::Vector3d curSwingFootPos_W = (curLegState == LegState::RSt) ? rb_wrapper.pos_L_feet_W : rb_wrapper.pos_R_feet_W;
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
    theta0 = (curLegState == LegState::RSt) ? M_PI / 2.0 : -M_PI / 2.0;

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
    double side = (curLegState == LegState::RSt) ? 1.0 : -1.0; // keyed on RSt to agree with KinWBC.cpp on DSt -- see curSwingFootPos_W's comment above
    hipPos_W = base_pos + Rz * Eigen::Vector3d(0, side * hip_width / 2.0, 0);

    if (gait_scheduler.motionState == MotionState::WARM_UP)
    {
        this->inPlaceOnly = true;
    }
    else if (gait_scheduler.motionState == MotionState::WALK) {
        this->inPlaceOnly = false;
    }
}

void FootPlacement::updateFromRobot(const RobotWrapper &rb_wrapper, const JoyStickInterpreter &joyStick, const CP_Planning &cp_planner)
{
    // Keyed on leg_state_swing_ (leg_state_ delayed by one full cycle), not
    // leg_state_ -- see CP_Planning.h's comment on leg_state_swing_. This is
    // what makes the foot lift only once the ZMP has actually arrived at
    // last cycle's target, instead of lifting while the CoM is still mid
    // weight-shift toward it.
    LegState curLegState = cp_planner.leg_state_swing_;

    // Snapshot the swing foot's liftoff position the instant the swing leg
    // changes (stance-leg transition) -- getSwingPos()'s cycloid/Trajectory
    // blends start from here.
    // Keyed on RSt (not LSt) -- see the other updateFromRobot() overload's
    // comment above; must agree with KinWBC.cpp's RSt-keyed task assignment
    // so DSt (which cp_planner.leg_state_swing_ holds for the very first
    // cycle, before any real target has been reached) is treated as "right
    // swinging" by both -- though StepSwingPlanning() additionally pins the
    // foot in place (no lift at all) while curLegState==DSt, since there's
    // no real one-cycle-old target to swing toward yet.
    Eigen::Vector3d curSwingFootPos_W = (curLegState == LegState::RSt) ? rb_wrapper.pos_L_feet_W : rb_wrapper.pos_R_feet_W;
    if (!swingInit_ || curLegState != swingLegPrev_)
    {
        posStart_W = curSwingFootPos_W;
        swingInit_ = true;
    }
    swingLegPrev_ = curLegState;
    legState = curLegState;

    phi = cp_planner.phi_swing;
    tSwing = cp_planner.t_swing;
    // baseline swing-arc angle for the current stance side -- mirrors
    // MyGaitScheduler::step()'s own (private) theta0 computation.
    theta0 = (curLegState == LegState::RSt) ? M_PI / 2.0 : -M_PI / 2.0;

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
    double side = (curLegState == LegState::RSt) ? 1.0 : -1.0; // keyed on RSt to agree with KinWBC.cpp on DSt -- see curSwingFootPos_W's comment above
    hipPos_W = base_pos + Rz * Eigen::Vector3d(0, side * hip_width / 2.0, 0);

    // inPlaceOnly is caller-controlled (see test files toggling it for
    // WARM_UP vs WALK) -- no longer forced here now that this overload also
    // drives real forward walking, not just WARM_UP testing.
    return;
}

void FootPlacement::StepSwingPlanning(const RobotWrapper &rb_wrapper,
                                        const MyGaitScheduler &gait_scheduler,
                                         const JoyStickInterpreter &joyStick,
                                        const CP_Planning &cp_planner)
{
    updateFromRobot(rb_wrapper, gait_scheduler, joyStick);

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

    // Z-lift ignores inPlaceOnly -- same rationale as the cp_planner-driven
    // overload below: "in place" only pins X/Y (no forward stepping), the
    // foot should still lift/lower straight up/down; only phi>1.0 (already
    // landed/holding) suppresses it. legState==DSt means there's no real
    // swing target yet (still first cycle) -- keep both feet grounded.
    if (phi > 1.0 || legState == LegState::DSt)
        pDesCur[2] = posStart_W(2);
    else
        pDesCur[2] = posStart_W(2) +
                     stepHeight * 0.5 * (1 - cos(2 * 3.1415 * phi)) +
                     (posDes_W(2) - posStart_W(2)) / (2 * 3.1415) * (2 * 3.1415 * phi - sin(2 * 3.1415 * phi));
}

void FootPlacement::StepSwingPlanning(const RobotWrapper &rb_wrapper, const JoyStickInterpreter &joyStick,const CP_Planning &cp_planner)
{
    updateFromRobot(rb_wrapper, joyStick, cp_planner);

    double step_length = cp_planner.step_length;

    // Foot-local forward target: exactly one step-length ahead of where THIS
    // foot lifted off (posStart_W), not the globally-drifting CoM position
    // (cp_planner.cxi_x_). With the 1-cycle swing delay, cxi_xd_ has already
    // been advanced for both the cycle that just finished AND the new cycle
    // starting now by the time this foot's swing begins, so chasing cxi_x_
    // directly forced every swing to cover ~2 step-lengths instead of 1,
    // permanently, not just at startup -- that's what was diverging.
    posDes_W[0] = posStart_W(0) + step_length;
    posDes_W[1] = hipPos_W[1] ;

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

    // Z-lift ignores inPlaceOnly -- "in place" (see updateFromRobot()'s
    // inPlaceOnly=true, line ~145) means no FORWARD stepping (X/Y pinned
    // above), but the foot should still lift and lower straight up/down;
    // only phi>1.0 (already landed/holding) should suppress it. legState==DSt
    // means leg_state_swing_ hasn't seen a real one-cycle-old target yet
    // (still the very first cycle) -- keep both feet grounded.
    if (phi > 1.0 || legState == LegState::DSt)
        pDesCur[2] = posStart_W(2);
    else
        pDesCur[2] = posStart_W(2) +
                     stepHeight * 0.5 * (1 - cos(2 * 3.1415 * phi)) +
                     (posDes_W(2) - posStart_W(2)) / (2 * 3.1415) * (2 * 3.1415 * phi - sin(2 * 3.1415 * phi));

    return;
}

void FootPlacement::StepSwingPlanning(const RobotWrapper &rb_wrapper, const MyGaitScheduler &gait_scheduler, const JoyStickInterpreter &joyStick)
{
    updateFromRobot(rb_wrapper, gait_scheduler, joyStick);

    // Pure Raibert heuristic -- ported directly from OpenLoong-Dyn-Control's
    // FootPlacement::getSwingPos(). No capture-point/ZMP preview at all:
    // land the foot where the CoM's current velocity (plus a feedback term
    // on the velocity ERROR vs. the commanded one) says it will be by the
    // time the swing finishes.
    Eigen::Matrix3d KP = Eigen::Matrix3d::Zero(), Rz;
    KP(0, 0) = kp_vx;
    KP(1, 1) = kp_vy;
    Rz << cos(yawCur), -sin(yawCur), 0,
          sin(yawCur),  cos(yawCur), 0,
          0, 0, 1;
    KP = Rz * KP * Rz.transpose();

    posDes_W = hipPos_W - KP * (desV_W - curV_W) + 0.5 * tSwing * curV_W + curV_W * (1 - phi) * tSwing;

    // Yaw-rate correction: predicts where the hip-offset point will be at
    // touchdown given the current (and commanded) turning rate, same as
    // OpenLoong's thetaF term.
    double thetaF = yawCur + theta0 + omegaZ_W * (1 - phi) * tSwing + 0.5 * omegaZ_W * tSwing + kp_wz * (omegaZ_W - desWz_W);
    posDes_W(0) += 0.5 * hip_width * (cos(thetaF) - cos(yawCur + theta0));
    posDes_W(1) += 0.5 * hip_width * (sin(thetaF) - sin(yawCur + theta0));

    posDes_W(2) = base_pos(2) - legLength + zOff_W;

    double xOff_W(0), yOff_W(0);
    if (legState == LegState::LSt)
    {
        xOff_W = cos(yawCur) * xOff_L - sin(yawCur) * yOff_L;
        yOff_W = sin(yawCur) * xOff_L + cos(yawCur) * yOff_L;
    }
    else if (legState == LegState::RSt)
    {
        xOff_W = cos(yawCur) * xOff_L - sin(yawCur) * (-yOff_L);
        yOff_W = sin(yawCur) * xOff_L + cos(yawCur) * (-yOff_L);
    }
    posDes_W(0) += xOff_W;
    posDes_W(1) += yOff_W;

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

    if (phi > 1.0 || legState == LegState::DSt)
        pDesCur[2] = posStart_W(2);
    else
        pDesCur[2] = posStart_W(2) +
                     stepHeight * 0.5 * (1 - cos(2 * 3.1415 * phi)) +
                     (posDes_W(2) - posStart_W(2)) / (2 * 3.1415) * (2 * 3.1415 * phi - sin(2 * 3.1415 * phi));

    return;
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

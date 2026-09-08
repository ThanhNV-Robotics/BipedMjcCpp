/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024-2025 Humanoid Robot (Shanghai) Co., Ltd.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/

#pragma once

#include <Eigen/Dense>
#include <string>
#include "data_type.h"
#include "robot_wrapper.h"
#include "my_gait_scheduler.h"
#include "joystick_interpreter.h"
#include "CP_Planning.h"

class FootPlacement
{
public:
    double kp_vx{0}, kp_vy{0}, kp_wz{0};
    double legLength{1};
    double stepHeight{0.05};
    double hip_width{0.334}; // fixed geometry constant (was DataBus-sourced runtime state); matches computeInitial_Stand()'s width_hips
    // foot-end placement offsets, in body frame (x/y) and world frame (z).
    // Loaded from yaml foot_placement: x_offset / y_offset / z_offset.
    double xOff_L{0.0};  // body-frame forward offset
    double yOff_L{0.0};   // body-frame lateral offset (positive = inward)
    double zOff_W{0.0}; // world-frame vertical offset
    double phi{0};      // phase varialbe for trajectory generation, must between 0 and 1
    double tSwing{0.4}; // swing time
    // yamlPath's "foot_placement:" block supplies stepHeight/hip_width/offsets --
    // see config/step_planning_cf.yaml
    FootPlacement(const std::string &yamlPath);
    Eigen::Vector3d posStart_W, posDes_W, hipPos_W;
    Eigen::Vector3d desV_W, curV_W;
    double desWz_W;
    Eigen::Vector3d base_pos;
    LegState legState{LegState::DSt};
    // when true, StepSwingPlanning() pins x,y at the swing foot's liftoff
    // position (posStart_W) instead of stepping toward posDes_W -- lift and
    // lower straight up/down in place, no forward stepping. Useful for
    // isolating vertical swing-leg motion (e.g. combined with CoM sway)
    // from actual forward walking.
    bool inPlaceOnly{false};

    double Trajectory(double phase, double des1, double des2);
    void StepSwingPlanning(const RobotWrapper &rb_wrapper, const MyGaitScheduler &gait_scheduler, const JoyStickInterpreter &joyStick,const CP_Planning &cp_planner);

    void updateFromRobot(const RobotWrapper &rb_wrapper, const MyGaitScheduler &gait_scheduler, const JoyStickInterpreter &joyStick);
    Eigen::Vector3d getSwingDesPos() const { return Eigen::Vector3d(pDesCur[0], pDesCur[1], pDesCur[2]); }

private:
    double pDesCur[3]{0};
    double yawCur;
    double theta0;
    double omegaZ_W;
    bool stretchLeg{false};
    double zStretch{0};
    bool finish_Stretch;

    // tracks the swing leg across ticks so updateFromRobot() can detect a
    // stance-leg transition and snapshot the swing foot's liftoff position
    // into posStart_W. Mirrors MyGaitScheduler::step()'s own swingStartPos_W
    // bookkeeping, which isn't wired to real robot state in the new
    // pipeline (its dataBusRead() is commented out), so this class tracks
    // it independently instead of depending on that.
    LegState swingLegPrev_{LegState::DSt};
    bool swingInit_{false};
};

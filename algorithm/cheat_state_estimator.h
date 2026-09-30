#pragma once

#include <mujoco/mujoco.h>
#include <string>
#include <vector>
#include "robot_wrapper.h"

// Reads the robot's TRUE state directly from MuJoCo's mjData -- no Kalman
// filter, no IMU integration, zero estimation error. A real robot has no
// way to obtain this (base position/velocity aren't directly measurable,
// which is exactly why StateEstimator exists), so this is a diagnostic-only
// tool: swap it in for the real StateEstimator to isolate whether a control
// loop's instability under real feedback is caused by state-ESTIMATION
// noise specifically, versus closing the feedback loop at all.
class CheatStateEstimator
{
public:
  // jointNames must be in the same order as robot_wrapper.jointNames_ (its
  // q/dq's joint segment), same convention as MJ_Interface/the various
  // tests' jointQposAdr/jointQvelAdr lookups.
  CheatStateEstimator(const mjModel *model, const std::vector<std::string> &jointNames);

  // Writes robot_wrapper.q/dq directly from mj_data's ground-truth
  // floating-base + joint state, in the layout robot_wrapper expects:
  //   q:  [base_pos_world(3), base_quat_xyzw(4), joint_pos(na)]
  //   dq: [base_lin_vel_local(3), base_ang_vel_local(3), joint_vel(na)]
  void update(const mjModel *model, const mjData *data, RobotWrapper &robot_wrapper);

private:
  int freeQposAdr_;
  int freeQvelAdr_;
  std::vector<int> jointQposAdr_;
  std::vector<int> jointQvelAdr_;
};

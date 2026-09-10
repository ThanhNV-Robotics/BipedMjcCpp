#pragma once

#include "KinWBC.h" //
#include "data_type.h"
#include "qpOASES.hpp" // QP solver
#include "robot_wrapper.h"
#include "useful_math.h"
#include <iostream>
#include <string>
#include <vector>

class DynWBC {
public:
  DynWBC(const std::string &config_yaml_path, RobotWrapper &robot_wrapper,
         bool verbose);
  void computeCommandAcceleration();

  VectorXd computeTorque(); // main function to solve the QP problem

  void updateRobotState(RobotWrapper &robot_wrapper); // update robot feedback state

private:
  std::vector<double> maxTorque_;
  qpOASES::QProblem QP_prob;
  std::vector<double> Kp_j_, Kd_j_; // Kp and Kd of the joint controller
  int na_;                          // number of actuated joints
  std::vector<std::string> joint_name_list_;
  VectorXd q_, dq_, ddq_, q_des_, dq_des_; // reference configuration and velocity
  MatrixXd Ma_, Mu_; // mass matrix corresponding to actuated (leg) and underactuated (base link) dof
  VectorXd ha_, hu_; // nonlinear dynamics term corresponding to actuated and underactuated
  MatrixXd Ja_, Ju_; // Jacobian matrix fo the contact frame corresponding to actuated and underactuated dof
   
  VectorXd ddq_cmd;
};

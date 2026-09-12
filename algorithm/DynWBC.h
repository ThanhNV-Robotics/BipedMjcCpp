#pragma once

#include "KinWBC.h" //
#include "data_type.h"
#include "qpOASES.hpp" // QP solver
#include "robot_wrapper.h"
#include "useful_math.h"
#include <iostream>
#include <string>
#include <vector>
#include "MyStateEstimator.h"

class DynWBC {
public:
  DynWBC(const std::string &joint_config_yaml_path, const std::string &qp_config_yaml_path,
     RobotWrapper &robot_wrapper,
         bool verbose);

  VectorXd computeTorque(KinWBC& kin_wbc_sol, RobotWrapper &robot_wrapper); // main function to solve the QP problem

  void updateRobotState(RobotWrapper &robot_wrapper, StateEstimator &state_estimator); // update robot feedback state

  void setupQPproblem (RobotWrapper &robot_wrapper);

  
private:
  const int contact_dim_ = 6; // force and moment
  VectorXd maxTorque_;
  qpOASES::QProblem QP_prob;
  
  LegState contact_state_{DSt};

  MatrixXd W_c_; // contact wrench weight matrix

  MatrixXd Kp_j_, Kd_j_;  // joint PD gain matrices (diagonal, na x na)
  MatrixXd Kp_b_, Kd_b_;  // base link PD gain matrices (diagonal, 6x6)
  MatrixXd Kp_, Kd_;
  int na_;                          // number of actuated joints
  std::vector<std::string> joint_name_list_;
  VectorXd q_, dq_, ddq_, q_des_, dq_des_; // reference configuration and velocity
  MatrixXd Ma_, Mu_; // mass matrix corresponding to actuated (leg) and underactuated (base link) dof
  VectorXd ha_, hu_; // nonlinear dynamics term corresponding to actuated and underactuated
  MatrixXd Jc_, Ja_, Ju_; // Jacobian matrix fo the contact frame corresponding to actuated and underactuated dof
   
  VectorXd ddq_cmd;

  // QP problem dimensions
  // Decision variable: x = [τ(na) ; fc(nc*nContacts)]  →  nv_qp = na + 12
  int QP_nv_des{0}; // total decision variables: joint torques + contact wrenches
  int QP_nc_des{0}; // number of inequality constraints (friction cone, ...)

  // Flat row-major buffers — qpOASES takes const real_t* pointers, so
  // std::vector<real_t> is the correct host type; call .data() to get the ptr.
  std::vector<qpOASES::real_t> qp_H_;          // Hessian           [nv x nv]
  std::vector<qpOASES::real_t> qp_A_;          // constraint matrix [nc x nv]
  std::vector<qpOASES::real_t> qp_g_;          // gradient          [nv]
  std::vector<qpOASES::real_t> qp_lb_;         // simple lower bound [nv]
  std::vector<qpOASES::real_t> qp_ub_;         // simple upper bound [nv]
  std::vector<qpOASES::real_t> qp_lbA_;        // lower bound on Ax [nc]
  std::vector<qpOASES::real_t> qp_ubA_;        // upper bound on Ax [nc]
  std::vector<qpOASES::real_t> xOpt_iniGuess_; // warm-start guess  [nv]
};

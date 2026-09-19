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

#include <memory>

class DynWBC {
public:
  DynWBC(const std::string &joint_config_yaml_path,
         const std::string &qp_config_yaml_path, 
         RobotWrapper &robot_wrapper, 
         bool verbose);

  VectorXd computeTorque(KinWBC& kin_wbc_sol, RobotWrapper &robot_wrapper); // main function to solve the QP problem

  void updateRobotState(RobotWrapper &robot_wrapper, StateEstimator &state_estimator); // update robot feedback state

  VectorXd getOptimalContactWrench();
  void setupQPproblem (RobotWrapper &robot_wrapper);

private:
  const int na_ = 12;
  const int nv_ = na_+6;
  const int contact_dim_ = 6; // force and moment
  LegState contact_state_;
  
  //-------------------------------------------------------------------------
  // attributes for QP problem
  //-------------------------------------------------------------------------
  std::unique_ptr<qpOASES::QProblem> qp_prob_;
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

  // QP problem dimensions
  // Decision variable: x = [Fr(6) ; ddxc(6) ; delta_ddq(6+na) ; tau (na)]
  // Fr - in R6: reaction wrech (force and moment) at the contact point
  // ddxc - in R6: acceleration of the contact point in world frame
  // delta_ddq - in R(6+na): joint acceleration command increment
  // tau - in Rna: joint torque
  int n_Fr_, n_ddxc_;
  const int n_dof_ = 18;
  const int n_tau_ = 12;

  int QP_numOfvars_{0}; // total decision variables: joint torques + contact wrenches
  int QP_numOfconstr_{0}; // number of inequality constraints (friction cone, ...)

  // cost fcn J = Fr^T*Wr*Fr + ddxc^T*Wc*ddxc + delta_ddq^T*W_ddq*delta_ddq
  MatrixXd W_tau_;
  MatrixXd Wr_single_, Wr_, Wc_single_, Wc_, W_ddq_; // weight matrix for the cost function
  
  //--------constraints attributes----------------------------
  // Linear Friction cone constraint
  // U*Fr >= 0
  double muy_; // friction coefficient, loaded from yaml config file
  MatrixXd U_single_,U_; // built based on muy

  // Normal reaction force constraints:
  // S*Fr <= Fz_max
  // where S is a selection matrix that extract the normal reaction force Fr_z
  double Fz_max_; // upper bound on normal contact force, loaded from yaml config file
  MatrixXd S_; // size depends on the contact state

  // contact acceleration equality constraints
  // dd_xc = Jc*ddq + dJc*dq
  MatrixXd Jc_, dJc_;
  VectorXd ddq_, dq_, q_;

  // Dynamics equality constraint
  // M*ddq + h(q,dq) = S_tau * tau + Jc'*Fr
  // S_tau = diag(0(6x1) , I(na_)) is the selection matrix
  MatrixXd Mq_, Jc_T_, S_tau_;
  VectorXd h_nl_;
  
  // acceleration command
  // ddq_cmd_ = ddq_des_ - Kp*(q_-q_des_) - Kd*(dq_-dq_des_) 
  VectorXd ddq_cmd_, q_des_, dq_des_;
  MatrixXd Kp_b_,Kd_b_,Kp_j_,Kd_j_,Kp_,Kd_;

  // joint torque limit vector
  // -tau_limit_ <= tau <= tau_limit_
  VectorXd tau_lim_;

  // actuated joint names, in the order read from joint_config_yaml_path --
  // cross-checked against robot_wrapper.jointNames_ in the constructor since
  // both must agree with the URDF declaration order (see the load-bearing-
  // order note at the top of config/12dof_joint_config.yaml).
  std::vector<std::string> joint_names_;
};

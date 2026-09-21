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

  void solveWBQP(KinWBC& kin_wbc_sol, RobotWrapper &robot_wrapper, StateEstimator &state_estimator); // main function to solve the QP problem

  void updateRobotState(RobotWrapper &robot_wrapper, StateEstimator &state_estimator); // update robot feedback state

  VectorXd getOptimalContactWrench();
  VectorXd getOptimalJointTorque(); // actuated-joint torque consistent with ddq_cmd_/optSol_[0] via inverse dynamics
  void setupQPproblem (RobotWrapper &robot_wrapper);
  bool getQPStatus() const { return qp_solved_; } // true iff the last solveWBQP() call's QP solve succeeded
  double getDqNorm() const { return dq_.norm(); } // diagnostic: current joint+base velocity norm, set by updateRobotState()
  double getLastSolveTimeUs() const { return last_solve_time_us_; } // wall-clock time of the last qp_prob_->init() call, in microseconds
  int getLastNWSR() const { return last_nWSR_; } // number of working-set recalculations the last solve actually used
  double getMuy() const { return muy_; } // friction coefficient (eq. 12), for visualizing the friction cone
  double getFzMax() const { return Fz_max_; } // normal-force upper bound (eq. 13), for visualizing the friction cone

private:
  const int na_ = 12;
  const int nv_ = na_+6;
  const int contact_dim_ = 6; // force and moment
  LegState contact_state_;
  bool qp_solved_{false}; // set by solveWBQP() each call
  double last_solve_time_us_{0.0}; // wall-clock time of the last qp_prob_->init() call, microseconds
  int last_nWSR_{0}; // working-set recalculations actually used by the last solve

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
  std::vector<VectorXd> optSol_;               // Optimal solution of QP: optSol = [Fr]

  // QP problem dimensions
  // Decision variable: x = [Fr(6 per contact)] ONLY -- following
  // OpenLoong-Dyn-Control's approach (see doc discussion), ddq is NOT a
  // decision variable here at all: it comes in as a FIXED, already-solved
  // quantity from KinWBC::out_ddq (a dynamically-consistent, per-task
  // operational-space acceleration solve -- see KinWBC.cpp), and the QP's
  // only remaining job is finding contact wrenches consistent with that
  // ddq (dynamics equality) and the friction/normal-force limits.
  // Fr - in R6 per contact: reaction wrench (force and moment)
  int n_Fr_;
  const int n_dof_ = 18; // dimension of ddq_cmd_ (= KinWBC::out_ddq), NOT a QP variable count

  int QP_numOfvars_{0}; // total decision variables (== n_Fr_)
  int QP_numOfconstr_{0}; // number of constraints (friction cone + Fz_max + dynamics equality)

  // cost fcn J = Fr^T*Wr*Fr
  MatrixXd Wr_single_, Wr_; // weight matrix for the cost function

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

  // CoP (center-of-pressure) constraint, per contact: dx_lower <= x_cop <=
  // dx_upper, dy_lower <= y_cop <= dy_upper, keeping the solved contact
  // wrench's effective point of application within the foot's support
  // polygon -- a real stability margin the friction cone + Fz_max alone
  // don't enforce (those bound the wrench's magnitude/direction, not
  // where on the sole it's consistent with acting). x_cop=-My/Fz,
  // y_cop=Mx/Fz are the standard ZMP-on-the-sole formulas, but Fr's force/
  // moment components are expressed in WORLD frame (same frame as Jc_) --
  // My/Mx there don't mean "along the foot's own length/width" unless the
  // foot happens to be world-axis-aligned. So each solve rotates the
  // per-foot wrench into the foot's own frame (via robot_wrapper.rot_*_feet_W)
  // before applying the formula -- see buildCoPConstraintSingle(). Assumes
  // the contact frame origin (same point Jc_/pos_*_feet_W use) lies ON the
  // sole/ground plane; if that frame sits some height above the sole
  // instead, x_cop/y_cop pick up an extra h*fx / h*fy term this doesn't
  // account for.
  double dx_lower_, dx_upper_; // foot-local x (forward) CoP bounds, meters, loaded from yaml
  double dy_lower_, dy_upper_; // foot-local y (lateral) CoP bounds, meters, loaded from yaml
  MatrixXd buildCoPConstraintSingle(const Matrix3d &R) const;

  MatrixXd Jc_; // stacked contact Jacobian(s), used by the dynamics constraint below
  VectorXd dq_; // current joint+base velocity, kept only for the getDqNorm() diagnostic

  // Dynamics equality constraint (eq. 15), UNDERACTUATED/BASE ROWS ONLY,
  // with ddq FIXED (= ddq_cmd_, set from KinWBC::out_ddq each call, not
  // solved for) -- a direct linear equation in Fr alone:
  //   Jc_.transpose().topRows(6) * Fr = Mq_.topRows(6)*ddq_cmd_ + h_nl_.head(6)
  MatrixXd Mq_;
  VectorXd h_nl_;

  // ddq used in the dynamics constraint above -- set directly from
  // KinWBC::out_ddq in solveWBQP() each call (no PD law here anymore;
  // KinWBC's per-task dynamically-consistent acceleration solve already
  // produces a proper ddq target, see doc discussion on operational-space
  // control / Khatib's dynamically-consistent pseudo-inverse).
  VectorXd ddq_cmd_;

  // joint torque limit vector, loaded from yaml but not enforced here --
  // matches OpenLoong-Dyn-Control's approach of enforcing it downstream in
  // the low-level motor controller (PVT_Ctr) instead of inside the WBC QP.
  VectorXd tau_lim_;

  // actuated joint names, in the order read from joint_config_yaml_path --
  // cross-checked against robot_wrapper.jointNames_ in the constructor since
  // both must agree with the URDF declaration order (see the load-bearing-
  // order note at the top of config/12dof_joint_config.yaml).
  std::vector<std::string> joint_names_;
};

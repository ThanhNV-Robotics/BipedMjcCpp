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

  // current_leg_state: the PLANNED/commanded stance (defaults to DSt,
  // preserving existing callers' behavior exactly -- e.g. test_DynWBC.cpp's
  // STAND-only usage never passes this). Callers that also drive KinWBC
  // through WARM_UP/WALK (kin_task_init_walk/kin_task_walk, whose
  // static_contact/lift_foot or swing_leg tasks assume only ONE foot is
  // rigidly planted) MUST pass the matching LegState here, or DynWBC's QP
  // assumes double support while KinWBC's IK already assumed single
  // support that same tick.
  void solveWBQP(KinWBC& kin_wbc_sol, RobotWrapper &robot_wrapper, StateEstimator &state_estimator, LegState current_leg_state = LegState::DSt); // main function to solve the QP problem

  void updateRobotState(RobotWrapper &robot_wrapper, StateEstimator &state_estimator, LegState current_leg_state = LegState::DSt); // update robot feedback state

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
  std::vector<VectorXd> optSol_;               // Optimal solution of QP: optSol = [delta_r; delta_Fr]

  // QP problem dimensions
  // Decision variable: x = [delta_r (6, base-acceleration CORRECTION) ;
  // delta_Fr (6 per contact, contact-wrench CORRECTION around a
  // feedforward)] -- ported from OpenLoong-Dyn-Control's
  // WBC_priority::computeTau() (see plan.md's "Full QP formulation"
  // section for the full derivation). Unlike the earlier Fr-only
  // formulation, ddq IS partially a decision variable here: ddq_opt =
  // ddq_cmd_ + [delta_r;0...], only the floating base's own 6 rows get
  // corrected, the kinematic joint-space ddq is still trusted as-is. This
  // gives the dynamics equality constraint below slack to always be
  // satisfiable (delta_r absorbs whatever small kinematic/dynamic
  // mismatch delta_Fr alone can't), instead of forcing Fr alone to
  // compensate for all of it -- the "zero slack" failure mode that caused
  // most QP-failure/divergence symptoms under the old formulation.
  // Fr - in R6 per contact: reaction wrench (force and moment)
  int n_Fr_;
  const int n_dof_ = 18; // dimension of ddq_cmd_ (= KinWBC::out_ddq), NOT a QP variable count

  int QP_numOfvars_{0}; // total decision variables (== 6 + n_Fr_)
  int QP_numOfconstr_{0}; // number of constraints (friction cone + Fz_max + CoP + dynamics equality)

  const double g_ = 9.81; // gravity, for Fr_ff_'s nominal-weight computation

  // Feedforward/nominal contact wrench -- Fz = (total weight)/nContacts per
  // foot in contact (half each in DSt, full on the single stance foot in
  // LSt/RSt), all other components zero. delta_Fr is the QP-solved
  // CORRECTION around this baseline (Fr_opt = Fr_ff_ + delta_Fr), not the
  // raw force itself -- lets the cost penalize deviation from an
  // already-sensible stance instead of penalizing raw force magnitude
  // (which would perversely reward a SMALLER contact force, even though a
  // big one is needed just to hold the robot up). Rebuilt each
  // setupQPproblem() call from Mq_(0,0) -- see its assignment for why that
  // directly gives total mass with no separate bookkeeping needed.
  VectorXd Fr_ff_;

  // Cost weights for delta_r/delta_Fr respectively (OpenLoong's Q2/Q1,
  // applied as uniform diagonal matrices: w_ddq_b_*I(6), w_ddq_j_*I(n_Fr_)).
  // Loaded from wbc_config.yaml's qp_cost_weight.delta_joint_acceleration
  // (W_ddq_b/W_ddq_j) -- present in that yaml since early in this project
  // but never actually read until this formulation. w_ddq_b_ should be
  // MUCH larger than w_ddq_j_ (OpenLoong uses a ~1e6 ratio, 2e7 vs 2e1) so
  // the QP trusts the kinematic ddq almost completely and only reaches for
  // delta_r as a last resort.
  double w_ddq_b_, w_ddq_j_;

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
  // now in terms of the decision variables (delta_r, delta_Fr) --
  // M_bb*delta_r - Jc_bb^T*delta_Fr = -(Mq_.topRows(6)*ddq_cmd_ +
  // h_nl_.head(6)) + Jc_bb^T*Fr_ff_, where M_bb = Mq_.topLeftCorner(6,6)
  // and Jc_bb^T = Jc_.transpose().topRows(6) -- see setupQPproblem() and
  // plan.md's "Full QP formulation" for the derivation. Always solvable
  // (6 equations, 6+n_Fr_ unknowns).
  MatrixXd Mq_;
  VectorXd h_nl_;

  // ddq_cmd_ is the KINEMATIC ddq from KinWBC::out_ddq (ddq_opt =
  // ddq_cmd_ + [delta_r;0...] once the QP-solved delta_r is applied --
  // see getOptimalJointTorque()). KinWBC's per-task dynamically-consistent
  // acceleration solve already produces a proper ddq target, see doc
  // discussion on operational-space control / Khatib's
  // dynamically-consistent pseudo-inverse.
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

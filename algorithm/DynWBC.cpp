#include "DynWBC.h"
#include "data_type.h"
#include "robot_wrapper.h"
#include "yaml-cpp/yaml.h"
#include <algorithm>
#include <chrono>
#include <iterator>
#include <stdexcept>
#include <vector>

  DynWBC::DynWBC(const std::string &joint_config_yaml_path,
                 const std::string &qp_config_yaml_path,
                 RobotWrapper &robot_wrapper,
                 bool verbose){
  // joint_config_yaml_path: contain torque limit (maxTorque)
  // qp_config_yaml_path: parameters for QP problem

  //-----------------------------------------------------------------------
  // 1) joint_config_yaml_path -- maxTorque (loaded but not enforced here,
  //    see tau_lim_'s comment in DynWBC.h). Iterated in file order, same
  //    pattern PVT_Ctr uses for the same file -- that order is load-
  //    bearing (must match robot_wrapper.jointNames_ / URDF declaration
  //    order, per the top-of-file comment in 12dof_joint_config.yaml).
  //-----------------------------------------------------------------------
  YAML::Node joint_config = YAML::LoadFile(joint_config_yaml_path);
  joint_names_.clear();
  for (const auto &kv : joint_config)
    joint_names_.push_back(kv.first.as<std::string>());

  if (static_cast<int>(joint_names_.size()) != na_)
    throw std::runtime_error(
        "DynWBC: " + joint_config_yaml_path + " lists " +
        std::to_string(joint_names_.size()) + " joints, expected na_=" +
        std::to_string(na_));

  if (static_cast<int>(robot_wrapper.jointNames_.size()) != na_)
    throw std::runtime_error(
        "DynWBC: robot_wrapper.jointNames_ size (" +
        std::to_string(robot_wrapper.jointNames_.size()) +
        ") does not match na_=" + std::to_string(na_));

  for (int i = 0; i < na_; ++i)
    if (joint_names_[i] != robot_wrapper.jointNames_[i])
      throw std::runtime_error(
          "DynWBC: joint order mismatch at index " + std::to_string(i) +
          ": " + joint_config_yaml_path + " has '" + joint_names_[i] +
          "', robot_wrapper (URDF order) has '" +
          robot_wrapper.jointNames_[i] + "'");

  tau_lim_ = VectorXd::Zero(na_);
  for (int i = 0; i < na_; ++i) {
    const YAML::Node &jt = joint_config[joint_names_[i]];
    tau_lim_(i) = jt["maxTorque"].as<double>();
  }

  //-----------------------------------------------------------------------
  // 2) qp_config_yaml_path -- QP cost weight (eq. 11), friction
  //    coefficient (eq. 12), normal-force bound (eq. 13).
  //-----------------------------------------------------------------------
  YAML::Node qp_config = YAML::LoadFile(qp_config_yaml_path);
  const YAML::Node &cost = qp_config["qp_cost_weight"];

  muy_    = qp_config["friction_coefficient"]["muy"].as<double>();
  Fz_max_ = qp_config["maximum_normal_contact_force"]["Fz_max"].as<double>();

  // delta_r/delta_Fr cost weights (OpenLoong's Q2/Q1) -- see their comment
  // in DynWBC.h. Present in this yaml's qp_cost_weight.delta_joint_acceleration
  // since early in this project but unused until this formulation.
  const YAML::Node &ddq_cost = cost["delta_joint_acceleration"];
  w_ddq_b_ = ddq_cost["W_ddq_b"].as<double>();
  w_ddq_j_ = ddq_cost["W_ddq_j"].as<double>();

  const YAML::Node &cop = qp_config["cop_constraint"];
  dx_lower_ = cop["dx_lower"].as<double>();
  dx_upper_ = cop["dx_upper"].as<double>();
  dy_lower_ = cop["dy_lower"].as<double>();
  dy_upper_ = cop["dy_upper"].as<double>();

  //-----------------------------------------------------------------------
  // Friction cone (eq. 12): U_single_ * Fr >= 0, single contact point,
  // Fr = [fx, fy, fz, tx, ty, tz]. Box (pyramid) approximation on the
  // linear force components only -- moments are unconstrained by friction.
  // setupQPproblem() expands this block-diagonally per active contact.
  //   row 0: fz >= 0
  //   row 1: muy_*fz - fx >= 0   (fx <=  muy_*fz)
  //   row 2: muy_*fz + fx >= 0   (fx >= -muy_*fz)
  //   row 3: muy_*fz - fy >= 0   (fy <=  muy_*fz)
  //   row 4: muy_*fz + fy >= 0   (fy >= -muy_*fz)
  //-----------------------------------------------------------------------
  U_single_ = MatrixXd::Zero(5, contact_dim_);
  U_single_(0, 2) = 1.0;
  U_single_(1, 0) = -1.0; U_single_(1, 2) = muy_;
  U_single_(2, 0) =  1.0; U_single_(2, 2) = muy_;
  U_single_(3, 1) = -1.0; U_single_(3, 2) = muy_;
  U_single_(4, 1) =  1.0; U_single_(4, 2) = muy_;

  contact_state_ = LegState::DSt;
  ddq_cmd_ = VectorXd::Zero(n_dof_);

  if (verbose) {
    printf("===================== DynWBC config =====================\n");
    for (int i = 0; i < na_; ++i)
      printf("joint: %-24s maxTorque=%6.2f\n", joint_names_[i].c_str(), tau_lim_(i));
    printf("muy_=%.3f  Fz_max_=%.1f  W_ddq_b=%.3e  W_ddq_j=%.3e\n", muy_, Fz_max_, w_ddq_b_, w_ddq_j_);
    std::cout << "U_single_ (U_single_ * Fr >= 0):\n" << U_single_ << std::endl;
    printf("CoP bounds: dx=[%.3f, %.3f]  dy=[%.3f, %.3f]\n", dx_lower_, dx_upper_, dy_lower_, dy_upper_);
    printf("===========================================================\n");
  }

  return;
}

// Builds the 4 CoP inequality rows (dx_lower<=x_cop<=dx_upper, dy_lower<=
// y_cop<=dy_upper) for ONE contact, as a 4 x contact_dim_ matrix C such that
// C * Fr_foot >= 0 -- same "A*Fr >= 0, lbA=0, ubA=+inf" convention
// setupQPproblem() already uses for the friction cone (U_).
//
// Derivation: with Fr_foot=[fx,fy,fz,tx,ty,tz] in WORLD frame (same frame
// Jc_ uses) and R = rot_*_feet_W (foot-local axes expressed in world,
// columns r1=local x, r2=local y, r3=local z/normal), the wrench rotated
// into the foot's own frame is f_local=R^T*f_world, m_local=R^T*m_world, so
//   fz_local = r3 . f_world,  mx_local = r1 . m_world,  my_local = r2 . m_world
// The standard sole-frame CoP identities x_cop=-my_local/fz_local,
// y_cop=mx_local/fz_local, multiplied through by fz_local (>0, already
// enforced by U_'s fz>=0 row) to stay linear, give the 4 rows below.
MatrixXd DynWBC::buildCoPConstraintSingle(const Matrix3d &R) const
{
  const Vector3d r1 = R.col(0); // foot-local x (forward) axis, in world
  const Vector3d r2 = R.col(1); // foot-local y (lateral) axis, in world
  const Vector3d r3 = R.col(2); // foot-local z (normal)  axis, in world

  MatrixXd C = MatrixXd::Zero(4, contact_dim_);
  // dx_lower <= x_cop  <=>  -my_local - dx_lower_*fz_local >= 0
  C.block<1, 3>(0, 0) = -dx_lower_ * r3.transpose();
  C.block<1, 3>(0, 3) = -r2.transpose();
  // x_cop <= dx_upper  <=>  dx_upper_*fz_local + my_local >= 0
  C.block<1, 3>(1, 0) =  dx_upper_ * r3.transpose();
  C.block<1, 3>(1, 3) =  r2.transpose();
  // dy_lower <= y_cop  <=>  mx_local - dy_lower_*fz_local >= 0
  C.block<1, 3>(2, 0) = -dy_lower_ * r3.transpose();
  C.block<1, 3>(2, 3) =  r1.transpose();
  // y_cop <= dy_upper  <=>  dy_upper_*fz_local - mx_local >= 0
  C.block<1, 3>(3, 0) =  dy_upper_ * r3.transpose();
  C.block<1, 3>(3, 3) = -r1.transpose();
  return C;
}

void DynWBC::solveWBQP(KinWBC& kin_wbc_sol, RobotWrapper &robot_wrapper, StateEstimator &state_estimator, LegState current_leg_state)
{
    //-----------------------------------------------
    // Update internal states
    //-----------------------------------------------
    updateRobotState(robot_wrapper, state_estimator, current_leg_state);

    //-----------------------------------------------
    // ddq for the dynamics constraint is NOT solved for -- it's taken
    // directly from KinWBC's per-task, dynamically-consistent operational-
    // space acceleration solve (out_ddq). See KinWBC.cpp / the discussion
    // on Khatib's operational-space formulation for why this is a properly
    // derived acceleration target rather than an ad hoc PD law.
    //-----------------------------------------------
    ddq_cmd_ = kin_wbc_sol.out_ddq;

    //-----------------------------------------------
    // Build QP problem
    //-----------------------------------------------
    setupQPproblem(robot_wrapper);


    //-----------------------------------------------
    // Solve QP problem
    //-----------------------------------------------
    // qpOASES::QProblemB::setH()/setA() (called internally by init() below)
    // do NOT copy the H/A data -- they wrap the raw pointer we pass them in
    // a SymDenseMat/DenseMatrix and set freeHessian_/freeConstraintMatrix_
    // = true, meaning qp_prob_ will `delete` that pointer itself the next
    // time it's replaced/destroyed (see QProblemB::setH()/QProblem::setA()).
    // Passing qp_H_.data()/qp_A_.data() directly would hand qpOASES a
    // pointer OWNED by our own std::vector members -- qpOASES freeing it
    // out from under the still-alive vector (and the vector separately
    // trying to free/reallocate the same memory later) is exactly a
    // double-free / mismatched-deallocator bug, which is what surfaced as
    // "malloc(): unaligned tcache chunk detected" once qp_prob_ got
    // reconstructed on a later tick. Fix: hand qpOASES its OWN heap copies
    // (new[]), which it can safely own and free -- our qp_H_/qp_A_ vectors
    // stay independently ours. (qp_g_/qp_lb_/qp_ub_/qp_lbA_/qp_ubA_ don't
    // have this problem -- QProblemB::setLB() etc. memcpy() into internal
    // storage instead of wrapping the pointer.)
    qpOASES::real_t *H_owned = new qpOASES::real_t[qp_H_.size()];
    std::copy(qp_H_.begin(), qp_H_.end(), H_owned);
    qpOASES::real_t *A_owned = new qpOASES::real_t[qp_A_.size()];
    std::copy(qp_A_.begin(), qp_A_.end(), A_owned);

    qpOASES::int_t nWSR = 1000; // setToReliable()'s more conservative homotopy sometimes needs more than 100 (observed intermittent RET_MAX_NWSR_REACHED-style failures at nWSR=100); solve time stays ~3-4ms regardless since this is just a ceiling, not a fixed cost
    const auto solve_t0 = std::chrono::steady_clock::now();
    qpOASES::returnValue status = qp_prob_->init(
        H_owned, qp_g_.data(), A_owned,
        qp_lb_.data(), qp_ub_.data(), qp_lbA_.data(), qp_ubA_.data(), nWSR);
    const auto solve_t1 = std::chrono::steady_clock::now();
    last_solve_time_us_ = std::chrono::duration<double, std::micro>(solve_t1 - solve_t0).count();
    last_nWSR_ = static_cast<int>(nWSR); // init() overwrites nWSR with the actual count used

    qp_solved_ = (status == qpOASES::SUCCESSFUL_RETURN);

    optSol_.assign(1, VectorXd::Zero(0));
    if (!qp_solved_) {
        std::cout << "[DynWBC] QP solve failed! Status code: " << status
                   << "  |dq_|=" << dq_.norm() << "  |ddq_cmd_|=" << ddq_cmd_.norm() << std::endl;
        return;
    }

    xOpt_iniGuess_.assign(QP_numOfvars_, 0.0);
    qp_prob_->getPrimalSolution(xOpt_iniGuess_.data());
    // x = [delta_r (6); delta_Fr (n_Fr_)] -- getOptimalContactWrench()/
    // getOptimalJointTorque() split this back into delta_r/delta_Fr and
    // recover the actual ddq_opt/Fr_opt (ddq_cmd_+[delta_r;0...],
    // Fr_ff_+delta_Fr).
    optSol_[0] = Eigen::Map<const Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, 1>>(
        xOpt_iniGuess_.data(), QP_numOfvars_).cast<double>();
}

void DynWBC::updateRobotState(RobotWrapper &robot_wrapper, StateEstimator &state_estimator, LegState current_leg_state)
{
   robot_wrapper.computeDyn(); // compute robot dynamics terms

   // Set from the caller's PLANNED leg state (see solveWBQP()'s comment),
   // not a SENSED contact signal -- state_estimator.getContactState()
   // legitimately reports LSt/RSt during normal standing too (weight-shift
   // noise, brief single-support blips), which previously caused qp_prob_
   // to be (re)constructed at the wrong size for that tick's actual task
   // list. setupQPproblem() rebuilds qp_prob_ fresh every call sized for
   // THIS tick's QP_numOfvars_/QP_numOfconstr_, and the LSt/RSt branches
   // there already set n_Fr_/QP_numOfvars_/Jc_/U_ correctly (just
   // without a CoP constraint) -- so as long as this always matches
   // whatever contact assumption KinWBC's active task list made THIS same
   // tick, switching between DSt/LSt/RSt tick-to-tick is safe.
   this->contact_state_ = current_leg_state;
   this->Mq_ = robot_wrapper.dyn_M;
   this->h_nl_ = robot_wrapper.dyn_Non;
   this->dq_ = robot_wrapper.dq;

   return;
}

void DynWBC::setupQPproblem (RobotWrapper &robot_wrapper)
{

  // QP problem is defined as:
  // J = 1/2x^THx + g^Tx
  // s.t. x_lb <= x <= x_ub
  // lb_A <= Ax <= ub_A
  // Decision variable x = [delta_r (6); delta_Fr (n_Fr_)] -- see the
  // layout comment in DynWBC.h / plan.md's "Full QP formulation".

  // Check the contact state
  int nContacts = 0;
  // CoP constraint rows, built only for DSt below (LSt/RSt are already
  // known-incomplete stubs, see updateRobotState()'s comment on why
  // contact_state_ is pinned to DSt) -- 0 rows by default so the
  // A_cop.rows() stacking below is a no-op for LSt/RSt.
  MatrixXd A_cop = MatrixXd::Zero(0, 0);

  switch (this->contact_state_)
  {
    case LegState::DSt: { // double support — stack both foot Jacobians
      nContacts = 2;
      this->Jc_.resize(2*contact_dim_, this->na_ + 6); // 2*6
      this->Jc_.topRows(contact_dim_)    = robot_wrapper.J_Lfeet_W;
      this->Jc_.bottomRows(contact_dim_) = robot_wrapper.J_Rfeet_W;

      // Expand friction cone matrix U block-diagonally for 2 feet: 5 rows
      // (friction cone) per contact, 6 cols (Fr) per contact -- NOT square.
      U_ = MatrixXd::Zero(2*5, 2*contact_dim_); // 10x12
      U_.block(0, 0,            5, contact_dim_) = U_single_;
      U_.block(5, contact_dim_, 5, contact_dim_) = U_single_;

      n_Fr_ = 2*contact_dim_;

      // CoP constraint (eq. not in the paper's eq.11-18 set, added
      // separately): 4 rows per foot, block-diagonal same as U_ above.
      A_cop = MatrixXd::Zero(2*4, 2*contact_dim_); // 8x12
      A_cop.block(0, 0,            4, contact_dim_) = buildCoPConstraintSingle(robot_wrapper.rot_L_feet_W);
      A_cop.block(4, contact_dim_, 4, contact_dim_) = buildCoPConstraintSingle(robot_wrapper.rot_R_feet_W);

      break;
    }
    case LegState::LSt: // left stance — only left foot in contact
      nContacts = 1;
      Jc_ = robot_wrapper.J_Lfeet_W; // 6 x nv
      U_ = U_single_;
      n_Fr_ = contact_dim_;
      break;
    case LegState::RSt: // right stance — only right foot in contact
      nContacts = 1;
      Jc_ = robot_wrapper.J_Rfeet_W; // 6 x nv
      U_ = U_single_;
      n_Fr_ = contact_dim_;
      break;
  }

  QP_numOfvars_ = 6 + n_Fr_;

  // Feedforward nominal contact wrench Fr_ff_ -- Fz = (total weight) /
  // nContacts per foot in contact, all else zero. Total mass read directly
  // off Mq_(0,0): for ANY free-flyer-jointed robot the floating-base mass
  // matrix's top-left 3x3 translational block is total_mass*I3 regardless
  // of configuration (standard rigid-body-dynamics identity), so this
  // needs no separate mass bookkeeping or yaml entry.
  const double total_mass = Mq_(0, 0);
  const double total_weight = total_mass * g_;
  Fr_ff_ = VectorXd::Zero(n_Fr_);
  for (int i = 0; i < nContacts; ++i)
    Fr_ff_(i * contact_dim_ + 2) = total_weight / nContacts;

  // Hessian: block-diagonal [Q2 (delta_r), Q1 (delta_Fr)] -- see
  // w_ddq_b_/w_ddq_j_'s comment in DynWBC.h.
  MatrixXd H = MatrixXd::Zero(QP_numOfvars_, QP_numOfvars_);
  H.topLeftCorner(6, 6) = w_ddq_b_ * MatrixXd::Identity(6, 6);
  H.bottomRightCorner(n_Fr_, n_Fr_) = w_ddq_j_ * MatrixXd::Identity(n_Fr_, n_Fr_);

  //-------------------------------------------------------------
  // -------------Construct constraints -------------------------
  // All inequality constraints are expressed against the ACTUAL contact
  // wrench Fr = Fr_ff_ + delta_Fr, so each becomes "(block * delta_Fr) >=/<=
  // bound - (block * Fr_ff_)" -- delta_r never appears in these (zero
  // columns in that half of each constraint row).
  //-------------------------------------------------------------

  // Friction cone (eq. 12): U_*(Fr_ff_+delta_Fr) >= 0  =>  U_*delta_Fr >= -U_*Fr_ff_
  MatrixXd A_fr_ = MatrixXd::Zero(U_.rows(), QP_numOfvars_);
  A_fr_.rightCols(n_Fr_) = U_;
  VectorXd b_fr_lo = -U_ * Fr_ff_;

  // Normal reaction force constraint (eq. 13): S_*(Fr_ff_+delta_Fr) <= Fz_max_
  S_ = MatrixXd::Zero(nContacts, n_Fr_);
  for (int i = 0; i < nContacts; ++i)
    S_(i, i * contact_dim_ + 2) = 1.0;
  MatrixXd A_Fzmax = MatrixXd::Zero(nContacts, QP_numOfvars_);
  A_Fzmax.rightCols(n_Fr_) = S_;
  VectorXd Fz_max_ub = Fz_max_ * VectorXd::Ones(nContacts) - S_ * Fr_ff_;

  // CoP constraint, same Fr_ff_-shift as above.
  MatrixXd A_cop_full = MatrixXd::Zero(A_cop.rows(), QP_numOfvars_);
  VectorXd b_cop_lo = VectorXd::Zero(A_cop.rows());
  if (A_cop.rows() > 0) {
    A_cop_full.rightCols(n_Fr_) = A_cop;
    b_cop_lo = -A_cop * Fr_ff_;
  }

  //-------------------------------------------------------------
  // Dynamic constraint (eq. 15), UNDERACTUATED/BASE ROWS ONLY, now in
  // terms of (delta_r, delta_Fr) -- see plan.md's "Full QP formulation"
  // for the derivation:
  //   M_bb*delta_r - Jc_bb^T*delta_Fr =
  //       -(Mq_.topRows(6)*ddq_cmd_ + h_nl_.head(6)) + Jc_bb^T*Fr_ff_
  // where M_bb = Mq_.topLeftCorner(6,6) (base-base mass matrix block) and
  // Jc_bb^T = Jc_.transpose().topRows(6). ALWAYS solvable -- 6 equations,
  // 6+n_Fr_ unknowns, delta_r free to absorb whatever delta_Fr can't.
  // An EQUALITY constraint (lbA = ubA = b_dyn), 6 rows.
  //-------------------------------------------------------------
  MatrixXd M_bb = Mq_.topLeftCorner(6, 6);
  MatrixXd Jc_bb_T = Jc_.transpose().topRows(6);
  MatrixXd A_dyn = MatrixXd::Zero(6, QP_numOfvars_);
  A_dyn.leftCols(6) = M_bb;
  A_dyn.rightCols(n_Fr_) = -Jc_bb_T;
  VectorXd b_dyn = -(Mq_.topRows(6) * ddq_cmd_ + h_nl_.head(6)) + Jc_bb_T * Fr_ff_;

  //-------------------------------------------------------------
  // Stack every constraint block into the single matrix/bounds qpOASES
  // needs (A_fr_, A_Fzmax, A_cop_full inequalities; A_dyn equality):
  //   A_fr_     : lbA = b_fr_lo,  ubA = +inf        (eq. 12, friction cone)
  //   A_Fzmax   : lbA = -inf,     ubA = Fz_max_ub    (eq. 13, normal force)
  //   A_cop_full: lbA = b_cop_lo, ubA = +inf        (CoP-in-support-polygon)
  //   A_dyn     : lbA = ubA = b_dyn                  (eq. 15, equality)
  //-------------------------------------------------------------
  QP_numOfconstr_ = A_fr_.rows() + A_Fzmax.rows() + A_cop_full.rows() + A_dyn.rows();

  MatrixXd A = MatrixXd::Zero(QP_numOfconstr_, QP_numOfvars_);
  VectorXd lbA = VectorXd::Zero(QP_numOfconstr_);
  VectorXd ubA = VectorXd::Zero(QP_numOfconstr_);

  int row = 0;
  A.middleRows(row, A_fr_.rows()) = A_fr_;
  lbA.segment(row, A_fr_.rows()) = b_fr_lo;
  ubA.segment(row, A_fr_.rows()).setConstant(qpOASES::INFTY);
  row += A_fr_.rows();

  A.middleRows(row, A_Fzmax.rows()) = A_Fzmax;
  lbA.segment(row, A_Fzmax.rows()).setConstant(-qpOASES::INFTY);
  ubA.segment(row, A_Fzmax.rows()) = Fz_max_ub;
  row += A_Fzmax.rows();

  if (A_cop_full.rows() > 0) {
    A.middleRows(row, A_cop_full.rows()) = A_cop_full;
    lbA.segment(row, A_cop_full.rows()) = b_cop_lo;
    ubA.segment(row, A_cop_full.rows()).setConstant(qpOASES::INFTY);
    row += A_cop_full.rows();
  }

  A.middleRows(row, A_dyn.rows()) = A_dyn;
  lbA.segment(row, A_dyn.rows()) = b_dyn;
  ubA.segment(row, A_dyn.rows()) = b_dyn;
  row += A_dyn.rows();

  //-------------------------------------------------------------
  // Simple bounds on x itself. No physical bound applies inside the QP
  // (eq. 18's torque bound doesn't apply -- tau isn't a variable, and
  // isn't even computed here; see DynWBC.h's tau_lim_ comment). Generous
  // but FINITE bounds anyway, more conventional than literal
  // qpOASES::INFTY on every variable.
  //-------------------------------------------------------------
  const double kFiniteBound = 1000.0;
  VectorXd lb = VectorXd::Constant(QP_numOfvars_, -kFiniteBound);
  VectorXd ub = VectorXd::Constant(QP_numOfvars_,  kFiniteBound);

  //-------------------------------------------------------------
  // Flatten Eigen matrices into qpOASES's row-major real_t buffers.
  //-------------------------------------------------------------
  qp_H_.assign(QP_numOfvars_ * QP_numOfvars_, 0.0);
  Eigen::Map<Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      qp_H_.data(), QP_numOfvars_, QP_numOfvars_) = H.cast<qpOASES::real_t>();

  qp_g_.assign(QP_numOfvars_, 0.0); // no linear cost term

  qp_A_.assign(QP_numOfconstr_ * QP_numOfvars_, 0.0);
  Eigen::Map<Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      qp_A_.data(), QP_numOfconstr_, QP_numOfvars_) = A.cast<qpOASES::real_t>();

  qp_lb_.assign(QP_numOfvars_, 0.0);
  qp_ub_.assign(QP_numOfvars_, 0.0);
  Eigen::Map<Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, 1>>(qp_lb_.data(), QP_numOfvars_) = lb.cast<qpOASES::real_t>();
  Eigen::Map<Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, 1>>(qp_ub_.data(), QP_numOfvars_) = ub.cast<qpOASES::real_t>();

  qp_lbA_.assign(QP_numOfconstr_, 0.0);
  qp_ubA_.assign(QP_numOfconstr_, 0.0);
  Eigen::Map<Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, 1>>(qp_lbA_.data(), QP_numOfconstr_) = lbA.cast<qpOASES::real_t>();
  Eigen::Map<Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, 1>>(qp_ubA_.data(), QP_numOfconstr_) = ubA.cast<qpOASES::real_t>();

  // (Re)construct qp_prob_ sized for THIS tick's QP_numOfvars_/
  // QP_numOfconstr_ -- setupQPproblem() only builds the problem (qp_H_/
  // qp_g_/qp_A_/qp_lb_/qp_ub_/qp_lbA_/qp_ubA_ above are fully populated at
  // this point, and qp_prob_ is ready to init()); calling init()/
  // getPrimalSolution() is left to solveWBQP()'s separate solve step.
  // H = diag(w_ddq_b_*I6, w_ddq_j_*I(n_Fr_)), strictly positive diagonal
  // entries from qp_config.yaml -- positive definite.
  qp_prob_ = std::make_unique<qpOASES::QProblem>(QP_numOfvars_, QP_numOfconstr_, qpOASES::HST_POSDEF);
  qpOASES::Options options;
  // setToReliable() is the confirmed fix for a "Division by zero" ->
  // "Abnormal termination due to TQ factorisation" -> RET_INIT_FAILED_HOTSTART
  // failure seen with the default options preset on the earlier, larger
  // (42-variable) formulation, even though H/A were verified well-posed
  // there. Kept here since it's a strictly safer default regardless of
  // problem size.
  options.setToReliable();
  options.printLevel = qpOASES::PL_NONE;
  qp_prob_->setOptions(options);

  return;
}

VectorXd DynWBC::getOptimalContactWrench()
{
  // optSol_[0] = [delta_r; delta_Fr] -- the ACTUAL contact wrench callers
  // want is Fr_ff_ + delta_Fr (see DynWBC.h's Fr_ff_/decision-variable
  // comment), not the raw delta.
  return Fr_ff_ + optSol_[0].tail(n_Fr_);
}

VectorXd DynWBC::getOptimalJointTorque()
{
  // Inverse dynamics, actuated-joint rows only: given the QP-solved
  // ddq_opt (= ddq_cmd_ + [delta_r;0...]) and Fr_opt (= Fr_ff_+delta_Fr),
  // the full nv_-dim generalized force needed is Mq_*ddq_opt + h_nl_ -
  // Jc_^T*Fr_opt -- the first 6 (base) rows are exactly zero by
  // construction (that's what the dynamics equality constraint in
  // setupQPproblem() enforced when solving for delta_r/delta_Fr); the
  // remaining na_ rows are the actual actuated-joint torques. Same
  // approach OpenLoong-Dyn-Control's WBC_priority::computeTau() uses
  // (tauRes = dyn_M*eigen_ddq_Opt + dyn_Non - Jfe.transpose()*eigen_fr_Opt).
  VectorXd ddq_opt = ddq_cmd_;
  ddq_opt.head(6) += optSol_[0].head(6);
  VectorXd Fr_opt = Fr_ff_ + optSol_[0].tail(n_Fr_);
  VectorXd tau_full = Mq_ * ddq_opt + h_nl_ - Jc_.transpose() * Fr_opt;
  return tau_full.tail(na_);
}

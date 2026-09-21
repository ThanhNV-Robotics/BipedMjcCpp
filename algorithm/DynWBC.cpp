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

  // Wr_single_ is a SINGLE-contact-point (6x6) block; setupQPproblem()
  // expands it block-diagonally per active contact -- eq. 11's
  // F_r^T*Wr*F_r term is a per-contact 6-dim wrench, one per foot.
  const YAML::Node &wr = cost["contact_wrench"];
  Wr_single_ = MatrixXd::Zero(contact_dim_, contact_dim_);
  Wr_single_(0, 0) = wr["Wr_fx"].as<double>();
  Wr_single_(1, 1) = wr["Wr_fy"].as<double>();
  Wr_single_(2, 2) = wr["Wr_fz"].as<double>();
  Wr_single_(3, 3) = wr["Wr_tx"].as<double>();
  Wr_single_(4, 4) = wr["Wr_ty"].as<double>();
  Wr_single_(5, 5) = wr["Wr_tz"].as<double>();

  muy_    = qp_config["friction_coefficient"]["muy"].as<double>();
  Fz_max_ = qp_config["maximum_normal_contact_force"]["Fz_max"].as<double>();

  const YAML::Node &cop = qp_config["cop_constraint"];
  dx_lower_ = cop["dx_lower"].as<double>();
  dx_upper_ = cop["dx_upper"].as<double>();
  dy_lower_ = cop["dy_lower"].as<double>();
  dy_upper_ = cop["dy_upper"].as<double>();

  //-----------------------------------------------------------------------
  // Friction cone (eq. 12): U_single_ * Fr >= 0, single contact point,
  // Fr = [fx, fy, fz, tx, ty, tz]. Box (pyramid) approximation on the
  // linear force components only -- moments are unconstrained by friction.
  // setupQPproblem() expands this block-diagonally per active contact,
  // same as Wr_single_ above.
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
    std::cout << "Wr_single_ (diag):\n" << Wr_single_.diagonal().transpose() << std::endl;
    printf("muy_=%.3f  Fz_max_=%.1f\n", muy_, Fz_max_);
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

void DynWBC::solveWBQP(KinWBC& kin_wbc_sol, RobotWrapper &robot_wrapper, StateEstimator &state_estimator)
{
    //-----------------------------------------------
    // Update internal states
    //-----------------------------------------------
    updateRobotState(robot_wrapper, state_estimator);

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
    // x == Fr entirely now (the only decision variable), so no slicing needed.
    optSol_[0] = Eigen::Map<const Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, 1>>(
        xOpt_iniGuess_.data(), QP_numOfvars_).cast<double>();
}

void DynWBC::updateRobotState(RobotWrapper &robot_wrapper, StateEstimator &state_estimator)
{
   robot_wrapper.computeDyn(); // compute robot dynamics terms

   // Pinned to DSt for now -- double-support-only is the current target,
   // and setupQPproblem()'s LSt/RSt branches are known-incomplete stubs
   // (they never update n_Fr_, so a stale DSt-sized n_Fr_ gets used
   // against a freshly-6-sized QP_numOfvars_, a severe out-of-bounds
   // write). Letting contact_state_ follow
   // state_estimator.getContactState() -- which legitimately reports
   // LSt/RSt during normal standing (weight-shift noise, brief single-
   // support blips) -- crashed here; confirmed via gdb backtrace showing
   // qpOASES::QProblem constructed with _nV=6 (contact_dim_, the LSt/RSt
   // size) while n_Fr_ was still 12 from the previous DSt tick.
   // Revisit once LSt/RSt are actually implemented.
   this->contact_state_ = LegState::DSt;
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
  // Decision variable x = Fr ONLY -- see the layout comment in DynWBC.h.

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

      // Expand Wr_ block-diagonally for 2 feet
      Wr_ = MatrixXd::Zero(2*contact_dim_, 2*contact_dim_); // 12x12
      Wr_.block(0,             0,             contact_dim_, contact_dim_) = Wr_single_;
      Wr_.block(contact_dim_, contact_dim_, contact_dim_, contact_dim_) = Wr_single_;

      // Expand friction cone matrix U block-diagonally for 2 feet: 5 rows
      // (friction cone) per contact, 6 cols (Fr) per contact -- NOT square.
      U_ = MatrixXd::Zero(2*5, 2*contact_dim_); // 10x12
      U_.block(0, 0,            5, contact_dim_) = U_single_;
      U_.block(5, contact_dim_, 5, contact_dim_) = U_single_;

      n_Fr_ = 2*contact_dim_;
      QP_numOfvars_ = n_Fr_;

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
      Wr_ = Wr_single_;
      U_ = U_single_;
      n_Fr_ = contact_dim_;
      QP_numOfvars_ = n_Fr_;
      break;
    case LegState::RSt: // right stance — only right foot in contact
      nContacts = 1;
      Jc_ = robot_wrapper.J_Rfeet_W; // 6 x nv
      Wr_ = Wr_single_;
      U_ = U_single_;
      n_Fr_ = contact_dim_;
      QP_numOfvars_ = n_Fr_;
      break;
  }

  // Hessian H = Wr_ directly -- x IS Fr, no other blocks to assemble.
  MatrixXd H = Wr_;

  //-------------------------------------------------------------
  // -------------Construct constraints -------------------------
  //-------------------------------------------------------------

  // Friction cone (eq. 12): U_ * Fr >= 0
  MatrixXd A_fr_ = U_;

  // Normal reaction force constraint (eq. 13): S_ * Fr <= Fz_max_
  S_ = MatrixXd::Zero(nContacts, QP_numOfvars_);
  for (int i = 0; i < nContacts; ++i)
    S_(i, i * contact_dim_ + 2) = 1.0;
  VectorXd Fz_max_ub = Fz_max_ * VectorXd::Ones(nContacts);
  MatrixXd A_Fzmax = S_;

  //-------------------------------------------------------------
  // Dynamic constraint (eq. 15), UNDERACTUATED/BASE ROWS ONLY, ddq FIXED
  // (= ddq_cmd_, set from KinWBC::out_ddq in solveWBQP() -- NOT solved for
  // here). A direct linear equation in Fr alone:
  //   Jc_.transpose().topRows(6) * Fr = Mq_.topRows(6)*ddq_cmd_ + h_nl_.head(6)
  // An EQUALITY constraint (lbA = ubA = b_dyn), 6 rows.
  //-------------------------------------------------------------
  MatrixXd A_dyn = Jc_.transpose().topRows(6);
  VectorXd b_dyn = Mq_.topRows(6) * ddq_cmd_ + h_nl_.head(6);

  //-------------------------------------------------------------
  // Stack every constraint block into the single matrix/bounds qpOASES
  // needs (A_fr_, A_Fzmax, A_cop inequalities; A_dyn equality):
  //   A_fr_  : lbA = 0,    ubA = +inf        (eq. 12, U_*Fr >= 0)
  //   A_Fzmax: lbA = -inf, ubA = Fz_max_ub   (eq. 13, S_*Fr <= Fz_max_)
  //   A_cop  : lbA = 0,    ubA = +inf        (CoP-in-support-polygon, see buildCoPConstraintSingle())
  //   A_dyn  : lbA = ubA = b_dyn             (eq. 15, equality)
  //-------------------------------------------------------------
  QP_numOfconstr_ = A_fr_.rows() + A_Fzmax.rows() + A_cop.rows() + A_dyn.rows();

  MatrixXd A = MatrixXd::Zero(QP_numOfconstr_, QP_numOfvars_);
  VectorXd lbA = VectorXd::Zero(QP_numOfconstr_);
  VectorXd ubA = VectorXd::Zero(QP_numOfconstr_);

  int row = 0;
  A.middleRows(row, A_fr_.rows()) = A_fr_;
  lbA.segment(row, A_fr_.rows()).setZero();
  ubA.segment(row, A_fr_.rows()).setConstant(qpOASES::INFTY);
  row += A_fr_.rows();

  A.middleRows(row, A_Fzmax.rows()) = A_Fzmax;
  lbA.segment(row, A_Fzmax.rows()).setConstant(-qpOASES::INFTY);
  ubA.segment(row, A_Fzmax.rows()) = Fz_max_ub;
  row += A_Fzmax.rows();

  if (A_cop.rows() > 0) {
    A.middleRows(row, A_cop.rows()) = A_cop;
    lbA.segment(row, A_cop.rows()).setZero();
    ubA.segment(row, A_cop.rows()).setConstant(qpOASES::INFTY);
    row += A_cop.rows();
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
  // H = Wr_, block-diagonal with strictly positive diagonal entries from
  // qp_config.yaml -- positive definite.
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
  return optSol_[0];
}

VectorXd DynWBC::getOptimalJointTorque()
{
  // Inverse dynamics, actuated-joint rows only: given the (fixed) ddq_cmd_
  // and the QP-solved contact wrench Fr, the full nv_-dim generalized force
  // needed is Mq_*ddq_cmd_ + h_nl_ - Jc_^T*Fr -- the first 6 (base) rows
  // are exactly zero by construction (that's what the dynamics equality
  // constraint in setupQPproblem() enforced when solving for Fr); the
  // remaining na_ rows are the actual actuated-joint torques. Same
  // approach OpenLoong-Dyn-Control's WBC_priority::computeTau() uses
  // (tauRes = dyn_M*eigen_ddq_Opt + dyn_Non - Jfe.transpose()*eigen_fr_Opt).
  VectorXd tau_full = Mq_ * ddq_cmd_ + h_nl_ - Jc_.transpose() * optSol_[0];
  return tau_full.tail(na_);
}

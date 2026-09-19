#include "DynWBC.h"
#include "data_type.h"
#include "robot_wrapper.h"
#include "yaml-cpp/yaml.h"
#include <iterator>
#include <stdexcept>
#include <vector>

  DynWBC::DynWBC(const std::string &joint_config_yaml_path,
                 const std::string &qp_config_yaml_path,
                 RobotWrapper &robot_wrapper,
                 bool verbose){
  // joint_config_yaml_path: contain Kp and Kd gain and torque limit
  // qp_config_yaml_path: parameters for QP problem

  //-----------------------------------------------------------------------
  // 1) joint_config_yaml_path -- per-joint kp/kd (eq. 17's joint-space PD
  //    gain) and maxTorque (eq. 18's torque bound). Iterated in file order,
  //    same pattern PVT_Ctr uses for the same file -- that order is load-
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

  Kp_j_    = MatrixXd::Zero(na_, na_);
  Kd_j_    = MatrixXd::Zero(na_, na_);
  tau_lim_ = VectorXd::Zero(na_);
  for (int i = 0; i < na_; ++i) {
    const YAML::Node &jt = joint_config[joint_names_[i]];
    Kp_j_(i, i) = jt["kp"].as<double>();
    Kd_j_(i, i) = jt["kd"].as<double>();
    tau_lim_(i) = jt["maxTorque"].as<double>();
  }

  //-----------------------------------------------------------------------
  // 2) qp_config_yaml_path -- base PD gain, QP cost weights (eq. 11),
  //    friction coefficient (eq. 12), normal-force bound (eq. 13).
  //-----------------------------------------------------------------------
  YAML::Node qp_config = YAML::LoadFile(qp_config_yaml_path);

  const YAML::Node &base_pd = qp_config["base_pd_gain"];
  Kp_b_ = MatrixXd::Zero(6, 6);
  Kd_b_ = MatrixXd::Zero(6, 6);
  Kp_b_(0, 0) = base_pd["kp_x"].as<double>();
  Kp_b_(1, 1) = base_pd["kp_y"].as<double>();
  Kp_b_(2, 2) = base_pd["kp_z"].as<double>();
  Kp_b_(3, 3) = base_pd["kp_roll"].as<double>();
  Kp_b_(4, 4) = base_pd["kp_pitch"].as<double>();
  Kp_b_(5, 5) = base_pd["kp_yaw"].as<double>();
  Kd_b_(0, 0) = base_pd["kd_x"].as<double>();
  Kd_b_(1, 1) = base_pd["kd_y"].as<double>();
  Kd_b_(2, 2) = base_pd["kd_z"].as<double>();
  Kd_b_(3, 3) = base_pd["kd_roll"].as<double>();
  Kd_b_(4, 4) = base_pd["kd_pitch"].as<double>();
  Kd_b_(5, 5) = base_pd["kd_yaw"].as<double>();

  // Stack into the full nv_ = 6(base) + na_(joints) PD gain used by eq. 17:
  //   q̈^cmd = q̈^d + Kd_*(q̇^d - q̇) + Kp_*(q^d - q)
  // Base occupies rows/cols [0,6), joints [6,nv_) -- matches robot_wrapper's
  // dq layout [local_base_linear(3), local_base_angular(3), joints(na_)].
  Kp_ = MatrixXd::Zero(nv_, nv_);
  Kd_ = MatrixXd::Zero(nv_, nv_);
  Kp_.block(0, 0, 6, 6)     = Kp_b_;
  Kp_.block(6, 6, na_, na_) = Kp_j_;
  Kd_.block(0, 0, 6, 6)     = Kd_b_;
  Kd_.block(6, 6, na_, na_) = Kd_j_;

  const YAML::Node &cost = qp_config["qp_cost_weight"];

  // Wr_/Wc_ are SINGLE-contact-point (6x6) blocks; setupQPproblem() expands
  // them block-diagonally per active contact -- eq. 11's F_r^T*Wr*F_r and
  // ẍ_c^T*Wc*ẍ_c terms are each a per-contact 6-dim wrench/acceleration, one
  // per foot in contact, not a fixed-size quantity like delta_ddq is.
  const YAML::Node &wr = cost["contact_wrench"];
  Wr_single_ = MatrixXd::Zero(contact_dim_, contact_dim_);
  Wr_single_(0, 0) = wr["Wr_fx"].as<double>();
  Wr_single_(1, 1) = wr["Wr_fy"].as<double>();
  Wr_single_(2, 2) = wr["Wr_fz"].as<double>();
  Wr_single_(3, 3) = wr["Wr_tx"].as<double>();
  Wr_single_(4, 4) = wr["Wr_ty"].as<double>();
  Wr_single_(5, 5) = wr["Wr_tz"].as<double>();

  W_tau_ = MatrixXd::Zero(na_, na_); //

  const YAML::Node &wc = cost["contact_acceleration"];
  Wc_single_ = MatrixXd::Zero(contact_dim_, contact_dim_);
  Wc_single_(0, 0) = wc["Wc_x"].as<double>();
  Wc_single_(1, 1) = wc["Wc_y"].as<double>();
  Wc_single_(2, 2) = wc["Wc_z"].as<double>();
  Wc_single_(3, 3) = wc["Wc_roll"].as<double>();
  Wc_single_(4, 4) = wc["Wc_pitch"].as<double>();
  Wc_single_(5, 5) = wc["Wc_yaw"].as<double>();

  // W_ddq_ is fixed-size (nv_ x nv_): delta_ddq's dimension doesn't change
  // with the number of active contacts, unlike Wr_/Wc_ above.
  const YAML::Node &wddq = cost["delta_joint_acceleration"];
  const double W_ddq_b = wddq["W_ddq_b"].as<double>();
  const double W_ddq_j = wddq["W_ddq_j"].as<double>();
  W_ddq_ = MatrixXd::Zero(nv_, nv_);
  W_ddq_.block(0, 0, 6, 6).diagonal().setConstant(W_ddq_b);
  W_ddq_.block(6, 6, na_, na_).diagonal().setConstant(W_ddq_j);

  muy_    = qp_config["friction_coefficient"]["muy"].as<double>();
  Fz_max_ = qp_config["maximum_normal_contact_force"]["Fz_max"].as<double>();

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

  //-----------------------------------------------------------------------
  // 3) constant selection matrix for eq. 15:
  //      A*q̈ + b + g = (0_6 ; τ^cmd) + Jc^T*F_r
  //    S_tau_ maps τ (na_) into an nv_ vector with the floating-base rows
  //    zeroed -- S_tau_ = [0(6 x na_) ; I(na_)].
  //-----------------------------------------------------------------------
  S_tau_ = MatrixXd::Zero(nv_, na_);
  S_tau_.block(6, 0, na_, na_) = MatrixXd::Identity(na_, na_);

  contact_state_ = LegState::DSt;
  q_des_ = VectorXd::Zero(n_dof_);
  dq_des_ = VectorXd::Zero(n_dof_);
  ddq_cmd_ = VectorXd::Zero(n_dof_);
  
  if (verbose) {
    printf("===================== DynWBC config =====================\n");
    for (int i = 0; i < na_; ++i)
      printf("joint: %-24s kp=%6.2f kd=%5.2f maxTorque=%6.2f\n",
             joint_names_[i].c_str(), Kp_j_(i, i), Kd_j_(i, i), tau_lim_(i));
    std::cout << "Kp_b_ (diag):\n" << Kp_b_.diagonal().transpose() << std::endl;
    std::cout << "Kd_b_ (diag):\n" << Kd_b_.diagonal().transpose() << std::endl;
    std::cout << "Wr_single_ (diag):\n" << Wr_single_.diagonal().transpose() << std::endl;
    std::cout << "Wc_single_ (diag):\n" << Wc_single_.diagonal().transpose() << std::endl;
    printf("W_ddq_b=%.4f  W_ddq_j=%.4f\n", W_ddq_b, W_ddq_j);
    printf("muy_=%.3f  Fz_max_=%.1f\n", muy_, Fz_max_);
    std::cout << "U_single_ (U_single_ * Fr >= 0):\n" << U_single_ << std::endl;
    printf("===========================================================\n");
  }

  return;
}

VectorXd DynWBC::computeTorque(KinWBC& kin_wbc_sol, RobotWrapper &robot_wrapper)
{
    VectorXd tqr_cmd = VectorXd::Zero(na_);

    // Desired configuration/velocity for eq. 17, converted into the same
    // n_dof_ (18) minimal-coordinate layout q_ uses (see updateRobotState()).
    // kin_wbc_sol.q_des is nq_-dim (19: base_pos(3), quat(4), joints(na_));
    // kin_wbc_sol.out_dq is already tangent-space (nv_ = n_dof_ = 18), so
    // dq_des_ needs no conversion.
    q_des_ = VectorXd::Zero(n_dof_);
    q_des_.segment<3>(0) = kin_wbc_sol.q_des.head<3>();
    Eigen::Quaterniond quat_des_W(kin_wbc_sol.q_des(6), kin_wbc_sol.q_des(3), kin_wbc_sol.q_des(4), kin_wbc_sol.q_des(5));
    Eigen::Matrix3d Rdes_base = quat_des_W.toRotationMatrix();
    q_des_.segment<3>(3) = diffRot(Eigen::Matrix3d::Identity(), Rdes_base);
    q_des_.segment(6, na_) = kin_wbc_sol.q_des.segment(7, na_);

    dq_des_ = kin_wbc_sol.out_dq;

    // TODO: call setupQPproblem(robot_wrapper), solve the QP, and extract
    // tau from the solution (x = [Fr; ddxc; delta_ddq; tau]).
    return tqr_cmd;
}

void DynWBC::updateRobotState(RobotWrapper &robot_wrapper, StateEstimator &state_estimator)
{
   // TODO: implement
   robot_wrapper.computeDyn(); // compute robot dynamics terms
   this->contact_state_ = state_estimator.getContactState();
   this->Mq_ = robot_wrapper.dyn_M;
   this->h_nl_ = robot_wrapper.dyn_Non;
   this->dq_ = robot_wrapper.dq;

   // Current configuration in the SAME n_dof_ (18) minimal-coordinate layout
   // as Kp_/Kd_/q_des_: [base_pos_W(3), base_orientation-deviation-from-
   // upright(3), joint_pos(na_)]. The orientation block uses diffRot() the
   // same way KinWBC::updateCurrent() builds task_base_rpy.X_cur -- NOT the
   // raw quaternion -- since Kp_ is a tangent-space (nv_) gain and can't be
   // multiplied against a naive quaternion difference (that's the exact bug
   // documented in doc/DynWBC_review_and_plan.md, just on the other operand).
   q_ = VectorXd::Zero(n_dof_);
   q_.segment<3>(0) = robot_wrapper.q.head<3>(); // base position, world frame
   Eigen::Quaterniond quat_base_W(robot_wrapper.q(6), robot_wrapper.q(3), robot_wrapper.q(4), robot_wrapper.q(5));
   Eigen::Matrix3d Rcur_base = quat_base_W.toRotationMatrix();
   q_.segment<3>(3) = diffRot(Eigen::Matrix3d::Identity(), Rcur_base);
   q_.segment(6, na_) = robot_wrapper.q.segment(7, na_); // joint positions

   return;
}

void DynWBC::setupQPproblem (RobotWrapper &robot_wrapper)
{

  // QP problem is defined as: 
  // J = 1/2x^THx + g^Tx
  // s.t. x_lb <= x <= x_ub
  // lb_A <= Ax <= ub_A
  
  // Check the contact state
  int nContacts = 0;

  switch (this->contact_state_) 
  {
    case LegState::DSt: { // double support — stack both foot Jacobians
      nContacts = 2;
      this->Jc_.resize(2*contact_dim_, this->na_ + 6); // 2*6
      this->Jc_.topRows(contact_dim_)    = robot_wrapper.J_Lfeet_W;
      this->Jc_.bottomRows(contact_dim_) = robot_wrapper.J_Rfeet_W;

      this->dJc_.resize(2*contact_dim_, this->na_ + 6);
      this->dJc_.topRows(contact_dim_)    = robot_wrapper.dJ_Lfeet_W;
      this->dJc_.bottomRows(contact_dim_) = robot_wrapper.dJ_Rfeet_W;

      // Expand Wr_ block-diagonally for 2 feet
      Wr_ = MatrixXd::Zero(2*contact_dim_, 2*contact_dim_); // 12x12
      Wr_.block(0,             0,             contact_dim_, contact_dim_) = Wr_single_;
      Wr_.block(contact_dim_, contact_dim_, contact_dim_, contact_dim_) = Wr_single_;

      // Expand Wc_ block-diagonally for 2 feet, same as Wr_ above
      Wc_ = MatrixXd::Zero(2*contact_dim_, 2*contact_dim_); // 12x12
      Wc_.block(0,             0,             contact_dim_, contact_dim_) = Wc_single_;
      Wc_.block(contact_dim_, contact_dim_, contact_dim_, contact_dim_) = Wc_single_;

      // Expand friction cone matrix U block-diagonally for 2 feet: 5 rows
      // (friction cone) per contact, 6 cols (Fr) per contact -- NOT square.
      U_ = MatrixXd::Zero(2*5, 2*contact_dim_); // 10x12
      U_.block(0, 0,            5, contact_dim_) = U_single_;
      U_.block(5, contact_dim_, 5, contact_dim_) = U_single_;

      // number of variable
      n_Fr_ = 2*contact_dim_;
      n_ddxc_ = 2*contact_dim_;
      QP_numOfvars_ = n_Fr_ + n_ddxc_ + n_dof_ + n_tau_;

      break;
    }
    case LegState::LSt: // left stance — only left foot in contact
      nContacts = 1;
      Jc_ = robot_wrapper.J_Lfeet_W; // 6 x nv
      Wr_ = Wr_single_;
      QP_numOfvars_ = contact_dim_;
      break;
    case LegState::RSt: // right stance — only right foot in contact
      nContacts = 1;
      Jc_ = robot_wrapper.J_Rfeet_W; // 6 x nv
      Wr_ = Wr_single_;
      QP_numOfvars_ = contact_dim_;
      break;
  }

  // Hessian H correspond to optimize variable x = [Fr]
  MatrixXd H = MatrixXd::Zero(QP_numOfvars_, QP_numOfvars_);
  // H = diag(Wr, wc, Wqdd, Wt)
  H.block(0,            0,           2*contact_dim_,      2*contact_dim_) = Wr_;
  H.block(2*contact_dim_, 2*contact_dim_, 2*contact_dim_,      2*contact_dim_) = Wc_;
  H.block(4*contact_dim_, 4*contact_dim_, 6+na_,             6+na_)            = W_ddq_;
  H.block(4*contact_dim_+6+na_, 4*contact_dim_+6+na_, na_, na_) = W_tau_;

  //-------------------------------------------------------------
  // -------------Construct constraints -------------------------
  //-------------------------------------------------------------

  //-------------------------------------------------------------
  // Friction cone matrix (eq. 12): U_ * Fr >= 0, so this constraint's rows
  //-------------------------------------------------------------

  // number U_.rows() (5 per contact), NOT n_Fr_ (Fr's own dimension,
  // 6 per contact) -- those only happen to share a value by coincidence
  // in other blocks; U_'s row/col shape is 5*nContacts x 6*nContacts.
  // for this set lower bound to -inf
  MatrixXd A_fc = MatrixXd::Zero(U_.rows(), QP_numOfvars_);
  A_fc.block(0, 0, U_.rows(), U_.cols()) = U_;
  VectorXd fc_ub = VectorXd::Zero(U_.rows(), 1) + Fz_max_ * VectorXd::Ones(U_.rows(), 1);
  
  //-------------------------------------------------------------
  // Normal reaction force constraint (eq. 13): S_ * Fr <= Fz_max_
  //-------------------------------------------------------------
  MatrixXd A_Fzmax = MatrixXd::Zero(nContacts, QP_numOfvars_);
  S_ = MatrixXd::Zero(nContacts, QP_numOfvars_);
  for (int i = 0; i < nContacts; ++i)
    S_(i, i * contact_dim_ + 2) = 1.0;
  VectorXd Fz_max_ub = Fz_max_ * VectorXd::Ones(nContacts);
  A_Fzmax.block(0, 0, nContacts, QP_numOfvars_) = S_;

  //-------------------------------------------------------------
  // contact acceleration equality ddxc = Jc*ddq + dJc*dq
  // <=> ddxc - Jc*delta_ddq = Jc*ddq_cmd + dJc*dq
  // (substituting eq. 16's ddq = ddq_cmd + delta_ddq) -- an EQUALITY
  // constraint (lbA = ubA = b_ddxc below), n_ddxc_ rows, touching only the
  // ddxc block (coefficient I) and the delta_ddq block (coefficient -Jc_);
  // Fr and tau get zero coefficient, same layout convention as A_fc/A_Fzmax.
  //-------------------------------------------------------------

  // compute ddq_cmd
  ddq_cmd_ = Kp_ * (q_des_ - q_) + Kd_ * (dq_des_ - dq_);

  MatrixXd A_ddxc = MatrixXd::Zero(n_ddxc_, QP_numOfvars_);
  A_ddxc.block(0, n_Fr_,           n_ddxc_, n_ddxc_) = MatrixXd::Identity(n_ddxc_, n_ddxc_);
  A_ddxc.block(0, n_Fr_ + n_ddxc_, n_ddxc_, n_dof_)  = -Jc_;
  VectorXd b_ddxc = Jc_ * ddq_cmd_ + dJc_ * dq_;

  std::cout<<"Number of variables: "<<QP_numOfvars_<<std::endl;
  std::cout<<"Size of Hessian matrix H: "<<H.rows()<<"x"<<H.cols()<<std::endl;
  std::cout<< "Hessian matrix H: "<<std::endl;
  std::cout<<H<<std::endl;

  return;
}
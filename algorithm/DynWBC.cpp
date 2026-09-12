#include "DynWBC.h"
#include "data_type.h"
#include "robot_wrapper.h"
#include "yaml-cpp/yaml.h"
#include <iterator>
#include <vector>

  DynWBC::DynWBC(const std::string &joint_config_yaml_path, const std::string &qp_config_yaml_path,
     RobotWrapper &robot_wrapper,
         bool verbose){
  // joint_config_yaml_path: contain Kp and Kd gain and torque limit
  // qp_config_yaml_path: parameters for QP problem

  // getMaxTorque() returns std::vector<double>; map it into the VectorXd
  const std::vector<double> maxTorqueVec = robot_wrapper.getMaxTorque();
  this->maxTorque_ = Eigen::Map<const VectorXd>(maxTorqueVec.data(), maxTorqueVec.size());
  this->na_ = robot_wrapper.model_na_;

  // load joint name
  YAML::Node joint_config = YAML::LoadFile(joint_config_yaml_path);
  for (const auto &joint_name_cf : joint_config) {
    const std::string name = joint_name_cf.first.as<std::string>();
    joint_name_list_.push_back(name);
  }
  printf("number of joint name in yaml file: %zu \n", joint_name_list_.size());
  printf("model na: %d\n", this->na_);
  if (static_cast<int>(this->joint_name_list_.size()) != this->na_) {
    printf("Configuration file is not valid");
    return;
  }

  // load Kp_j and Kd_j for actuated joint
  this->Kp_j_.resize(na_, na_); this->Kp_j_.setZero();
  this->Kd_j_.resize(na_, na_); this->Kd_j_.setZero();
  for (int i = 0; i < static_cast<int>(this->joint_name_list_.size()); i++) {
    const std::string &jname = joint_name_list_[i];
    double kp = joint_config[jname]["kp"].as<double>();
    double kd = joint_config[jname]["kd"].as<double>();
    Kp_j_(i,i) = kp;
    Kd_j_(i,i) = kd;
  }

  // load Kp_b_ and Kd_b_ for the floating base -- read from qp_config.yaml under 'base_pd_gain'
  // (these are NOT in joint_config / 12dof_joint_config.yaml)
  YAML::Node qp_config = YAML::LoadFile(qp_config_yaml_path);
  const auto& base_pd = qp_config["base_pd_gain"];
  this->Kp_b_.resize(6,6); this->Kp_b_.setZero();
  this->Kd_b_.resize(6,6); this->Kd_b_.setZero();
  this->Kp_b_(0,0) = base_pd["kp_x"].as<double>();
  this->Kp_b_(1,1) = base_pd["kp_y"].as<double>();
  this->Kp_b_(2,2) = base_pd["kp_z"].as<double>();
  this->Kp_b_(3,3) = base_pd["kp_roll"].as<double>();
  this->Kp_b_(4,4) = base_pd["kp_pitch"].as<double>();
  this->Kp_b_(5,5) = base_pd["kp_yaw"].as<double>();
  this->Kd_b_(0,0) = base_pd["kd_x"].as<double>();
  this->Kd_b_(1,1) = base_pd["kd_y"].as<double>();
  this->Kd_b_(2,2) = base_pd["kd_z"].as<double>();
  this->Kd_b_(3,3) = base_pd["kd_roll"].as<double>();
  this->Kd_b_(4,4) = base_pd["kd_pitch"].as<double>();
  this->Kd_b_(5,5) = base_pd["kd_yaw"].as<double>();

  // stack to Kp_ and Kd_ matrix
  this->Kp_.resize(na_ + 6, na_ + 6);
  this->Kd_.resize(na_ + 6, na_ + 6);
  this->Kp_.setZero();
  this->Kd_.setZero();
  this->Kp_.block(0, 0, na_, na_) = this->Kp_j_;
  this->Kp_.block(na_, na_, 6, 6) = this->Kp_b_;
  this->Kd_.block(0, 0, na_, na_) = this->Kd_j_;
  this->Kd_.block(na_, na_, 6, 6) = this->Kd_b_;

  // Load remaining QP parameters from the already-opened qp_config node
  double W_fx = qp_config["contact_weight"]["W_fx"].as<double>();
  double W_fy = qp_config["contact_weight"]["W_fy"].as<double>();
  double W_fz = qp_config["contact_weight"]["W_fz"].as<double>();
  double W_tx = qp_config["contact_weight"]["W_tx"].as<double>();
  double W_ty = qp_config["contact_weight"]["W_ty"].as<double>();
  double W_tz = qp_config["contact_weight"]["W_tz"].as<double>();

  // construct contact weight matrix
  this->W_c_ = MatrixXd::Zero(6, 6);
  this->W_c_(0,0) = W_fx;
  this->W_c_(1,1) = W_fy;
  this->W_c_(2,2) = W_fz;
  this->W_c_(3,3) = W_tx;
  this->W_c_(4,4) = W_ty;
  this->W_c_(5,5) = W_tz;
  // Note: W_c_ is 6x6 (one contact foot); for two feet it will be block-diagonally expanded later

  double mu = qp_config["friction_coefficient"]["muy"].as<double>(); // key is 'muy' in qp_config.yaml
  double Fz_max = qp_config["maximum_normal_contact_force"]["Fz_max"].as<double>();

  // init Matrix
  const int nA = na_; // actuated DoF (legs)
  const int nU = 6;   // underactuated DoF (floating base)
  const int nc = 6;   // spatial dims per contact foot (3 linear + 3 angular)
  const int nContacts = 2; // left + right foot

  Ma_ = MatrixXd::Zero(nA, nA);
  Mu_ = MatrixXd::Zero(nU, nU);
  // Contact Jacobians stacked over both feet: (nc*nContacts) x nU / nA
  Jc_ = MatrixXd::Zero(nc * nContacts, nA);
  // Ja_ = MatrixXd::Zero(nc * nContacts, nA);
  // Ju_ = MatrixXd::Zero(nc * nContacts, nU);
  ha_ = VectorXd::Zero(nA);
  hu_ = VectorXd::Zero(nU);

  // QP decision variable: x = [τ(na) ; fc_left(6) ; fc_right(6)]
  //   QP_nv_des = na + nc*nContacts = 12 + 12 = 24  (DSt worst-case)
  // QP_nc_des: friction pyramid (4) + normal force bound (1) per foot = 5 * 2 = 10
  QP_nv_des = nA + nc * nContacts; // = 24
  QP_nc_des = 5 * nContacts;       // = 10

  // Pre-allocate QP buffers at DSt (max) size; resized in setupQPproblem() as needed
  qp_H_.assign(QP_nv_des * QP_nv_des, 0.0);
  qp_A_.assign(QP_nc_des * QP_nv_des, 0.0);
  qp_g_.assign(QP_nv_des,             0.0);
  qp_lbA_.assign(QP_nc_des,           0.0);
  qp_ubA_.assign(QP_nc_des,           0.0);
  xOpt_iniGuess_.assign(QP_nv_des,    0.0);

  // Simple bounds: τ ∈ [-maxTorque, +maxTorque], fc unconstrained (friction cone via lbA/ubA)
  const double INF = qpOASES::INFTY;
  qp_lb_.assign(QP_nv_des, -INF);
  qp_ub_.assign(QP_nv_des, +INF);
  for (int i = 0; i < nA; ++i) {
    qp_lb_[i] = -maxTorque_(i);
    qp_ub_[i] = +maxTorque_(i);
  }


  if (verbose) {
    printf("===================== QP Parameters =====================\n");
    // print joint name, kp, kp and max torque attribute
    for (int i = 0; i < na_; i++) {
      printf("joint name: %s, kp: %f, kd: %f, max torque: %f\n",
             this->joint_name_list_[i].c_str(), this->Kp_j_(i,i), this->Kd_j_(i,i),
             this->maxTorque_(i));
    }
    // print base link kp and kd matrix
    printf("Kp_b_ matrix:\n");
    std::cout << this->Kp_b_ << std::endl;
    printf("Kd_b_ matrix:\n");
    std::cout << this->Kd_b_ << std::endl;
    printf("Kp_ matrix:\n");
    std::cout << this->Kp_ << std::endl;
    printf("Kd_ matrix:\n");
    std::cout << this->Kd_ << std::endl;
    // print weight matrix
    printf("contact weight matrix:\n");
    std::cout << this->W_c_ << std::endl;
    
    // print friction coefficient and maximum normal contact force
    printf("friction coefficient: %f\n", mu);
    printf("maximum normal contact force: %f\n", Fz_max);
    
  }
}

VectorXd DynWBC::computeTorque(KinWBC& kin_wbc_sol, RobotWrapper &robot_wrapper)
{
    // get q_des and dq_des
    VectorXd tqr_cmd = VectorXd::Zero(na_);
    // TODO: Implement QP here

    return tqr_cmd;
    // this->q_des_ += kin_wbc_sol.out_delta_q
}

void DynWBC::updateRobotState(RobotWrapper &robot_wrapper, StateEstimator &state_estimator)
{
  this->contact_state_ = state_estimator.getContactState();
  // update robot configuration
  q_  = robot_wrapper.getQ();
  dq_ = robot_wrapper.getDq();

  // recompute dynamics terms (M, C, G, ...) from the current q/dq
  robot_wrapper.computeDyn();

  // Extract sub-blocks from the full (nv x nv) mass matrix Mq:
  //   rows/cols [0..5]      -> underactuated floating-base DoF  (Mu_: 6x6)
  //   rows/cols [6..6+na-1] -> actuated joint DoF               (Ma_: na x na)
  // block<R,C>() requires compile-time constants; na_ is a runtime int,
  // so we use the 4-argument dynamic overload block(row, col, rows, cols).
  const MatrixXd& Mq = robot_wrapper.dyn_M;
  this->Mu_ = Mq.block(0, 0, 6,    6   );
  this->Ma_ = Mq.block(6, 6, na_,  na_ );

  // Nonlinear (Coriolis + gravity) vector h = C*dq + G
  const VectorXd& h = robot_wrapper.dyn_Non;
  this->hu_ = h.segment(0,   6  );
  this->ha_ = h.segment(6,   na_);

  // Extract contact Jacobians for both feet from robot_wrapper.
  // robot_wrapper.J_Lfeet_W / J_Rfeet_W are each 6 x nv, where columns are
  // ordered [floating_base(6) | actuated_joints(na_)]:
  //   Ju_: cols [0 .. 5]         -- underactuated / floating-base sub-Jacobian
  //   Ja_: cols [6 .. 6+na_-1]  -- actuated-joint sub-Jacobian
  // Stacked vertically: left foot on top, right foot on bottom.
  const MatrixXd& Jl = robot_wrapper.J_Lfeet_W; // 6 x nv
  const MatrixXd& Jr = robot_wrapper.J_Rfeet_W; // 6 x nv

  Ju_.topRows(6)    = Jl.leftCols(6);      // left  foot, floating-base cols
  Ju_.bottomRows(6) = Jr.leftCols(6);      // right foot, floating-base cols
  Ja_.topRows(6)    = Jl.rightCols(na_);   // left  foot, actuated-joint cols
  Ja_.bottomRows(6) = Jr.rightCols(na_);   // right foot, actuated-joint cols
}

void DynWBC::setupQPproblem (RobotWrapper &robot_wrapper)
{
  // compute acceleration command
  VectorXd ddq_cmd = VectorXd::Zero(na_ + 6);
  ddq_cmd = Kp_ * (q_des_ - q_) + Kd_ * (dq_des_ - dq_);
  const int nv = robot_wrapper.model_nv_;

  // QP problem is defined as: 
  // J = 1/2x^THx + g^Tx
  // s.t. x_lb <= x <= x_ub
  // lb_A <= Ax <= ub_A
  
  switch (this->contact_state_) 
  {
    case LegState::DSt: { // double support — stack both foot Jacobians
      Jc_.resize(2*contact_dim_, nv);
      Jc_.topRows(contact_dim_)    = robot_wrapper.J_Lfeet_W;
      Jc_.bottomRows(contact_dim_) = robot_wrapper.J_Rfeet_W;

      // Expand W_c_ block-diagonally for 2 feet (avoid self-aliasing by using a temp)
      MatrixXd W1 = W_c_;  // save 6x6 base weight
      W_c_ = MatrixXd::Zero(2*contact_dim_, 2*contact_dim_);
      W_c_.block(0,             0,             contact_dim_, contact_dim_) = W1;
      W_c_.block(contact_dim_, contact_dim_, contact_dim_, contact_dim_) = W1;

      QP_nv_des = na_ + 2*contact_dim_;
      break;
    }
    case LegState::LSt: // left stance — only left foot in contact
      Jc_ = robot_wrapper.J_Lfeet_W; // 6 x nv
      QP_nv_des = na_ + contact_dim_;
      break;
    case LegState::RSt: // right stance — only right foot in contact
      Jc_ = robot_wrapper.J_Rfeet_W; // 6 x nv
      QP_nv_des = na_ + contact_dim_;
      break;
  }
  // Extract floating-base / actuated sub-Jacobians from Jc_ (nc x nv, col-split)
  Ju_ = Jc_.leftCols(6);        // cols [0..5]        → floating-base DoF
  Ja_ = Jc_.rightCols(na_);     // cols [6..6+na_-1]  → actuated-joint DoF



  // Hessian H = block_diag(0_{na x na}, W_c_) — zero torque cost, penalise contact wrench
  const int nv_qp = QP_nv_des;
  qp_H_.assign(nv_qp * nv_qp, 0.0);
  // Copy W_c_ into the bottom-right block of qp_H_ (row-major flat storage)
  Eigen::Map<Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      qp_H_.data() + na_ * nv_qp + na_, W_c_.rows(), W_c_.cols()) =
      W_c_.cast<qpOASES::real_t>();

  // gradient g = 0
  qp_g_.assign(nv_qp, 0.0);

  // equality constraints: dynamics equation
  // Matr


  return;
}
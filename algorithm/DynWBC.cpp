#include "DynWBC.h"
#include "robot_wrapper.h"
#include "yaml-cpp/yaml.h"
#include <iterator>
#include <vector>

DynWBC::DynWBC(const std::string &config_yaml_path, RobotWrapper &robot_wrapper,
               bool verbose) {

  this->maxTorque_ =
      robot_wrapper.getMaxTorque(); // robot wrapper load maxtorque from urdf
  this->na_ = robot_wrapper.model_na_;

  // load joint name
  YAML::Node config = YAML::LoadFile(config_yaml_path);
  for (const auto &joint_name_cf : config) {
    const std::string name = joint_name_cf.first.as<std::string>();
    joint_name_list_.push_back(name);
  }
  printf("number of joint name in yaml file: %zu \n", joint_name_list_.size());
  printf("model na: %d\n", this->na_);
  if (static_cast<int>(this->joint_name_list_.size()) != this->na_) {
    printf("Configuration file is not valid");
    return;
  }

  // load Kp and Kd

  for (int i = 0; i < static_cast<int>(this->joint_name_list_.size()); i++) {
    const std::string &jname = joint_name_list_[i];
    double kp = config[jname]["kp"].as<double>();
    double kd = config[jname]["kd"].as<double>();

    this->Kp_j_.push_back(kp);
    this->Kd_j_.push_back(kd);
  }

  // init Matrix
  const int nA = na_; // actuated DoF (legs)
  const int nU = 6;   // underactuated DoF (floating base)
  const int nc = 6;   // spatial dims per contact foot (3 linear + 3 angular)
  const int nContacts = 2; // left + right foot

  Ma_ = MatrixXd::Zero(nA, nA);
  Mu_ = MatrixXd::Zero(nU, nU);
  // Contact Jacobians stacked over both feet: (nc*nContacts) x nU / nA
  Ja_ = MatrixXd::Zero(nc * nContacts, nA);
  Ju_ = MatrixXd::Zero(nc * nContacts, nU);
  ha_ = VectorXd::Zero(nA);
  hu_ = VectorXd::Zero(nU);

  if (verbose) {
    // print joint name, kp, kp and max torque attribute
    for (int i = 0; i < na_; i++) {
      printf("joint name: %s, kp: %f, kd: %f, max torque: %f\n",
             this->joint_name_list_[i].c_str(), this->Kp_j_[i], this->Kd_j_[i],
             this->maxTorque_[i]);
    }
  }
}

VectorXd DynWBC::computeTorque()
{
    // temperory return 0
    return VectorXd::Zero(na_);
}

void DynWBC::updateRobotState(RobotWrapper &robot_wrapper)
{
  // update robot configuration
  q_  = robot_wrapper.q;
  dq_ = robot_wrapper.dq;

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
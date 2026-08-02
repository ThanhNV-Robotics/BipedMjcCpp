#include "MyStateEstimator.h"
#include <Eigen/src/Core/Matrix.h>

// constructor
StateEstimator::StateEstimator(
    double dt,
    bool verbose) // verbose is option to print out filter param or note
{
  // ------------------------
  // Init Measurement vars
  // ------------------------
  this->dt_ = dt;
  joint_pos_mea_.setZero();
  joint_vel_mea_.setZero();
  joint_tor_mea_.setZero();

  imu_acceleration_mea_.setZero();
  imu_angular_vel_mea_.setZero();

  imu_quaternion_ << 0, 0, 0,
      1; // pinocchio style (x,y,z,w), identity Rotation matrix

  ps_.setZero(dimContact_); // foot position w.r.t base frame
  vs_.setZero(dimContact_); // foot velocity w.r.t base frame

  footEndPos_.setZero(2*dimContact_); // foot position vector w.r.t base frame
  footEndVel_.setZero(2*dimContact_); // foot velocity vector w.r.t base frame
  feetHeights_.setZero(this->numContact_);

  // Init Kalman filter
  // state vector:
  // x = [base_pos(3), base linear velocity(3), foot position(dim_contact_)]
  Eigen::Matrix<double, 3, 3> I =
      Eigen::Matrix<double, 3, 3>::Identity(); // Identity matrix

  // Init matrices
  this->A_ = Eigen::MatrixXd::Identity(this->dimState_, this->dimState_);
  this->A_.block<3, 3>(0, 3) = dt * I;

  this->B_ = Eigen::MatrixXd::Zero(this->dimState_, 3);
  this->B_.block<3, 3>(0, 0) = dt * dt * I;
  this->B_.block<3, 3>(3, 0) = dt * I;

  // Init matrix C
  // measurement vector, stacked per contact i = 0..numContact_-1:
  //   rows [0            : dimContact_)          : p_foot_i - p_base   = [ I_3,
  //   0, -I_3 ] (foot position, 6) rows [dimContact_   : 2*dimContact_) :
  //   v_base             = [ 0, I_3,  0   ] (foot velocity, 6) rows
  //   [2*dimContact_ : 2*dimContact_+numContact_) : p_foot_i.z    = [ 0, 0,  H
  //   ] (foot height, 2)
  // dimObserve_ = 2*dimContact_ + numContact_ = 14
  this->C_ = Eigen::MatrixXd::Zero(this->dimObserve_, this->dimState_);

  for (Eigen::Index i = 0; i < this->numContact_; ++i) {
    // position-residual rows for foot i: [ I_3, 0, -I_3 ]
    this->C_.block<3, 3>(3 * i, 0) = Eigen::Matrix3d::Identity();
    this->C_.block<3, 3>(3 * i, 6 + 3 * i) = -Eigen::Matrix3d::Identity();

    // velocity-residual rows for foot i: [ 0, I_3, 0 ]
    this->C_.block<3, 3>(this->dimContact_ + 3 * i, 3) =
        Eigen::Matrix3d::Identity();

    // height-residual row for foot i: H = [0, 0, 1] picking foot_i_pos.z
    this->C_(2 * this->dimContact_ + i, 6 + 3 * i + 2) = 1.0;
  }
  // ----------------------------------------------------------------------
  // construct Q, P, R vector
  // first we construct as Identity matrix and then we will scale it
  this->Q_ = Eigen::MatrixXd::Identity(
      this->dimState_, this->dimState_); // Process noise covariance
  this->P_ = 100 * this->Q_;             // Error covariance matrix
  this->R_ = Eigen::MatrixXd::Identity(
      this->dimObserve_, this->dimObserve_); // Measurement noise covariance

  // ----------------------------------------------------------------------
  // scale Process noise covariance matrix Q_
  this->Q_.block<3, 3>(0, 0) *=
      imuProcessNoisePosition_; // scale position noise
  // scale velocity noise
  this->Q_.block<3, 3>(3, 3) *= imuProcessNoiseVelocity_;
  // scale foot position noise (one 3x3 block per foot)
  for (Eigen::Index i = 0; i < this->numContact_; ++i) {
    this->Q_.block<3, 3>(6 + 3 * i, 6 + 3 * i) *= footProcessNoisePosition_;
  }

  // scale Measurement noise covariance R_
  // scale foot position noise covariance
  this->R_.block(0, 0, this->dimContact_, this->dimContact_) *=
      footSensorNoisePosition_;
  // scale foot velocity noise covariance
  this->R_.block(this->dimContact_, this->dimContact_, this->dimContact_,
                 this->dimContact_) *= footSensorNoiseVelocity_;
  // scale foot height noise covariance
  this->R_.block(2 * this->dimContact_, 2 * this->dimContact_,
                 this->numContact_, this->numContact_) *=
      footHeightSensorNoise_;

  // snapshot the baseline noise covariances: update() scales per-contact
  // relative to these fixed baselines, not to Q_/R_'s own running value
  this->Q0_ = this->Q_;
  this->R0_ = this->R_;

  this->xhat_.setZero(this->dimState_); // init state estimate to zero

  this->feetHeights_.setZero(this->numContact_); // init footh height to [0,0]

  if (verbose) {
    // print out kalman filter paramter for checking
    std::cout << "dim_state = " << this->dimState_ << std::endl;
    std::cout << "A_ =\n" << this->A_ << std::endl;
    std::cout << "B_ =\n" << this->B_ << std::endl;
    std::cout << "C_ =\n" << this->C_ << std::endl;

    std::cout << "Q_ =\n" << this->Q_ << std::endl;
    std::cout << "P_ =\n" << this->P_ << std::endl;
    std::cout << "R_ =\n" << this->R_ << std::endl;
  }
}

void StateEstimator::update(DataBus &Data) {
  // check the leg contact status based on touch sensor values
  // need to revise this more
  if (this->touch_lf >= 50) {
    this->leg_contact[0] = true; // set left leg contact flag to true
  } else {
    this->leg_contact[0] = false; // set left leg contact flag to false
  }
  if (this->touch_rf >= 50) {
    this->leg_contact[1] = true; // set left leg contact flag to true
  } else {
    this->leg_contact[1] = false; // set left leg contact flag to false
  }

  // process the contact flag
  for (int i = 0; i<this->numContact_; i++) {

    int i1 = 3 * i;
    int qIndex = 6 + i1; // start index of i-th feet position in the state
                         // vector x = [p_B, v_B, p_f0, p_f1]
    int rIndex1 = i1; // start index of i-th feet position in the measurement y
                      // = [ps, vs, hf]
    int rIndex2 = i1 + this->dimContact_; // start index of i-th feet velocity
                                          // in the measurement y = [ps, vs, hf]
    int rIndex3 =
        i1 + 2 * this->dimContact_; // start index of i-th feet height in the
                                    // measurement y = [ps, vs, hf]

    const double high_suspect_number = 100;
    bool isContact = this->leg_contact[i];

    // If a foot is in contact, we more believe on the measurement model of the
    // foot position that assume the foot position is unchanged
    // then we use small Q
    // If a foot is not in contact (swing) -> model is not accurate -> increase
    // Q
    // NOTE: scale from the fixed baseline Q0_/R0_, not from Q_/R_'s own
    // current value, otherwise the scaling compounds every call and the
    // covariance diverges after a few steps of a foot staying out of contact
    this->Q_.block(qIndex, qIndex, 3, 3) =
        (isContact ? 1. : high_suspect_number) * Q0_.block(qIndex, qIndex, 3, 3);

    // use small R for feet position if a feet in contact
    this->R_.block(rIndex1, rIndex1, 3, 3) = (isContact ? 1. : high_suspect_number) * R0_.block(rIndex1, rIndex1, 3, 3);
    this->R_.block(rIndex2, rIndex2, 3, 3) = (isContact ? 1. : high_suspect_number) * R0_.block(rIndex2, rIndex2, 3, 3);
    this->R_(rIndex3, rIndex3) =    (isContact ? 1. : high_suspect_number) * R0_(rIndex3, rIndex3);
  
    this->ps_.segment(3 * i, 3) = -this->footEndPos_.segment(3 * i, 3); 
    this->ps_.segment(3 * i, 3)[2] += footRadius_ ;
    this->vs_.segment(3 * i, 3) = -this->footEndVel_.segment(3 * i, 3);  
  }

  Eigen::Matrix<double,3,1> g(0,0,-9.81); //gravity vector
  // compute body acceleration w,r,t global frame

  Eigen::Matrix<double,3,1> accel = Data.eul2Rot(Data.rpy[0], Data.rpy[1], Data.rpy[2]) * this->imu_acceleration_mea_ + g;

  Eigen::Matrix<double,-1,1> y(this->dimObserve_);
  y << ps_, vs_, feetHeights_; // measurement vector


  // -----------------------------------------------
  // Prediction
  //------------------------------------------------
  this->xhat_ = this->A_ * this->xhat_ + this->B_ * accel;
  this->P_ = this->A_ * this->P_ * this->A_.transpose() + this->Q_;

  // -----------------------------------------------
  // Update
  //------------------------------------------------
  Eigen::Matrix<double,-1,-1> ymodel = this->C_ * this->xhat_; // measurement model prediction
  Eigen::Matrix<double,-1,-1> ey = y - ymodel; // measurement error
  Eigen::Matrix<double,-1,-1> S = this->C_ * this->P_ * this->C_.transpose() + this->R_; // measurement covariance
  // Kalman gain
  Eigen::Matrix<double,-1,-1> K = this->P_ * this->C_.transpose() * S.inverse(); // Kalman gain

  // update the state
  this->xhat_ = this->xhat_ + K * ey; // update the state
    
  // update covariance 
  // this->P_ = (I - K * C) P
  Eigen::Matrix<double,-1,-1> I = Eigen::Matrix<double,-1,-1>::Identity(this->dimState_, this->dimState_);
  this->P_ = (I - K * this->C_) * this->P_; // jose's covariance update
  // force to symmetry
  this->P_ = 0.5 * (this->P_ + this->P_.transpose());
}

void StateEstimator::getSensorMeansurement(DataBus &Data) {
  // assign from DataBus
  // joint_pos_mea_ is Eigen::Matrix but Data.joint_pos_cur is
  // std::vector<double> so we have to use Eigen::Map here
  this->joint_pos_mea_ =
      Eigen::Map<const Eigen::Matrix<double, 12, 1>>(Data.joint_pos_cur.data());
  this->joint_vel_mea_ =
      Eigen::Map<const Eigen::Matrix<double, 12, 1>>(Data.joint_vel_cur.data());
  this->joint_tor_mea_ =
      Eigen::Map<const Eigen::Matrix<double, 12, 1>>(Data.joint_tor_cur.data());

  this->imu_acceleration_mea_ = Eigen::Map<Eigen::Vector3d>(Data.baseAcc);
  this->imu_angular_vel_mea_ = Eigen::Map<Eigen::Vector3d>(Data.baseAngVel);
  auto base_quat = Data.eul2quat(Data.rpy[0], Data.rpy[1], Data.rpy[2]);
  this->imu_quaternion_ << base_quat.x(), base_quat.y(), base_quat.z(),
      base_quat.w();

  // foot position and velocity computed from forward kinematics
  this->footEndPos_ << Data.fe_l_pos_L, Data.fe_r_pos_L;
  this->footEndVel_ << Data.fe_l_vel_L, Data.fe_r_vel_L;
}

Eigen::Matrix<double, 4, 1> StateEstimator::getImuquaternion() {
  return this->imu_quaternion_;
}

Eigen::Matrix<double, 12, 1> StateEstimator::get_qj() // return joint position
{
  return this->joint_pos_mea_;
}
Eigen::Matrix<double, 12, 1> StateEstimator::get_qjd() // return joint velocity
{
  return this->joint_vel_mea_;
}

Eigen::Matrix<double, 3, 1> StateEstimator::getBasePosEst() {
  return this->xhat_.segment(0, 3);
}

Eigen::Matrix<double, 3, 1> StateEstimator::getBaseVelEst() {
  return this->xhat_.segment(3, 3);
}

void StateEstimator::setTouch(double touch_lf, double touch_rf) {
  this->touch_lf = touch_lf;
  this->touch_rf = touch_rf;
}

// Eigen::Matrix<double, 6,1> StateEstimator::get_qb() // return base pose
// (position, rpy)
// {
//     return this->joint_vel_mea_;
// }
// Eigen::Matrix<double, 6,1> StateEstimator::get_qbd() // return base linear
// velocity and angular velocity
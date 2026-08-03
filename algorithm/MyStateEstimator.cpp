#include "MyStateEstimator.h"

// constructor
StateEstimator::StateEstimator( double dt, bool verbose) // verbose is option to print out filter param or note
{
  // ------------------------
  // Init Measurement vars
  // ------------------------
  this->dt_ = dt;
  motor_pos_mea_.setZero();
  motor_vel_mea_.setZero();
  motor_tor_mea_.setZero();

  imu_acceleration_mea_.setZero();
  imu_angular_vel_mea_.setZero();

  imu_quaternion_ << 0, 0, 0, 1; // pinocchio style (x,y,z,w), identity Rotation matrix

  ps_.setZero(dimContact_); // foot position w.r.t base frame
  vs_.setZero(dimContact_); // foot velocity w.r.t base frame

  footEndPos_.setZero(2 * dimContact_); // foot position vector w.r.t base frame
  footEndVel_.setZero(2 * dimContact_); // foot velocity vector w.r.t base frame
  feetHeights_.setZero(this->numContact_);

  // Init Kalman filter
  // state vector:
  // x = [base_pos(3), base linear velocity(3), foot position(dim_contact_)]
  Eigen::Matrix<double, 3, 3> I = Eigen::Matrix<double, 3, 3>::Identity(); // Identity matrix

  // Init matrices
  this->A_ = Eigen::MatrixXd::Identity(this->dimState_, this->dimState_);
  this->A_.block<3, 3>(0, 3) = dt * I;
  // accelerometer bias couples into position/velocity the same way measured
  // accel does via B_, just with a minus sign (true accel = measured - bias)
  // and a random-walk (identity) block on its own diagonal, already set
  // above. Baking this into A_ (rather than subtracting bias ad hoc in
  // update()) keeps the P_ = A_*P_*A_^T + Q_ covariance propagation
  // consistent with the state propagation, so the filter actually learns
  // the bias/position/velocity correlation instead of it staying at zero.
  this->A_.block<3, 3>(0, this->dimState_ - 3) = -0.5 * dt * dt * I; // propagate effect of bias to position
  this->A_.block<3, 3>(3, this->dimState_ - 3) = -dt * I; // propagate effect of bias to velocity

  this->B_ = Eigen::MatrixXd::Zero(this->dimState_, 3);
  this->B_.block<3, 3>(0, 0) = 0.5 * dt * dt * I; // standard constant-accel position term; must match A_'s bias-position coupling above
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
    this->C_.block<3, 3>(this->dimContact_ + 3 * i, 3) = Eigen::Matrix3d::Identity();

    // height-residual row for foot i: H = [0, 0, 1] picking foot_i_pos.z
    this->C_(2 * this->dimContact_ + i, 6 + 3 * i + 2) = 1.0;
  }
  // ----------------------------------------------------------------------
  // construct Q, P, R vector
  // first we construct as Identity matrix and then we will scale it
  this->Q_ = Eigen::MatrixXd::Identity(this->dimState_, this->dimState_); // Process noise covariance
  this->P_ = 100 * this->Q_;             // Error covariance matrix
  this->R_ = Eigen::MatrixXd::Identity(this->dimObserve_, this->dimObserve_); // Measurement noise covariance

  // ----------------------------------------------------------------------
  // scale Process noise covariance matrix Q_ (per-axis, matching the
  // original's diagonal KF_Q_* vectors rather than isotropic scalars)
  this->Q_.block<3, 3>(0, 0) = imuProcessNoisePosition_.asDiagonal();
  this->Q_.block<3, 3>(3, 3) = imuProcessNoiseVelocity_.asDiagonal();
  // scale foot position noise (one 3x3 block per foot, isotropic + same both feet)
  for (Eigen::Index i = 0; i < this->numContact_; ++i) {
    this->Q_.block<3, 3>(6 + 3 * i, 6 + 3 * i) *= footProcessNoisePosition_;
  }
  // accelerometer bias drifts slowly, so its random-walk process noise
  // should be small relative to the other states, not the default 1.0
  this->Q_.block<3, 3>(this->dimState_ - 3, this->dimState_ - 3) *= accelBiasProcessNoise_;

  // scale Measurement noise covariance R_ (per-axis; symmetric across feet)
  this->R_.block<3, 3>(0, 0) = footSensorNoisePosition_.asDiagonal();
  this->R_.block<3, 3>(3, 3) = footSensorNoisePosition_.asDiagonal();
  // scale foot velocity noise covariance (same both feet)
  for (Eigen::Index i = 0; i < this->numContact_; ++i) {
    this->R_.block<3, 3>(this->dimContact_ + 3 * i, this->dimContact_ + 3 * i) = footSensorNoiseVelocity_.asDiagonal();
  }
  // scale foot height noise covariance
  this->R_.block(2 * this->dimContact_, 2 * this->dimContact_, this->numContact_, this->numContact_) *=
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

double StateEstimator::contactConfidence(double touchValue) const {
  return 1.0 / (1.0 + std::exp(-(touchValue - contactForceThreshold_) / contactTransitionWidth_));
}

void StateEstimator::update(DataBus &Data) {
  // smooth contact confidence in [0,1] instead of a hard threshold, so Q_/R_
  // scaling ramps continuously through touchdown/liftoff rather than jumping
  double contactConf[2] = {contactConfidence(this->touch_lf), contactConfidence(this->touch_rf)};
  // keep the boolean flag for getContactFlags()/external reporting
  this->contact_flag[0] = contactConf[0] > 0.5;
  this->contact_flag[1] = contactConf[1] > 0.5;

  // footEndPos_/footEndVel_ are expressed in the body frame (see DataBus'
  // fe_l_pos_L/fe_l_vel_L docs), but the KF state and A_/B_ propagate
  // base/foot positions in the world frame, so they must be rotated into
  // world frame before being used as measurements -- otherwise any yaw
  // mixes body-frame x/y into the wrong world axis and the estimate drifts
  // laterally as the robot turns while walking forward.
  Eigen::Matrix3d R_wb = Data.eul2Rot(Data.rpy[0], Data.rpy[1], Data.rpy[2]);

  // process the contact flag
  for (int i = 0; i < this->numContact_; i++) {

    int i1 = 3 * i;
    int qIndex = 6 + i1; // start index of i-th feet position in the state
                         // vector x = [p_B, v_B, p_f0, p_f1]
    int rIndex1 = i1; // start index of i-th feet position in the measurement y
                      // = [ps, vs, hf]
    int rIndex2 = i1 + this->dimContact_; // start index of i-th feet velocity
                                          // in the measurement y = [ps, vs, hf]
    int rIndex3 = i1 + 2 * this->dimContact_; // start index of i-th feet height in the
                                    // measurement y = [ps, vs, hf]

    const double high_suspect_number = 100;
    // linear blend between 1 (full contact confidence) and high_suspect_number
    // (no contact), driven by the smooth sigmoid confidence computed above --
    // replaces the old hard isContact ? 1 : high_suspect_number switch, which
    // made Q_/R_ jump discontinuously at every touchdown/liftoff instant
    double weight = high_suspect_number + contactConf[i] * (1.0 - high_suspect_number);

    // If a foot is in contact, we more believe on the measurement model of the
    // foot position that assume the foot position is unchanged
    // then we use small Q
    // If a foot is not in contact (swing) -> model is not accurate -> increase
    // Q
    // NOTE: scale from the fixed baseline Q0_/R0_, not from Q_/R_'s own
    // current value, otherwise the scaling compounds every call and the
    // covariance diverges after a few steps of a foot staying out of contact
    this->Q_.block(qIndex, qIndex, 3, 3) = weight * Q0_.block(qIndex, qIndex, 3, 3);

    // use small R for feet position if a feet in contact
    this->R_.block(rIndex1, rIndex1, 3, 3) = weight * R0_.block(rIndex1, rIndex1, 3, 3);
    this->R_.block(rIndex2, rIndex2, 3, 3) = weight * R0_.block(rIndex2, rIndex2, 3, 3);
    this->R_(rIndex3, rIndex3) = weight * R0_(rIndex3, rIndex3);

    this->ps_.segment(3 * i, 3) = -R_wb * this->footEndPos_.segment(3 * i, 3);
    this->ps_.segment(3 * i, 3)[2] += footRadius_; // offset in foot height
    this->vs_.segment(3 * i, 3) = -R_wb * this->footEndVel_.segment(3 * i, 3);

    // height-residual target: this was never updated after being zero-inited
    // in the constructor, so it permanently measured "this foot's absolute
    // world z is 0" regardless of contact state -- inconsistent with ps_'s
    // z-component above (which correctly targets footRadius_ relative to the
    // *current* base height), and increasingly disruptive now that the z
    // measurement noise is trusted much more tightly. Blend smoothly (by the
    // same contact confidence used for Q_/R_ above, instead of a hard
    // switch) between "assume the foot is at ground contact height"
    // (confidence->1) and "no new information, just echo the current
    // estimate" (confidence->0, residual becomes ~0 so it can't pull a
    // swinging foot toward the ground).
    // this->feetHeights_[i] = contactConf[i] * footRadius_ + (1.0 - contactConf[i]) * this->xhat_(qIndex + 2);
  }

  Eigen::Matrix<double, 3, 1> g(0, 0, -9.81); // gravity vector
  // compute body acceleration w,r,t global frame

  Eigen::Matrix<double, 3, 1> accel = R_wb * this->imu_acceleration_mea_ + g;

  Eigen::Matrix<double, -1, 1> y(this->dimObserve_);
  y << ps_, vs_, feetHeights_; // measurement vector

  // -----------------------------------------------
  // Prediction
  //------------------------------------------------
  // A_'s bias-coupling blocks (set in the constructor) already subtract the
  // current bias estimate's effect on position/velocity, so this stays the
  // plain linear prediction -- no separate "accel - bias" term needed here.
  this->xhat_ = this->A_ * this->xhat_ + this->B_ * accel;
  this->P_ = this->A_ * this->P_ * this->A_.transpose() + this->Q_;

  // -----------------------------------------------
  // Update
  //------------------------------------------------
  Eigen::Matrix<double, -1, -1> ymodel = this->C_ * this->xhat_; // measurement model prediction
  Eigen::Matrix<double, -1, -1> ey = y - ymodel; // measurement error
  Eigen::Matrix<double, -1, -1> S = this->C_ * this->P_ * this->C_.transpose() + this->R_; // measurement covariance
  // Kalman gain
  Eigen::Matrix<double, -1, -1> K = this->P_ * this->C_.transpose() * S.inverse(); // Kalman gain

  // update the state
  this->xhat_ = this->xhat_ + K * ey; // update the state

  // update covariance
  // this->P_ = (I - K * C) P
  Eigen::Matrix<double, -1, -1> I = Eigen::Matrix<double, -1, -1>::Identity(this->dimState_, this->dimState_);
  this->P_ = (I - K * this->C_) * this->P_; // jose's covariance update
  // force to symmetry
  this->P_ = 0.5 * (this->P_ + this->P_.transpose());
}

void StateEstimator::getSensorMeansurement(DataBus &Data) {
  // assign from DataBus
  // motor_pos_mea_ is Eigen::Matrix but Data.motor_pos_cur is
  // std::vector<double> so we have to use Eigen::Map here.
  // Data.motors_pos_cur/vel_cur/tor_cur are ordered per
  // MJ_Interface::JointName (arms, head, waist, then the 12 leg joints), and
  // this 12-dof estimator only tracks the legs, so we map the last 12 entries.
  this->motor_pos_mea_ = Eigen::Map<const Eigen::Matrix<double, 12, 1>>(
      Data.motors_pos_cur.data() + Data.motors_pos_cur.size() - 12);
  this->motor_vel_mea_ = Eigen::Map<const Eigen::Matrix<double, 12, 1>>(
      Data.motors_vel_cur.data() + Data.motors_vel_cur.size() - 12);
  this->motor_tor_mea_ = Eigen::Map<const Eigen::Matrix<double, 12, 1>>(
      Data.motors_tor_cur.data() + Data.motors_tor_cur.size() - 12);

  this->imu_acceleration_mea_ = Eigen::Map<Eigen::Vector3d>(Data.baseAcc);
  this->imu_angular_vel_mea_ = Eigen::Map<Eigen::Vector3d>(Data.baseAngVel);
  auto base_quat = Data.eul2quat(Data.rpy[0], Data.rpy[1], Data.rpy[2]);
  this->imu_quaternion_ << base_quat.x(), base_quat.y(), base_quat.z(),
      base_quat.w();

  // foot position and velocity computed from forward kinematics
  this->footEndPos_ << Data.fe_l_pos_L, Data.fe_r_pos_L;
  this->footEndVel_ << Data.fe_l_vel_L, Data.fe_r_vel_L;

  this->touch_lf = Data.touch_lf; // get touch sensor meansurement
  this->touch_rf = Data.touch_rf;
}

Eigen::Matrix<double, 4, 1> StateEstimator::getImuquaternion() {
  return this->imu_quaternion_;
}

Eigen::Matrix<double, 12, 1> StateEstimator::get_qj() // return joint position
{
  return this->motor_pos_mea_;
}
Eigen::Matrix<double, 12, 1> StateEstimator::get_qjd() // return joint velocity
{
  return this->motor_vel_mea_;
}

Eigen::Matrix<double, 3, 1> StateEstimator::getBasePosEst() {
  return this->xhat_.segment(0, 3);
}

Eigen::Matrix<double, 3, 1> StateEstimator::getBaseVelEst() {
  return this->xhat_.segment(3, 3);
}

Eigen::Matrix<double, 3, 1> StateEstimator::getAccelBiasEst() {
  return this->xhat_.segment(this->dimState_ - 3, 3);
}

Eigen::Matrix<double, 2, 1> StateEstimator::getTouchSensorValue() {
  return Eigen::Matrix<double, 2, 1>(this->touch_lf, this->touch_rf);
}

std::vector<bool> StateEstimator::getContactFlags() {
  return this->contact_flag;
}
#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <vector>

#include "data_bus.h"
#include "data_type.h"
#include "robot_wrapper.h"

class StateEstimator {
public:
  StateEstimator(double dt, bool verbose); // Constructor, input: sampling time dt

  // robot_wrapper is used to compute foot position/velocity in the base
  // frame via forward kinematics, since RobotSensor only carries raw
  // actuator/IMU sensor data (unlike DataBus, which already has fe_l/r_pos/vel_L precomputed)
  void getSensorMeansurement(const RobotSensor &rb_sensor, RobotWrapper &robot_wrapper);

  void update(const RobotSensor &rb_sensor, RobotWrapper &rb_wrapper);

  Eigen::Matrix<double, 2, 1> getTouchSensorValue(); // [lf-touch, rf-touch]

  // in our biped 12-dof robot, measurement includes:
  // joint states: position, velocity, torque
  // imu: quaternion, local acceleration, local angular velocity

  Eigen::Vector4d getImuquaternion();
  Eigen::VectorXd get_qj();  // return joint position
  Eigen::VectorXd get_qjd(); // return joint velocity
  Eigen::Vector3d getBasePosEst(); // return estimated base position (xhat_[0:3])
  Eigen::Vector3d getBaseVelEst(); // return estimated base linear velocity (xhat_[3:6])
  Eigen::Vector3d getAccelBiasEst(); // return estimated accelerometer bias (xhat_[dimState_-3:dimState_])
  RobotConfiguration getEstimatedRobotConfiguration();
  RobotSpatialVelocity getEstimatedRobotSpatialVelocity();
  
  std::vector<bool> getContactFlags();

  // seed xhat_'s base-position sub-state (e.g. with the sim's known ground-
  // truth initial pose), so the estimate starts where the real robot starts
  // instead of at the origin -- there's no absolute-position sensor for the
  // filter to otherwise correct that from quickly
  void setBasePosEst(const Eigen::Matrix<double, 3, 1> &pos);
  LegState getContactState();

private:
  // smooth (sigmoid) contact confidence in [0,1] from a touch-sensor reading,
  // instead of a hard threshold -- avoids the Q_/R_ scaling jumping
  // discontinuously at every touchdown/liftoff, which otherwise injects a
  // small transient error into the filter on every single footstep
  double contactConfidence(double touchValue) const;

  double dt_;
  const int na_ = 12; // number of actuated joints
  // measurement variables
  Eigen::Vector3d imu_acceleration_mea_, imu_angular_vel_mea_; // w.r.t imu/base's local frame
  Eigen::VectorXd motor_pos_mea_, motor_vel_mea_, motor_tor_mea_;
  Eigen::Vector4d imu_quaternion_; // our robot imu itself can estimate its orientation
                       // w.r.t world frame
  Eigen::Vector3d imu_rpy_; // imu roll, pitch, yaw
  Eigen::Vector3d base_pos_est_, base_linearVel_est_;

  double touch_lf{0}, touch_rf{0}; //touch sensor value (normal reaction force)
  std::vector<bool> contact_flag_ = {true, true};

  // Foot position in the base frame (local coordinate)
  Eigen::VectorXd footEndPos_;
  Eigen::VectorXd footEndVel_;
  Eigen::Matrix<double, -1, 1> feetHeights_; //

  // Kalman Filter for base position and linear velocity estimation
  // Prediction model:
  // x_k+1 = A_*x_k + B_*a_k
  // y_k = C_*x_k + v
  // where a_k is the imu acceleration w.r.t world frame
  // v process noise
  Eigen::Matrix<double, -1, -1> A_, B_, C_, Q_, P_, R_;
  // baseline (unscaled) Q_/R_, captured once after construction so that the
  // per-contact scaling in update() is applied fresh each call instead of
  // compounding on top of the previous step's already-scaled matrix
  Eigen::Matrix<double, -1, -1> Q0_, R0_;
  Eigen::Matrix<double, -1, 1> xhat_, ps_, vs_;
  const int numContact_ = 2; // 2 contact leg
  const int dimContact_ = 3 * this->numContact_; // 6, dim 3 is the feet cartein position

  // observation is the foot position and velocity measurement  w.r.t the base
  // frame and foot height w.r.t world frame consider that the foot is in
  // contact to the ground.
  const int dimObserve_ = 2 * this->dimContact_ + this->numContact_; // 12 + 2 = 14

  // add accelerometer bias to the state vector to estimate
  // x = [base_pos(3), base_linVel(3), foot_L_pos(3), foot_R_pos(3), accel_bias(3)]
  const int dimState_ = 6 + this->dimContact_ + 3; // base pose+vel + 2 foot position + accelerometer bias = 6 +3 +3 +3 = 15

  //----------------------------------------------------------------
  // Configuration parameters for the Kalman Filter
  //----------------------------------------------------------------
  double footRadius_ = 0.05;

  // process noise (Q)
  Eigen::Vector3d imuProcessNoisePosition_ = Eigen::Vector3d(0.02, 0.02, 0.02);
  Eigen::Vector3d imuProcessNoiseVelocity_ = Eigen::Vector3d(0.02, 0.02, 0.02);
  double footProcessNoisePosition_ = 0.002;
  // accel bias drifts slowly, so its random-walk process noise must stay well
  // below imuProcessNoiseVelocity_ -- otherwise the filter treats bias as
  // free to swing every step, and it starts chasing transients instead of
  // converging, feeding noise back into position/velocity via A_'s coupling
  double accelBiasProcessNoise_ = 1e-5;

  // measurement noise (R)
  Eigen::Vector3d footSensorNoisePosition_ = Eigen::Vector3d(0.005, 0.005, 0.005);
  Eigen::Vector3d footSensorNoiseVelocity_ = Eigen::Vector3d(0.1, 0.1, 0.1);
  double footHeightSensorNoise_ = 0.01;

  double contactForceThreshold_ = 50; // touch force (N) treated as the contact/no-contact midpoint
  double contactTransitionWidth_ = 30; // touch force (N) scale of the smoothing around that midpoint
};
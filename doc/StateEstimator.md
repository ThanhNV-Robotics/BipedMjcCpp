# StateEstimator

`algorithm/MyStateEstimator.{h,cpp}` estimates the floating base's world-frame
position and linear velocity for the 12-DOF legs-only biped, by fusing IMU
acceleration/orientation with leg-kinematics-derived foot measurements in a
linear Kalman filter. Base orientation itself is *not* estimated — it is taken
directly from the IMU's own quaternion (`Data.rpy`) and used as a fixed input
to the filter each step.

## State vector

```
x = [ base_pos(3), base_linVel(3), footL_pos(3), footR_pos(3), accelBias(3) ]   (dimState_ = 15)
```

All position/velocity states are in world frame. `accelBias` is the estimated
accelerometer bias (world frame), used to correct the IMU's raw acceleration
reading.

## Process model (prediction)

Standard constant-acceleration model, driven by bias-corrected IMU
acceleration `a_k`:

```
x_k+1 = A_ * x_k + B_ * a_k
```

- `A_` propagates `base_pos += dt*base_vel`, and also feeds the *negative*
  effect of the current bias estimate into position (`-0.5*dt^2`) and velocity
  (`-dt`), since true acceleration = measured − bias. This keeps the
  bias/position/velocity correlation inside the `P_ = A_ P_ A_^T + Q_`
  covariance propagation instead of being patched in ad hoc.
- `B_` maps `a_k` into position (`0.5*dt^2`) and velocity (`dt`), matching the
  bias-coupling terms in `A_`.
- Foot-position states have no dedicated dynamics row (identity in `A_`) —
  they're driven only by process noise `Q_`, i.e. "assumed stationary unless
  the measurement says otherwise."
- `accelBias` also has no dynamics row — modeled as a slow random walk via a
  small `Q_` block.

`a_k` itself is computed each `update()` call as:

```
accel = R_wb * imu_acceleration_mea_ + g        // world frame, gravity added back
```

where `R_wb` is the base→world rotation built from the IMU's roll/pitch/yaw
(`Data.rpy`), i.e. orientation is *not* filtered, only used to rotate vectors.

## Measurement model (update)

Measurement vector `y` (dimObserve_ = 14), stacked per foot `i = 0,1`:

```
y = [ ps (6) , vs (6) , feetHeights (2) ]
```

- `ps_[i] = -R_wb * footEndPos_[i] + [0,0,footRadius_]`: foot position
  relative to base, rotated into world frame, offset by the foot's physical
  radius. Modeled against `C_` rows `[I_3, 0, -I_3]` → predicts
  `base_pos - foot_pos_i`.
- `vs_[i] = -R_wb * footEndVel_[i]`: foot velocity relative to base, rotated
  into world frame. Modeled against `C_` rows `[0, I_3, 0]` → predicts
  `base_vel` (i.e. assumes the foot itself isn't moving, so base velocity is
  the negative of the foot's velocity relative to base).
- `feetHeights_[i]`: absolute world foot height target. **Currently always
  0** — the line that would set it from contact confidence is commented out
  in `update()` — so this residual permanently pulls each foot's estimated
  world-frame z toward 0, tempered by its `R_` weight (see below).

`footEndPos_`/`footEndVel_` come from `Data.fe_l_pos_L`/`fe_r_pos_L` and
`fe_l_vel_L`/`fe_r_vel_L` — forward-kinematics outputs in **body frame**,
populated by `Pin_KinDyn` before `getSensorMeansurement()` is called. They
must be rotated by `R_wb` to be compared in world frame; skipping that
rotation was a previously-fixed bug (see comment in `update()`).

## Contact-adaptive noise (smooth, not thresholded)

Each touch sensor reading is turned into a smooth confidence in [0,1] via a
sigmoid, not a hard on/off threshold:

```
contactConf = 1 / (1 + exp(-(touchValue - contactForceThreshold_) / contactTransitionWidth_))
```

`contact_flag[i] = contactConf[i] > 0.5` is kept only for external callers
(`getContactFlags()`). Internally, `weight` blends continuously between `1`
(full trust, foot in contact) and `100` (`high_suspect_number`, foot swinging):

```
weight = 100 + contactConf[i] * (1 - 100)
```

`weight` scales, **every call, from the fixed baseline `Q0_`/`R0_`** (snapshotted
once after construction) — not from `Q_`/`R_`'s own previous value — so the
scaling doesn't compound step over step:

- `Q_` block for that foot's position state: small `weight` (≈1) when in
  contact → filter trusts the "foot is stationary" process model; large when
  swinging → filter distrusts it and leans on the IMU-driven prediction
  instead.
- `R_` blocks for that foot's position/velocity/height measurement rows:
  same logic, inverted — small `weight` in contact means the measurement is
  trusted; large means it's downweighted.

This avoids injecting a discontinuous transient into the filter at every
touchdown/liftoff instant, which a hard `isContact ? 1 : 100` switch would.

## Update step

Standard linear KF correction:

```
ymodel = C_ * xhat_
ey     = y - ymodel
S      = C_ * P_ * C_^T + R_
K      = P_ * C_^T * S^-1
xhat_  = xhat_ + K * ey
P_     = (I - K*C_) * P_ , then symmetrized: P_ = 0.5*(P_ + P_^T)
```

## Per-tick call sequence (see `tests/test_state_estimator.cpp`)

1. `mj_interface.updateSensorValues()` / `dataBusWrite(RobotState)` — read
   MuJoCo sensors and encoders into `DataBus`.
2. `kinDynSolver.dataBusRead/computeJ_dJ/dataBusWrite` — forward kinematics,
   populates `fe_l_pos_L`/`fe_r_pos_L`/`fe_l_vel_L`/`fe_r_vel_L` (body-frame
   foot pos/vel) that the estimator needs.
3. `state_estimator.getSensorMeansurement(RobotState)` — pulls the latest
   joint pos/vel/torque (last 12 entries of `Data.motors_pos_cur` /
   `motors_vel_cur` / `motors_tor_cur`, i.e. this estimator's own leg-only
   slice), IMU accel/gyro/orientation, foot pos/vel, and touch sensor values
   into the estimator's internal `_mea_` members.
4. `state_estimator.update(RobotState)` — one predict+correct KF step, as
   described above.
5. Getters (`get_qj()`, `get_qjd()`, `getImuquaternion()`, `getBasePosEst()`,
   `getBaseVelEst()`, `getAccelBiasEst()`, `getContactFlags()`,
   `getTouchSensorValue()`) expose the latest measurements/estimate for
   logging or visualization (e.g. driving a second MuJoCo `mjData` purely for
   display, as `test_state_estimator.cpp` does).

## Known simplifications / open items

- **Orientation is not estimated.** `imu_quaternion_`/`R_wb` come straight
  from the IMU's own roll/pitch/yaw every step; the KF only estimates
  position, velocity, foot positions, and accelerometer bias. There is no
  correction of orientation drift.
- **`feetHeights_` is inert.** It's zero-initialized and never updated (the
  update line is commented out in `update()`), so the height measurement
  permanently targets world z = 0 for both feet, regardless of actual
  terrain or contact state.
- **No stance/foot-placement feedback into control.** This estimator only
  *estimates* — nothing in `tests/test_state_estimator.cpp` feeds the
  estimate back into joint commands, so e.g. torso disturbances during
  standing will visibly tilt the legs with the body (expected given the
  test's fixed-target PD control, not an estimator bug).

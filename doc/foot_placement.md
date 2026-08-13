# FootPlacement

`algorithm/foot_placement.{h,cpp}` is the swing-foot trajectory planner,
carried over from the OpenLoong project. Given the current gait phase `phi`
and stance/swing state produced by `GaitScheduler` (see
[`gait_scheduler.md`](gait_scheduler.md)), it computes where the swing foot
should be *right now* — both the final landing target for this step and the
smooth 3D path from lift-off to touchdown. It does not itself decide *when*
to switch legs; it only answers "where" for whichever leg `GaitScheduler`
currently says is swinging.

It is currently unused/unwired in any demo (`demo/float_control.cpp` and
`demo/walk_wbc.cpp` only reference it in a commented-out line). The only
place it's currently exercised in this repo is `tests/test_gait_scheduler.cpp`,
which chains `MyGaitScheduler` → `FootPlacement` every tick against a fixed
(non-physical) nominal standing pose.

## Per-tick call sequence

1. `dataBusRead(robotState)` — pulls in everything needed for the
   calculation: `swingStartPos_W` → `posStart_W`, desired/current velocity
   (`js_vel_des`/`js_omega_des` → `desV_W`/`desWz_W`, `dq` → `curV_W`), `phi`,
   `posHip_W` → `hipPos_W`, `posST_W` → `STPos_W`, `base_pos`, `tSwing`,
   `theta0`, `rpy[2]` → `yawCur`, `base_omega_W(2)` → `omegaZ_W`,
   `width_hips` → `hip_width`, `legState`. Everything except `desV_W`/
   `curV_W`/`desWz_W` originates from `GaitScheduler`'s own `dataBusWrite`,
   not from a joystick or state estimator directly.
2. `getSwingPos()` — computes `posDes_W` (final landing target) and
   `pDesCur[3]` (current-instant desired position along the swing path).
3. `dataBusWrite(robotState)` — writes `pDesCur` into both
   `swingDesPosCur_W` and `swing_fe_pos_des_W`, `posDes_W` into
   `swingDesPosFinal_W`, and a desired swing-foot orientation
   `swing_fe_rpy_des_W = (0, 0, base_rpy_des.z())` (flagged in the source
   itself as a `// WARNING! ThetaZ!` — pitch/roll are hardcoded to 0,
   orientation isn't actually planned, just yaw-aligned to the desired base
   heading).

## `getSwingPos()` calculation

The function does two conceptually separate things: first it computes
**`posDes_W`**, the predicted final landing position for the current swing
phase; then it interpolates **`pDesCur`**, the instantaneous desired
position between `posStart_W` (frozen at swing start — see
`gait_scheduler.md`) and `posDes_W`, parameterized by `phi ∈ [0,1]`.

### 1. Landing target — linear velocity term

```
KP = Rz(yawCur) * diag(kp_vx, kp_vy, 0) * Rz(yawCur)^T
posDes_W = hipPos_W
         - KP * (desV_W - curV_W)
         + 0.5 * tSwing * curV_W
         + curV_W * (1 - phi) * tSwing
```

`hipPos_W` (the swing-side hip position, from `GaitScheduler`) is the base
reference point — a Raibert-style heuristic: place the foot under the hip,
then correct. `diag(kp_vx, kp_vy, 0)` is defined along the robot's own
heading axes and rotated into world frame by `Rz(yawCur)`, so gains act
along forward/lateral, not raw world x/y.

- `-KP*(desV_W - curV_W)`: proportional correction — if current velocity is
  below desired, the foot is placed further back (`-KP*(neg)` → positive),
  which is the standard mechanism for accelerating a walking gait by
  under/overshooting foot placement relative to the hip.
- `0.5*tSwing*curV_W + curV_W*(1-phi)*tSwing` = `curV_W*tSwing*(1.5 - phi)`:
  a velocity lookahead — the foot should land ahead of the hip by roughly
  how far the body will travel during the swing. Because this is
  recomputed every tick from the *current* `phi`, the target isn't
  actually fixed for the whole swing: it continuously contracts from
  `curV_W*tSwing*1.5` (at `phi=0`) toward `curV_W*tSwing*0.5` (as `phi→1`)
  — a receding-horizon re-plan, not a one-shot prediction frozen at
  lift-off.

### 2. Landing target — angular velocity term

```
thetaF = yawCur + theta0 + omegaZ_W*(1-phi)*tSwing + 0.5*omegaZ_W*tSwing + kp_wz*(omegaZ_W - desWz_W)
posDes_W.x += 0.5*hip_width * (cos(thetaF) - cos(yawCur + theta0))
posDes_W.y += 0.5*hip_width * (sin(thetaF) - sin(yawCur + theta0))
```

Mirrors the linear term, but for yaw: `yawCur + theta0` is the swing hip's
*current* world heading (body yaw plus the fixed ±90° hip offset from
`GaitScheduler`, see `gait_scheduler.md`); `thetaF` predicts that heading at
the receding touchdown time, using the same lookahead + proportional-error
structure as above. Since the swing hip is a point at radius `hip_width/2`
rotating about the body's yaw center, the x/y displacement of that point
between "now" and "touchdown" is `0.5*hip_width*(cos(thetaF) - cos(now))`
— i.e., this term accounts for the hip itself sweeping sideways if the body
is turning, on top of the straight-line velocity term above.

### 3. Landing target — clearance offset and height

```
xOff_L = -0.07,  yOff_L = 0.04,  zOff_W = -0.035     // body-frame constants
posDes_W.z = base_pos.z - legLength + zOff_W
posDes_W.x += Rz(yawCur) * xOff_L   (mirrored y sign by stance leg)
posDes_W.y += Rz(yawCur) * (±yOff_L)
```

A small fixed body-frame nudge — `7cm` back, `4cm` toward the body
centerline (sign flips between `LSt`/`RSt` so it's always inward, presumably
to avoid the legs scuffing/colliding at a narrow stance) — rotated into
world frame by current yaw. `posDes_W.z` is set independently of everything
above: landing height is simply "current base height minus nominal leg
length, minus an extra 3.5cm," i.e. slightly short of full leg extension.

### 4. Swing-path interpolation — x/y (cycloid)

```
pDesCur[0] = posStart_W.x + (posDes_W.x - posStart_W.x)/(2π) * (2π·phi - sin(2π·phi))
pDesCur[1] = posStart_W.y + (posDes_W.y - posStart_W.y)/(2π) * (2π·phi - sin(2π·phi))
```

Applied only when `phi < 1.0` (holds its last value otherwise). This is a cycloid
(rolling-circle) interpolation between the frozen `posStart_W` and the
continuously-updated `posDes_W`: its defining property is zero velocity at
both `phi=0` and `phi=1`, so the foot starts and stops moving smoothly with
no velocity discontinuity at lift-off or touchdown. At `phi=1` the bracket
evaluates to exactly `2π`, so the formula lands exactly on `posDes_W` —
the `phi < 1.0` guard is defensive, not load-bearing.

### 5. Swing-path interpolation — z (lift arc + late-stance stretch)

```
if phi >= 0.98:  zStretch -= 0.002   (per call, clamped to zStretch >= -0.05)
else:            zStretch = 0
pDesCur[2] = posStart_W.z + Trajectory(0.2, stepHeight, posDes_W.z - posStart_W.z) + zStretch
```

`Trajectory(phase, hei, len)` (private helper) builds an 8-point Bezier
ease curve with control points `[0,0,0,0,0,1,1,1]` (`Bezier_1D`, see
`math/bezier_1D.{h,cpp}` — a standard Bernstein-polynomial evaluator) to
blend two behaviors:
- `phi < phase` (default `phase=0.2`): height ramps smoothly `0 → hei`
  (`stepHeight`) — the lift-off arc.
- `phi >= phase`: computes an ease-out weight
  `s = Bezier((1.4-phi)/(1.4-phase))` and blends `hei*s + len*(1-s)` —
  height eases from `stepHeight` down toward the total height change `len`
  needed to reach the landing target.

`zStretch` is a small extra downward press applied only in the last 2% of
the swing (`phi >= 0.98`), presumably to guarantee firm ground contact
rather than an air-gap landing from residual timing/velocity slack, capped
at 5cm total.

## Known gaps / notes (as copied into this repo)

- **`b` and `xNow` (lines 38–41) are dead code** — a 4x1 zero vector and a
  `[1, phi, phi², phi³]` row vector are computed but never used anywhere in
  the function. Likely a leftover from a polynomial-fit approach that was
  replaced by the cycloid/Bezier interpolation actually used below.
- **The `phi >= 1.4` branch of `Trajectory()`'s `else` clause
  (`output = len` when `s <= 0`) is effectively unreachable** given
  `GaitScheduler` clamps `phi <= 1`: since all Bezier control points lie in
  `[0,1]`, `s` never goes negative for `phi <= 1`, so the height profile
  approaches but never exactly reaches `len` by the nominal end of swing.
- **`zStretch`'s `-0.002` decrement is per call, not `dt`-scaled** — its
  effective rate in real time depends on the control loop's `dt`, unlike
  every other rate in this file (e.g. `GaitScheduler`'s `dPhi = dt/tSwing`).
- **`STPos_W` is read in `dataBusRead` but never used** in `getSwingPos()`
  — the one place it would have been used (an alternate z-height formula,
  `posDes_W.z = STPos_W.z - 0.04`) is commented out in the source in favor
  of the `base_pos.z - legLength + zOff_W` formula actually used.
- **`swing_fe_rpy_des_W` only sets yaw** (`dataBusWrite`); roll/pitch of the
  swing foot are hardcoded to 0 and flagged in-source as a warning — no
  foot-orientation planning happens here.
- **`kp_vx`/`kp_vy`/`kp_wz` default to `0`** (class member defaults in
  `foot_placement.h`) and are never set by any demo in this repo, so unless
  a caller sets them explicitly, the entire velocity/yaw-rate feedback term
  in §1–2 above is inert and `posDes_W` reduces to just `hipPos_W` plus the
  fixed clearance offset from §3.

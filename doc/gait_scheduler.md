# GaitScheduler

`algorithm/gait_scheduler.{h,cpp}` is a finite-state gait phase scheduler
carried over from the OpenLoong project, unmodified for this legs-only 12-DOF
robot. It tracks which leg is in stance/swing, a normalized swing-phase clock
`phi`, and estimates the vertical foot reaction forces used to decide
touchdown. It does **not** compute any trajectories or torques itself — it
only produces the state (`legState`, `phi`, stance/swing anchor positions,
estimated foot forces) that a foot-placement planner and WBC controller
consume. It is currently unused/unwired in this repo: it's only referenced,
commented out, in `demo/walk_wbc.cpp`.

## States

`DataBus::LegState` (defined in `common/data_bus.h`):
- `LSt` — left leg is stance (support) leg
- `RSt` — right leg is stance leg
- `DSt` — double support (both feet down), also used as the default/idle state

`DataBus::MotionState`:
- `Stand` — standing still, phase clock held at 0
- `Walk` — continuous walking
- `Walk2Stand` — transition from walking back to a stand

`GaitScheduler` doesn't own `motionState` — it's read from `DataBus` each tick
via `dataBusRead()` and only used to decide how `phi` advances and how
`legState` transitions; something outside this class (e.g. a joystick/task
FSM) is expected to set it.

## Per-tick call sequence

1. `dataBusRead(robotState)` — pull in everything needed to (a) estimate
   contact forces and (b) evaluate the state machine: joint torques
   (`motors_tor_cur`), the leg mass matrix/bias (`dyn_M`, `dyn_Non`), leg
   Jacobians and their derivatives (`J_l`/`J_r`, `dJ_l`/`dJ_r`), measured
   vertical GRF (`fL[2]`/`fR[2]`), hip and foot-end world positions, `dq`, and
   `motionState`.
2. `step()` — advances the state machine by one `dt` (see below).
3. `dataBusWrite(robotState)` — pushes the computed `legState`/`legStateNext`,
   `phi`, `tSwing`, `theta0`, `posHip_W`/`posST_W`, `swingStartPos_W`/
   `stanceDesPos_W`, estimated foot wrenches `FL_est`/`FR_est`, and the
   current stance foot's pose (`stance_fe_pos_cur_W`/`stance_fe_rot_cur_W`)
   back into `DataBus` for downstream consumers (foot-placement planner, WBC).

## Foot reaction force estimation

Before touching the state machine, `step()` estimates each foot's vertical
contact wrench from joint torque via the leg's dynamics, not from a force
sensor:

```
tauAll = [0(6); torJoint]                                    // pad out floating-base rows
F_est  = -pinv(J * M^-1 * J^T) * (J * M^-1 * (tauAll - Non) + dJ*dq)
```

for each leg (`J_l`/`dJ_l` → `FLest`, `J_r`/`dJ_r` → `FRest`), using
`pseudoInv_SVD` (`math/useful_math.cpp`) for the pseudo-inverse. This is the
standard "contact force consistent with current joint torques and dynamics"
estimate — `FLest[2]`/`FRest[2]` (vertical component) is what the state
machine below actually thresholds on, not `Fz_L_m`/`Fz_R_m` (the sensor-based
measurement read into `dataBusRead` but otherwise unused in `step()`).

## Phase clock

```
Stand:       dPhi = 0, phi held at 0, isIni=false, enableNextStep=false, stepNumCur=0
Walk:        dPhi = dt / tSwing, enableNextStep=true
Walk2Stand:  dPhi = dt / tSwing   (still advances the clock while braking to a stop)
phi += dPhi, clamped to phi <= 1
```

`tSwing` is a fixed swing-phase duration (constructor arg, e.g. `0.4`s); swing
and stance always take equal time — there's no separate double-support
duration modeled (per the source comment at the top of the `.cpp`).

## Startup

On construction, `legState = DSt`, `legStateNext = firstleg` (hardcoded
`LSt`), `motionState = Stand`. The first time `step()` sees
`!isIni && start_walk`, it latches `isIni = true`, sets `legState = firstleg`,
and seeds `swingStartPos_W`/`stanceStartPos_W` from the current foot
positions for whichever leg starts as swing/stance.

Note (from the integration comment in `demo/walk_wbc.cpp`): while
`motionState == Stand`, `step()` resets `isIni = false` every tick, which
would immediately re-trigger this startup block and force `legState = LSt`
forever *unless* `start_walk` is held `false` until you actually want to
start walking. `start()` (sets `start_walk = true`) is the intended trigger
to begin gait; leave `start_walk = false` while idling in `Stand` so
`legState` stays at its default `DSt`.

## Swing→stance leg-switch condition

While walking (`enableNextStep == true`), the support leg switches once the
*other* foot's estimated vertical force crosses a threshold and the swing
phase is far enough along:

```
LSt -> RSt  when  FRest[2] >= 280  &&  phi >= 0.6
RSt -> LSt  when  FLest[2] >= 280  &&  phi >= 0.6
```

On switch: `legState` flips, `swingStartPos_W`/`stanceStartPos_W` are
re-seeded from the current foot positions, `phi` resets to 0, and
`stepNumCur` increments. (Alternative conditions using a phase-only cutoff,
e.g. `phi >= 0.9`, are present in the source as commented-out alternatives.)

## Walk2Stand touchdown handling

When *not* expecting another step (`enableNextStep == false`, i.e. during
`Walk2Stand`), a high-force touchdown on the currently-swinging foot's
opposite leg force is instead treated as "coming to a stop":

```
if legState==LSt && FRest[2] >= 200:  touchDown=true, stepNumCur++, legState=DSt
if legState==RSt && FLest[2] >= 200:  touchDown=true, stepNumCur++, legState=DSt
```

Note the lower threshold here (200 vs 280 for the mid-walk switch) — this is
a "did we land" check, not a "should we switch support leg" check. Back in
`step()`'s top block, once `motionState==Walk2Stand` and `touchDown` becomes
true, `motionState` is forced back to `Stand` on the next call.

## Per-tick outputs (for the current `legState`)

```
LSt: posHip_W = hip_r_pos_W (opposite hip, used as foot-placement reference)
     posST_W  = fe_l_pos_W  (current stance foot)
     theta0   = -pi/2
     legStateNext = RSt if Walk, DSt if Walk2Stand (else defaults to RSt)

RSt: posHip_W = hip_l_pos_W
     posST_W  = fe_r_pos_W
     theta0   = +pi/2
     legStateNext = LSt if Walk, DSt if Walk2Stand (else defaults to LSt)

DSt (or any other): posHip_W = hip_l_pos_W, posST_W = fe_r_pos_W, theta0 = +pi/2,
     legStateNext = DSt
```

`theta0` is documented elsewhere (`DataBus`) as "offset yaw angle of the swing
leg, w.r.t body frame" — here it's just a fixed ±90° depending on which leg is
swinging, consistent with the two legs being laterally offset from the body
centerline.

## Known gaps (as copied into this repo)

- **`stop()` is declared but not defined.** `gait_scheduler.h` declares
  `void stop();` but `gait_scheduler.cpp` has no implementation — linking
  anything that calls it will fail. Only `start()` exists.
- **Not wired into any demo.** `demo/walk_wbc.cpp` only references
  `GaitScheduler` in comments (construction and the `start_walk` caveat above)
  — nothing currently constructs or steps it in this codebase.
- **`Fz_L_m`/`Fz_R_m` (sensor-measured vertical force) are read but unused**
  in the state machine; all thresholding uses the dynamics-based
  `FLest`/`FRest` instead.
- Carried over verbatim from OpenLoong's full humanoid — the leg-dynamics
  force estimate and Jacobians it reads (`J_l`/`J_r`, `dyn_M`, `dyn_Non`) need
  to come from this project's legs-only `Pin_KinDyn` model (see
  `doc`-pending note on hip reference frame / joint-name adaptation) for the
  force estimate to be meaningful here.

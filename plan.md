# Plan: Migrating OpenLoong-Dyn-Control's WBC (kinematic task priority + dynamic QP) into BipedMjcCpp

## Where we are

- **RobotWrapper / StateEstimator**: done, legs-only 12-DOF adaptation of OpenLoong's `Pin_KinDyn` + its Kalman-filter base-state estimator.
- **KinWBC**: a faithful, hand-specialized port of OpenLoong's generic `PriorityTasks` recursive null-space IK (`algorithm/priority_tasks.{h,cpp}` → our `algorithm/KinWBC.{h,cpp}`). Produces `out_delta_q` / `out_dq` / `out_ddq` per tick, task-list-switched by `MotionState` (STAND / WARM_UP / WALK), same pattern as OpenLoong's `kin_tasks_stand` / `kin_tasks_walk`.
- **CP_Planning / FootPlacement / MyGaitScheduler**: our own closed-form capture-point + Raibert foot-placement + gait-phase scheduler, filling the role OpenLoong's `FootPlacement`/gait logic + (optionally) its single-rigid-body `MPC` play.
- **PVT_Ctr**: joint-space PD + LPF low-level tracking, same role as OpenLoong's `PVT_Ctr` (per-joint `kp`/`kd`/`PVT_LPF_Fc`, same yaml convention).
- **Not done yet**: (1) a couple of small but real gaps in `KinWBC`'s task-priority solver relative to OpenLoong's, and (2) the dynamically-consistent QP that turns `KinWBC`'s kinematic output into actual contact wrenches / joint torques — `DynWBC` today is a **much simpler formulation** than OpenLoong's `WBC_priority::computeTau()`, and this gap is very likely the root cause of most of the QP-failure/divergence symptoms chased throughout this project so far (status-37 solves, `|dq|` spiking into the hundreds, base collapsing). **Revised priority: do the `KinWBC` fixes first** (cheap, low-risk, already-validated) before the larger `DynWBC` QP migration — see "Proposed migration, phased" below.

## `KinWBC` vs. OpenLoong's `PriorityTasks`: how close are we?

**The recursive null-space solve itself is an exact port.** Comparing `PriorityTasks::computeAll()`'s `i>0` (non-top-priority) recursion against `KinWBC::computeWBC_IK()`'s `else` branch term-for-term — `N`, `Jpre`, `delta_q`, `dq`, `ddq` — every line matches, including using the **real** `dq` (not a task's own commanded velocity) in the `dJ*dq` drift term for the dynamically-consistent `ddq` solve. This part of the port is faithful; no changes needed here.

Two real, scoped gaps remain:

1. **No top-of-chain velocity feedforward.** OpenLoong seeds the **first** (highest-priority) task from externally-supplied `des_delta_q`/`des_dq`/`des_ddq`:
   ```
   delta_q = des_delta_q + pseudoInv(Jpre,W)*errX
   dq      = des_dq                                    (used directly, not projected)
   ddq     = des_ddq + dyn_pseudoInv(...)*(ddxcmd − dJ*dq)
   ```
   In `walk_wbc_joystick.cpp` these are set directly from joystick `vx_W`/`vy_W`/`wz_L` each tick — a **direct base-velocity command** that every lower-priority task's correction is built on top of. Our `computeWBC_IK()`'s top task has no such input (`delta_q = pseudoInv(...)*errX` only). Forward-walking velocity currently has to come entirely indirectly, through `CP_Planning`'s CoM-position target. This is functionally equivalent to OpenLoong's own behavior whenever `des_*` happen to be zero (e.g. their own STAND demo), so this gap only bites during `WALK`.
2. **No postural/redundant-joint task.** OpenLoong's `kin_tasks_walk` includes a `RedundantJoints` task (lowest priority, simple joint-space `errX = q_nominal − q_cur`, biases specific joints to a nominal value) with no analog in our task lists. **This was independently diagnosed earlier this session as the root cause of the stance-knee-singularity-collapse bug during WALK** — nothing in our task list biases the redundant leg DOF toward a bent-knee posture, so the null-space of higher-priority tasks can drift the stance leg toward full-extension singularity. A `task_posture` fix along these exact lines was built and **confirmed working**, then reverted per a request to simplify and refocus on phase scheduling first — the diagnosis is still valid and this is a natural place to reintroduce it in the OpenLoong-validated form.

Two smaller, worth-knowing-about design differences (not bugs, just choices OpenLoong made that we didn't, and not necessarily worth copying):

- **Both feet as one task vs. two.** OpenLoong's `static_Contact` stacks both feet into a single 12-dim task at top priority; ours splits `left_contact`/`right_contact` into two sequential 6-dim tasks. For rigid zero-error contact tasks these are close but not identical — processing jointly vs. sequentially carves the null-space slightly differently.
- **CoMXY + base_rpy combined vs. separate.** OpenLoong's `CoMXY_HipRPY` is one 5-dim task (CoM-xy and hip-rpy at *equal* priority); ours runs `task_CoMXY` then `task_base_rpy` sequentially, so CoM-xy strictly dominates orientation in our version.
- Also worth knowing: a chunk of OpenLoong's own `addTask()` calls (`CoMTrack`, standalone `HipRPY`, standalone `Roll_Pitch_Yaw`, `fixedWaist`, `PxPy`, `Roll_Pitch_Yaw_Pz`) are **never actually linked into `taskOrder_stand`/`taskOrder_walk`** — `buildPriority()` only chains the tasks named in the order list, so these are computed every tick but never traversed by `computeAll()`. Their real active chain is 5 tasks each, same count as ours — a literal task-count comparison against OpenLoong overstates how much is "there" on their side.

## The core gap: `DynWBC`'s QP vs. OpenLoong's `WBC_priority`'s QP

### Current `DynWBC` (`algorithm/DynWBC.cpp`, `setupQPproblem()`/`solveWBQP()`)

- **Decision variable**: `x = Fr` only (6 per contact — 6 for single stance, 12 for double support). **Nothing else.**
- **`ddq` is NOT a decision variable at all.** `ddq_cmd_ = kin_wbc_sol.out_ddq` is taken as a hard, fixed input every tick.
- **Dynamics equality constraint** (base/underactuated rows only): `Jc_.transpose().topRows(6) * Fr = Mq_.topRows(6)*ddq_cmd_ + h_nl_.head(6)`. Since `ddq_cmd_` is fixed, this is a **direct linear equation in `Fr` alone** — if `ddq_cmd_` (purely kinematic, no mass-matrix awareness) isn't *exactly* dynamically consistent with the robot's actual inertia distribution that tick (it essentially never is), the equation can demand an unphysical or wildly large `Fr` to satisfy it exactly, or become infeasible outright.
- **Cost**: `Fr^T * Wr_ * Fr` — minimizes raw `Fr` magnitude. No feedforward/nominal baseline to minimize *deviation* from.
- **Net effect**: the QP has **zero slack** to absorb any kinematic/dynamic mismatch. This matches, symptom-for-symptom, the repeated `[DynWBC] QP solve failed! Status code: 37` / `|dq_|` exploding into the hundreds / `base_z` collapsing failures seen throughout this session, especially right as `KinWBC`'s task list switches (STAND→WARM_UP, or a stance-leg transition during WALK) and the kinematic `ddq` briefly has a larger inconsistency.

### OpenLoong's `WBC_priority::computeTau()` (`OpenLoong-Dyn-Control/algorithm/wbc_priority.cpp`)

- **Decision variable**: `x = [delta_r (6, base-acceleration CORRECTION) ; delta_Fr (12, contact-wrench CORRECTION around a feedforward)]` — **18 total**.
- **`ddq` is partially a decision variable**: `ddq_opt = ddq_final_kin + [delta_r; 0...]` — only the floating-base's own 6 rows get corrected; the kinematic joint-space `ddq` is still trusted as-is.
- **Equality constraint** (same structural shape as ours, base rows only): `Sf*dyn_M*St_qpV1*delta_r - Sf*Jfe^T*delta_Fr = -Sf*dyn_M*ddq_final_kin - Sf*dyn_Non + Sf*Jfe^T*Fr_ff`. Because `delta_r` is free (if penalized), this equation is **always satisfiable** — it's an equality in 18 unknowns, not 12.
- **Feedforward contact wrench `Fr_ff`**: a *nominal* contact wrench (e.g. `(0,0,370,0,0,0)` per foot ≈ half body weight in double support, set externally by the walking demo / would come from an MPC layer in the fuller pipeline). `delta_Fr` is the QP-solved *correction* around this baseline, not the raw force itself.
- **Cost**: `delta_r^T * Q2 * delta_r + delta_Fr^T * Q1 * delta_Fr`, with `Q2 = I*2e7` (heavily penalize correcting the kinematic base acceleration — trust `ddq_final_kin` almost completely) and `Q1 = I*2e1` (`delta_Fr` is cheap — let contact force deviate from `Fr_ff` fairly freely to satisfy dynamics). This is the standard "soft equality via heavily-weighted slack" trick: the dynamics constraint is still *effectively* enforced, but the QP always has a feasible answer and a well-conditioned one, because `delta_r` absorbs whatever small inconsistency the kinematic solve left behind, instead of forcing `Fr` alone to compensate for all of it.
- **Inequality constraints**: friction cone + normal-force bounds, but applied to `Fr_ff + delta_Fr` (not `Fr` directly), and **ankle-torque bounds** (`tau_upp/tau_low`, rotated into the foot's own frame) rather than our CoP-box-in-support-polygon formulation — a different but roughly equivalent way of keeping the ZMP inside the sole.

#### Full QP formulation

Setup: let `nv` = generalized-coordinate dimension (floating base 6 + joints), two feet in contact (double support shown; single stance is the same with half the contact dimension).

| Symbol | Meaning | Code variable |
|---|---|---|
| `M` ∈ ℝ^{nv×nv} | mass matrix | `dyn_M` |
| `h` ∈ ℝ^{nv} | Coriolis + gravity | `dyn_Non` |
| `Jc` ∈ ℝ^{12×nv} | stacked contact Jacobians (both feet) | `Jfe` |
| `S_f` = [I₆ 0] ∈ ℝ^{6×nv} | selects the **unactuated** floating-base rows | `Sf` |
| `ddq_kin` ∈ ℝ^{nv} | kinematic acceleration from KinWBC (already computed, fixed) | `ddq_final_kin` |
| `Fr_ff` ∈ ℝ^{12} | feedforward/nominal contact wrench (both feet) | `Fr_ff` |

**Decision variables** (the actual QP unknown, `x`):

```
x = [ δr  ]     δr  ∈ ℝ⁶   (base-acceleration correction)
    [ δFr ]     δFr ∈ ℝ¹²  (contact-wrench correction, both feet)
```

**Recovered quantities** (definitions, not decision variables):

```
ddq = ddq_kin + [δr; 0]        (only the base's own 6 rows get corrected)
Fr  = Fr_ff + δFr              (actual contact wrench used downstream)
```

**Cost** (no linear term, `g = 0`):

```
min   δrᵀ Q₂ δr  +  δFrᵀ Q₁ δFr

Q₂ = 2×10⁷ · I₆        Q₁ = 2×10¹ · I₁₂
```
(a few entries of `Q₁` get an extra ×100 in STAND mode for specific foot-wrench components — a minor per-component retune, not structural.)

**Equality constraint** — floating-base Newton-Euler, base rows only. Physically: `S_f M ddq + S_f h = S_f Jcᵀ Fr`. Substitute `ddq`/`Fr` from above and collect the unknowns `(δr, δFr)` on the left:

```
(S_f M S_fᵀ) δr  −  (S_f Jcᵀ) δFr   =   −S_f(M·ddq_kin + h)  +  S_f Jcᵀ Fr_ff
```

`S_f M S_fᵀ` is just the base-base 6×6 block of the mass matrix. This is **always solvable** — 6 equations in 18 unknowns, with `δr` free to absorb whatever `δFr` can't. (Code: `eigen_qp_A1`/`eqRes`, built via `Sf * dyn_M * St_qpV1` and `Sf * Jfe.transpose()`, where `St_qpV1` is just `S_fᵀ` embedding `δr` back into full coordinate space.)

**Inequality constraints** — bound `Fr = Fr_ff + δFr`, per foot, in the **foot's own local frame** (rotate world-frame `Fr` into the foot frame via `Rfe`, 8 rows per foot):

```
friction cone (4 rows, linearized circular cone, half-angle via μ):
   fx  − (√2/2)μ·fz ≤ 0
  −fx  − (√2/2)μ·fz ≤ 0
   fy  − (√2/2)μ·fz ≤ 0
  −fy  − (√2/2)μ·fz ≤ 0

normal force + ankle torque (4 rows):
   f_z,low ≤ fz ≤ f_z,upp
   τ_low   ≤ (τx,τy,τz) ≤ τ_upp      (motion-state-dependent bounds: STAND vs WALK)
```

During WALK/WALK_TO_STAND, whichever foot is the **swing** foot has all 8 of its rows pinned to ≈0 (`f_upp=f_low=0`, cone lower bound relaxed to `-1e-7` for numerical slack) — i.e. the QP is told that foot currently carries no load at all.

Both the equality and all inequalities are expressed in qpOASES' single `lbA ≤ Ax ≤ ubA` form — there are **no separate simple box bounds** on `x` itself (`init()` is called with `lb=ub=NULL`); everything goes through the general constraint matrix `A`.

**Recovering the final outputs:**

```
ddq_opt = ddq_kin + [δr; 0]
Fr_opt  = Fr_ff + δFr
τ_joint = (M·ddq_opt + h − Jcᵀ·Fr_opt)[joint rows]     ← standard inverse dynamics
```

That last line is identical in spirit to our own `DynWBC::getOptimalJointTorque()` — only `ddq_opt`/`Fr_opt` differ (ours just uses `ddq_cmd_`/`Fr` directly, with no `δr`/`Fr_ff` in the mix).

### Supporting evidence this was the intended direction all along

`config/wbc_config.yaml` **already has** a `qp_cost_weight.contact_acceleration` (`Wc_x..Wc_yaw`) and `qp_cost_weight.delta_joint_acceleration` (`W_ddq_b`, `W_ddq_j`) section — `W_ddq_b`/`W_ddq_j` line up almost exactly with OpenLoong's `Q2`/`Q1` naming (base-ddq-correction weight vs. joint-ddq-correction weight). **`DynWBC.cpp`/`DynWBC.h` never read or use these fields at all** (`grep` confirms zero references) — the yaml scaffolding for the fuller `delta_r`-style formulation already exists, but the implementation was simplified down to Fr-only at some point and the fuller formulation was never (re)built.

## Other gaps worth tracking (smaller, but real)

1. **No MPC layer.** OpenLoong has an optional single-rigid-body `MPC` (`algorithm/mpc.{h,cpp}`, 10-step horizon, qpOASES) that computes a reference CoM/base trajectory and feedforward `Fr_ff` over a preview horizon, used in `walk_mpc_wbc*.cpp`. Our `CP_Planning` is a simpler, single-step closed-form capture-point boundary-value blend — a reasonable MVP analog, but not a drop-in replacement for a real preview-horizon MPC. **Out of scope for the near-term plan below** — much larger lift, revisit only after the phases below are validated.

## Confirmed non-issue: base-link velocity oscillation during WALK is expected, not a bug

While validating `test_WBCKin_walk_joystick.cpp` (the `CP_Planning`-free joystick+Raibert pipeline), observed that `robot_wrapper.pos_base_W`/`vel_base_W`'s forward velocity is **not constant** during steady walking at a fixed commanded speed — it oscillates roughly ±8-9% in sync with the swing-leg phase `φ` (e.g. at `vx_des=0.2 m/s`: `base_vx` ranges ~0.269–0.286 m/s depending on `φ`). Investigated two hypotheses before finding the real explanation:

1. **Ruled out: task-priority leak.** Suspected `task_swing_leg` (priority 1 in `kin_task_walk`, ahead of the base-tracking tasks) could "borrow" floating-base velocity via the minimum-norm pseudo-inverse while achieving its own (non-constant, cycloid-velocity) target. Reordered `kin_task_walk` so `CoMXY`/`base_rpy`/`base_height` outrank `swing_leg` (matching OpenLoong's own `static_Contact → PosRot → SwingLeg → ...` order — kept this reordering, see below). Re-measured: **numbers came back identical to 5 decimal places.** Not the cause.
2. **Confirmed: CoM ≠ base link.** Added a `vel_CoM_W` trace alongside `vel_base_W`. Result: `com_vx` stays essentially perfectly constant (0.19999–0.20001 m/s) for the entire run, while `base_vx` is the one oscillating. `task_CoMXY` tracks the **CoM** (mass-weighted average over every body segment, including the swinging leg), not the base link — these are physically different points. The swing leg's cycloid trajectory has a deliberately non-constant velocity profile over `φ` (smooth/zero at liftoff and touchdown, faster mid-swing, by design — that's why cycloids are used for foot trajectories at all). Holding the CoM's velocity constant while the swing leg's velocity varies means **the base link must absorb a compensating wobble** so the mass-weighted average works out. This is correct kinematic behavior, not a synchronization bug.

Implication: if a perfectly constant *base-link* trajectory is ever wanted (e.g. for camera/sensor stabilization), that's a distinct design choice — switching `task_CoMXY` to track `J_base_W`/`pos_base_W` instead of `Jcom_W`/`pos_CoM_W` would just move the oscillation onto the CoM instead. The only way to reduce it on both ends simultaneously is a smoother swing-trajectory velocity profile, not a task-priority or tracking-target change. Not planned for now — noted here so this doesn't get re-investigated as a bug later.

## Proposed migration, phased

### Phase 1 — `KinWBC` task-priority fixes (do this first: cheap, low-risk, already validated)

#### Phase 1a — Postural/redundant-joint task

- Reintroduce the `task_posture` idea (joint-space, lowest priority, biases the leg toward a nominal bent-knee configuration) into `kin_task_walk` / `kin_task_init_walk`, modeled directly on OpenLoong's `RedundantJoints` task (simple joint-space `errX = q_nominal - q_cur`, moderate `kp`/`kd`, `J` = identity rows on the targeted joint columns).
- Small, well-isolated change (one more `Task` + one more `push_back`) — already validated as a real fix for the knee-singularity-collapse failure mode earlier this session, just reverted for scope reasons at the time, not because it didn't work.

#### Phase 1b — Top-of-chain velocity feedforward

- Add `des_delta_q`/`des_dq`/`des_ddq` parameters to `KinWBC::computeWBC_IK()` (default zero, so existing callers are unaffected), and seed the first task's `delta_q`/`dq`/`ddq` from them exactly as `PriorityTasks::computeAll()` does.
- Feed them from the joystick's `vx_W`/`vy_W`/`wz_L` during `WALK`, matching `walk_wbc_joystick.cpp`.
- **Validate** on `test_WBCKin_walk.cpp`/`test_WarmUp.cpp` — both tasks here are purely kinematic/task-list changes, so they can be fully exercised and validated before touching `DynWBC` at all.

### Phase 2 — Port the `delta_r` + `delta_Fr` QP formulation into `DynWBC`

This directly targets "QP dynamically stabilizing" and is expected to be the single biggest stability win, since it removes the "zero slack" failure mode that's caused most of this session's QP-failure debugging. Larger, riskier change than Phase 1 — do it once `KinWBC`'s output is already as clean as Phase 1 can make it, so any remaining instability during validation is more clearly attributable to the QP itself rather than a kinematic artifact underneath it.

- Add `Fr_ff` (feedforward nominal contact wrench) as a `DynWBC` member, settable per tick (start with a simple static nominal: half body weight per foot in `DSt`, full body weight on the stance foot in `LSt`/`RSt` — matching OpenLoong's hardcoded demo value as a first cut; a real per-tick nominal from gait phase can come later).
- Expand the QP decision variable from `Fr` (6/12-dim) to `[delta_r (6) ; delta_Fr (6/12)]`.
- Change the dynamics equality constraint to solve for `delta_r`/`delta_Fr` around `ddq_cmd_`/`Fr_ff`, per OpenLoong's `eigen_qp_A1`/`eqRes` construction.
- Wire `W_ddq_b`/`W_ddq_j` (already in `wbc_config.yaml`, currently unused) as the new cost weights for `delta_r`/`delta_Fr` respectively — finally giving them a reader.
- Keep the existing friction-cone/Fz_max/CoP constraint machinery, just re-expressed against `Fr_ff + delta_Fr` instead of `Fr` directly.
- Final joint torque stays the same inverse-dynamics formula, just with `ddq_opt = ddq_cmd_ + [delta_r;0...]` and `Fr_opt = Fr_ff + delta_Fr`.
- **Validate incrementally**: `test_DynBaseHeightWBC.cpp` (plain double-support stand) first — confirm it's at least as stable as today, ideally through a wider base-height oscillation range without QP failures. Then move to `test_WarmUp.cpp` (single-stance transitions), which is where today's formulation fails hardest.

### Phase 3 (stretch) — Evaluate the single-rigid-body MPC layer

- Only after Phases 1-2 are validated and walking is reasonably stable kinematically+dynamically. Significant new component (`algorithm/mpc.{h,cpp}` port), own QP, own horizon-prediction model — treat as a separate, later planning pass rather than part of this one.

## Risk / validation notes

- Phase 1a (postural task) and Phase 1b (feedforward) are both pure `KinWBC` task-list/solver changes — no QP, no `DynWBC` involvement — so they're fully testable against the existing kinematic-only test rigs (`test_WBCKin_stand.cpp`/`test_WBCKin_walk.cpp`) before any dynamic-QP risk enters the picture.
- Phase 2 changes `DynWBC`'s QP dimensionality (6/12 → 12/18 variables) and its constraint construction — this is the highest-risk, highest-value change. Test against `test_DynBaseHeightWBC.cpp` (double support only, no leg-state switching) before touching anything that exercises `LSt`/`RSt` single-stance.
- `DynWBC::setupQPproblem()` already reconstructs `qp_prob_` fresh every call sized for that tick's actual variable count — confirmed safe for the `DSt`↔`LSt`/`RSt` size change already (`QProblem` rebuilt each solve, not reused/hot-started across a size change). The same pattern extends cleanly to the larger `delta_r`+`delta_Fr` variable count.
- Keep `DynWBC::solveWBQP()`'s existing `LegState current_leg_state = LegState::DSt` default-parameter signature — no call-site changes needed for Phase 2.

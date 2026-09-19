# DynWBC (QP inverse-dynamics WBC) — review and implementation plan

Scope: `algorithm/DynWBC.{h,cpp}`, exercised by `tests/test_Assembling.cpp`
(applies `tau_wbc` to the robot) and `tests/test_DynWBC.cpp` (computes
`tau_wbc` but does not apply it — see below).

## 1. What it currently does

`DynWBC` takes `KinWBC`'s kinematic solve (`out_delta_q`, `out_dq`, `q_des`)
each tick, turns it into a desired whole-body acceleration `ddq_cmd`, and
solves a QP for `[tau (na); fc_left (6); fc_right (6)]` subject to:

- floating-base Newton-Euler equations (relaxed to a tolerance band)
- actuated-joint Newton-Euler equations (exact equality)
- linearized friction-cone (box, not diamond/pyramid) + normal-force bound
  per foot in contact
- torque bounds (±2.5×`maxTorque`, tightened again by clamping the QP output
  to ±`maxTorque` afterward)

`contact_state_` (DSt/LSt/RSt) picks how many feet's Jacobians go into the
problem, changing `QP_nv_des`/`QP_nc_des` every tick.

## 2. Verified behavior (not just read — actually run)

Built and ran `test_Assembling` headlessly for ~7s past its `t=5.0` WBC
activation point. It does **not** crash, but:

```
[DynWBC] QP solve failed! Status code: 37
```
on every single solve once the WBC path activates. **Status 37 is
`RET_INIT_FAILED_INFEASIBILITY`** (counted from qpOASES's
`MessageHandling.hpp` enum — confirmed against the "(10)/(18)/(20)/(30)/(40)"
inline counters in that file). `computeTorque()`'s success check is a strict
`status == SUCCESSFUL_RETURN`, so every one of these failures falls through
to the all-zero `tqr_cmd` that's returned — meaning **`test_Assembling` has
been commanding zero WBC torque to every joint for the entire ~7s window
tested**, not the QP's intended torque. Worth checking against what you
observed live — if the robot visibly stood there fine, that's more likely
residual stiffness from the pre-`t=5` PD ramp than the WBC actually doing
anything, and is worth re-confirming with the QP failures visible (they
currently only print for the first 3 failures — see §4).

### Root cause, confirmed with tick-by-tick data (§5 step 1 done)

Instrumented `out_delta_q`'s norm/max-abs and `ddq_cmd`'s pre- vs. post-clamp
max-abs every tick and re-ran `test_Assembling`. This **refutes the original
"one-shot IK spike" hypothesis** and replaces it with something more
specific and more useful: a **sustained, worsening runaway feedback loop**,
not a transient.

```
tick=1   out_delta_q norm=0.068 maxabs=0.042        ddq preclamp maxabs=7.15   postclamp=7.15
tick=10  out_delta_q norm=0.065 maxabs=0.041        ddq preclamp maxabs=5.15   postclamp=5.15
tick=30  out_delta_q norm=0.065 maxabs=0.040        ddq preclamp maxabs=21.9   postclamp=14.7
tick=40  out_delta_q norm=0.094 maxabs=0.059        ddq preclamp maxabs=24.7   postclamp=15.0 (saturated)
tick=80  out_delta_q norm=0.206 maxabs=0.142        ddq preclamp maxabs=65.2   postclamp=15.0
tick=120 out_delta_q norm=0.360 maxabs=0.199        ddq preclamp maxabs=82.9   postclamp=15.0
tick=160 out_delta_q norm=0.800 maxabs=0.549        ddq preclamp maxabs=175.3  postclamp=15.0
tick=200 out_delta_q norm=1.773 maxabs=1.176        ddq preclamp maxabs=387.7  postclamp=15.0
```

Key facts this establishes:

- `out_delta_q` starts small and bounded (~0.06–0.07 norm for the first ~30
  ticks) — there is no initial spike. It then grows **monotonically and
  without bound** for the rest of the run: 26× larger by tick 200 than at
  tick 1. This is a live divergence building up over ~0.2s of sim time, not
  an edge-case transient.
- The joint-acceleration clamp (±15 rad/s²) saturates by ~tick 30–40 and
  **stays pinned at exactly 15.0 for the rest of the run** — the pre-clamp
  demand keeps climbing (7 → 25 → 65 → 175 → 388) while the clamped output
  is flat. The controller is commanding maximum allowed acceleration, every
  tick, indefinitely, and still falling further behind.
- The QP itself does **not** start failing immediately — the first
  `Status code: 37` print (of the 3-print budget) lands around tick 174–180,
  well after the clamp had already been saturated for ~140 ticks. So the QP
  was finding *some* feasible torque/force combination for a long stretch
  while already being driven at max clamped acceleration — likely already
  producing near-torque-limit, physically rough motion — before eventually
  running out of feasible solutions once the demanded acceleration grew too
  extreme even for maxed-out torques and friction-limited contact forces.

**Mechanism**: `out_delta_q` is recomputed fresh every tick directly from
KinWBC's null-space IK against the robot's *actual current* configuration —
it isn't integrated/held state. For it to grow tick-over-tick like this, the
robot's real configuration must be drifting further from the task references
each tick, i.e. the closed loop through the physical sim is diverging, not
just the reference computation. The likely trigger:
`ddq_cmd = Kp_ * out_delta_q + Kd_ * (out_dq - dq_W)` treats `out_delta_q` —
which is *already* "the full correction needed to close the task error this
tick" (this is exactly how it's consumed elsewhere in the kinematic-only
pipeline: `robot_wrapper.integrateConfig(stepSize * out_delta_q)` with
`stepSize=1`, i.e. applied at full weight, no additional gain) — as if it
were a raw position error still needing a `Kp` of 200–300 on top. That's
effectively a second, large proportional gain stacked on a term that already
represents a full one-step correction, which is a textbook recipe for
overshoot/instability once it's actually driving a physical system through a
dynamics layer and QP rather than being consumed directly as a kinematic
integration step.

This reframes step 5.3 from "rate-limit an occasional spike" to "the
`Kp_ * out_delta_q` term is structurally double-counting an already-full
correction and needs to not do that" — see the updated step 5.3 below.

## 3. Confirmed bugs / undefined behavior

- **`Ju_`/`Ja_` are never sized in the constructor.** The sizing lines are
  present but commented out:
  ```cpp
  // Ja_ = MatrixXd::Zero(nc * nContacts, nA);
  // Ju_ = MatrixXd::Zero(nc * nContacts, nU);
  ```
  `updateRobotState()` then writes into `Ju_.topRows(6) = ...` /
  `Ja_.bottomRows(6) = ...` — block-assignment into an unsized (0×0)
  `MatrixXd`. Confirmed the build uses `-O3 -DNDEBUG` (Eigen's bounds
  `eigen_assert` compiles to nothing), so this is genuine undefined
  behavior on whichever tick first calls `updateRobotState()` before
  `setupQPproblem()` has ever run and properly resized them via plain `=`.
  `test_DynWBC.cpp` happens to dodge this via an early, one-off
  `dyn_wbc.setupQPproblem(robot_wrapper)` warm-up call before the main loop;
  `test_Assembling.cpp` has no such call, so its first `updateRobotState()`
  is exposed. It didn't visibly crash in the run I did, which is exactly
  what you'd expect from UB that happens not to corrupt anything load-bearing
  this time — not evidence it's safe. **Fix: uncomment the two sizing lines.**

- **`updateRobotState()`'s `Ju_`/`Ja_`/`Ma_`/`Mu_`/`ha_`/`hu_` are dead
  computations.** Grepped every use: `setupQPproblem()` never reads any of
  them — it reads `robot_wrapper.dyn_M`/`dyn_Non`/`J_Lfeet_W`/`J_Rfeet_W`
  directly and rebuilds its own `Jc_`/`Ju_`/`Ja_` from scratch. So roughly
  half of `updateRobotState()`'s body computes values nothing downstream
  uses — independent of the sizing bug above. Decide one of:
  (a) delete the dead computation (simplest), or
  (b) actually wire `setupQPproblem()` to consume the pre-split
      `Ma_`/`Mu_`/`ha_`/`hu_` instead of re-deriving from `dyn_M`/`dyn_Non`
      inline, removing the duplication the other direction.

- **`this->q_des_`/`this->dq_des_` (DynWBC's own members) are written in
  `computeTorque()` and never read anywhere afterward** — in this class or
  outside it (no getter exists). Dead state; either remove or use them for
  something (e.g. they'd be the natural place to compute a joint-space PD
  term for a torque law that blends WBC output with joint tracking — not
  currently done).

- **The constructor's initial `qp_lb_`/`qp_ub_` setup
  (`qp_lb_[i] = -maxTorque_(i)`) is immediately and unconditionally
  overwritten** by `setupQPproblem()`'s `±2.5×maxTorque_(i)` every solve —
  dead code, never actually in effect.

## 4. Design points worth a deliberate decision (not "bugs," but worth flagging)

- **Friction cone is a box, not a diamond/pyramid.** `|fx| ≤ μfz` and
  `|fy| ≤ μfz` independently allow `fx=fy=μfz` simultaneously, which is
  outside the true circular friction cone by a factor of √2 in diagonal
  push directions. Common simplification, but it's an over-approximation
  (unsafe direction), not the usual safe under-approximating pyramid
  (`|fx|+|fy| ≤ μfz`-style 4 planes). Worth deciding if the diagonal-slip
  risk matters for this robot's expected contact forces.

- **Floating-base equality is relaxed to a fixed ±50N/±20N/±60Nm tolerance
  band; the actuated-joint equality is exact.** That asymmetry is
  deliberate per the comment (avoid status-37-style infeasibility on
  transients), but given §2's finding, the actuated side is very possibly
  the actual source of the infeasibility now, and has no slack at all.
  Worth trying a matching (small) tolerance band there too, once the
  `ddq_cmd` root cause is confirmed/fixed — treat as a second lever, not
  a first fix (don't paper over a bad `ddq_cmd` by loosening every
  constraint until *something* is always feasible).

- **`fail_count++ < 3` caps diagnostic printing.** Fine for avoiding log
  spam, but means a long-running failure (like the one verified in §2)
  goes silent after the third tick with no visible signal that WBC torque
  is being silently zeroed the entire time. Consider a periodic
  (e.g. every N seconds) re-print, or a persistent status flag exposed to
  the caller so the test can visibly flag "WBC has been failing for Xs."

## 5. Proposed plan, in order

1. **Instrument, don't guess.** Add a temporary print of
   `kin_wbc_sol.out_delta_q.norm()` (or `.cwiseAbs().maxCoeff()`) right
   before it feeds `ddq_cmd`, alongside the existing failure diagnostics.
   Confirm or rule out the §2 hypothesis with real numbers before changing
   the control law.
2. **Fix the `Ju_`/`Ja_` sizing bug** (uncomment the two lines) —
   unambiguous, low-risk, do regardless of what step 1 finds.
3. **Resolve `ddq_cmd`'s semantics — confirmed root cause, not just a
   hypothesis now (see §2's tick-by-tick data).** `out_delta_q` is already
   a full one-step correction (that's how it's consumed everywhere else in
   the kinematic-only pipeline: applied directly via `integrateConfig`,
   `stepSize=1`, no extra gain). Multiplying it by `Kp_` (200–300) on top is
   what's driving the runaway. Candidate directions, in rough order of
   how directly they address the confirmed mechanism:
   - Drop the `Kp_ * out_delta_q` term's gain to something much smaller
     (effectively near 1, or reconsider `Kp_j_`'s values specifically for
     this use — they were tuned for `12dof_joint_config.yaml`'s original
     joint-PD purpose, not for scaling an already-complete IK correction),
     or don't multiply by `Kp_` at all and instead treat `out_delta_q/dt²`-
     scaled appropriately as the acceleration term directly.
   - Or: don't feed `out_delta_q` through a position-error-style `Kp` at
     all; use `out_dq`/`out_ddq` (velocity/acceleration-level KinWBC
     output) directly once those are actually populated (see next point).
   Either way, re-run with the same instrumentation from §2 afterward and
   confirm `out_delta_q`'s norm stays flat/bounded over a multi-second run
   instead of growing — that's the actual pass/fail signal, not just "QP
   stops printing failures."
4. **Decide whether `KinWBC::out_ddq` should ever be populated.**
   Currently `computeWBC_IK()` only ever sets `out_delta_q`/`out_dq`/`q_des`
   — `out_ddq` stays permanently empty, so
   `if (kin_wbc_sol.out_ddq.size() == nv)` in `computeTorque()` is dead —
   the code always takes the `out_delta_q`-based fallback branch. Either
   have `KinWBC` compute a genuine acceleration-level reference (bigger
   change, more correct) or remove the dead branch and the comment implying
   it's a real code path.
5. **Clean up the dead computation** from §3 (`Ju_`/`Ja_`/`Ma_`/`Mu_`/
   `ha_`/`hu_` in `updateRobotState()`, `q_des_`/`dq_des_`, the constructor's
   overwritten `qp_lb_`/`qp_ub_` init) — do this after 1–4 land, so nothing
   useful gets deleted by mistake while still mid-diagnosis.
6. **Point `test_DynWBC.cpp` at the same "apply `tau_wbc`" pattern
   `test_Assembling.cpp` uses**, or document why it deliberately doesn't
   (currently it computes `tau_wbc`, uses it only to draw contact-force
   arrows, and drives the robot with `pvtCtr`'s pure PD ramp the entire
   time — if that's intentional as a "QP output sanity check without
   risking the robot" test, say so in a comment; if it's just stale, wire
   it up).
7. **Revisit the friction-cone shape and the base/joint tolerance-band
   asymmetry** (§4) once 1–4 are settled and you have a QP that's
   succeeding — tune from a working baseline, not a failing one.

## 6. Test plan

Keep the two test files' roles distinct and say so in each file's own
top comment once this settles:

- **`test_DynWBC.cpp`** — kinematic-only sanity check of `DynWBC`'s QP
  formulation: solve every tick, plot/print whether it succeeds and what
  contact wrench it picks, but don't let a bad QP output destabilize the
  sim (robot stays on the safe `pvtCtr` PD path). Good place to verify
  QP feasibility and contact-force sanity in isolation before trusting it
  to actually drive the robot.
- **`test_Assembling.cpp`** — the real integration test: `tau_wbc` actually
  commands the robot. This is where "does standing/height-tracking actually
  work" gets answered, and where step 1's instrumentation should run first,
  since this is the file that surfaced the infeasibility.

Concrete verification steps, in order:
1. Re-run `test_DynWBC` and `test_Assembling` headlessly with the step-1
   instrumentation; capture `out_delta_q`'s magnitude at the moment of a
   QP failure. Confirms or kills the root-cause hypothesis in §2.
2. After the `Ju_`/`Ja_` fix, re-run both once more — should be a no-op
   for `test_Assembling`'s failure pattern (since, per §3, that Jacobian
   was never actually consumed), but confirms the fix doesn't change
   anything unexpected (it shouldn't).
3. After the `ddq_cmd` fix (step 5.3), check: does `qpOASES::init()` return
   `SUCCESSFUL_RETURN` consistently once the WBC path activates? Log the
   status code every tick (not just first-3) for one full run to confirm,
   rather than eyeballing a few printed lines.
4. Only then, visually confirm in the MuJoCo GUI that `test_Assembling`'s
   base-height tracking (its stated purpose per the commit history) still
   holds with real (non-zero, non-infeasible) WBC torque driving the robot
   — this is the thing the "control base height ok" commit message claims,
   and step 2's finding means it should be re-verified specifically with
   the QP actually succeeding, not assumed from the commit message alone.

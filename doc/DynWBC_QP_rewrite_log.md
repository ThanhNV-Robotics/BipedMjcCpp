# DynWBC QP rewrite — implementation log

Scope: `algorithm/DynWBC.{h,cpp}`, rewriting the QP from scratch to follow
section 3.1.2 ("Dynamic-level Whole-Body Controller") of
`doc/Dynamic Locomotion For Passive_Ankle_Biped Robots And Humanoids Using
Whole_Body Locomotion Control.pdf`, instead of the old `[tau; Fc]`-decision-
variable formulation that predated this rewrite (see
`doc/DynWBC_review_and_plan.md` for that formulation's bug history — this
document supersedes it going forward).

**Current target: double-support (DSt) base-height control only.** LSt/RSt
branches exist in `setupQPproblem()`'s switch but are not the focus yet and
are known-incomplete (see "Known gaps" below) — walking/single-stance is
deliberately deferred until DSt works end-to-end.

## 1. Paper → code mapping

The paper's QP (eqs. 11–18):

```
min  Fr^T Wr Fr + ddxc^T Wc ddxc + delta_ddq^T W_ddq delta_ddq      (11)
s.t. U Fr >= 0                                                      (12)
     S Fr <= Fz_max                                                 (13)
     ddxc = Jc*ddq + dJc*dq                                         (14)
     A*ddq + b + g = (0_6; tau_cmd) + Jc^T*Fr                       (15)
     ddq = ddq_cmd + delta_ddq                                      (16)
     ddq_cmd = ddq_d + Kd*(dq_d - dq) + Kp*(q_d - q)                 (17)
     tau_min <= tau_cmd <= tau_max                                  (18)
```

This implementation adds `tau` itself as a decision variable (with its own
zero-weighted cost term `W_tau_`) rather than treating it as purely derived,
so the decision vector is:

```
x = [ Fr (n_Fr_) ; ddxc (n_ddxc_) ; delta_ddq (n_dof_) ; tau (n_tau_) ]
```

For DSt (2 contacts): `n_Fr_ = n_ddxc_ = 12` (6 per foot: fx,fy,fz,tx,ty,tz),
`n_dof_ = 18` (= `nv_` = 6 floating-base + 12 joints), `n_tau_ = 12`.
`QP_numOfvars_ = 12+12+18+12 = 54`.

## 2. Constructor — yaml-driven config

Two yaml files, read the same way `PVT_Ctr`/`FootPlacement` already do
(`YAML::LoadFile` + `yaml-cpp`'s file-order iteration):

- **`config/12dof_joint_config.yaml`** → per-joint `kp`/`kd` (`Kp_j_`,
  `Kd_j_`, diagonal `na_ x na_`) and `maxTorque` (`tau_lim_`). Joint order is
  read from the file and **cross-checked against
  `robot_wrapper.jointNames_`** (throws `std::runtime_error` on any mismatch)
  since both must match the URDF declaration order — this is the same
  footgun the yaml file's own top-of-file comment warns about.
- **`config/qp_config.yaml`** →
  - `base_pd_gain` → `Kp_b_`/`Kd_b_` (6x6 diag), stacked with `Kp_j_`/`Kd_j_`
    into the full `Kp_`/`Kd_` (`nv_ x nv_` = 18x18) used by eq. 17.
  - `qp_cost_weight.contact_wrench` → `Wr_single_` (6x6 diag, **single**
    contact point).
  - `qp_cost_weight.contact_acceleration` → `Wc_single_` (6x6 diag, single
    contact point).
  - `qp_cost_weight.delta_joint_acceleration` → `W_ddq_` (`nv_ x nv_`,
    fixed-size — `delta_ddq`'s dimension doesn't change with contact count).
  - `friction_coefficient.muy` → `muy_`, used to build `U_single_` (see §3).
  - `maximum_normal_contact_force.Fz_max` → `Fz_max_`.
- `S_tau_` (constant, `nv_ x na_`): `[0(6 x na_); I(na_)]`, encodes eq. 15's
  `(0_6; tau_cmd)` split.
- `W_tau_ = Zero(na_, na_)` — tau is a decision variable but currently
  carries no cost.
- `contact_state_` is explicitly initialized to `LegState::DSt` in the
  constructor (see bug #1 below for why this matters).

## 3. Single-contact building blocks, expanded per contact in `setupQPproblem()`

`Wr_single_`, `Wc_single_`, `U_single_` are all sized for **one** contact
point and built once in the constructor; `setupQPproblem()` block-
diagonally expands them for however many feet are in contact (`nContacts`,
2 for DSt):

- `Wr_`, `Wc_`: `2*contact_dim_ x 2*contact_dim_` (12x12), each foot's
  6x6 single-contact block placed on the diagonal.
- `U_single_` (friction cone, eq. 12, box/pyramid approximation on the
  linear force components only — `fz>=0`, `|fx|<=mu*fz`, `|fy|<=mu*fz`;
  moments unconstrained): **5 rows x 6 cols**, NOT square. Expanded `U_` is
  therefore `2*5 x 2*contact_dim_` = **10x12**, not 12x12 — this shape
  mismatch (rows != cols) was the source of bug #3 below.

## 4. `q_`/`q_des_`/`dq_des_` — minimal-coordinate convention for eq. 17

`Kp_`/`Kd_` are tangent-space gains (`nv_ x nv_` = 18x18), but Pinocchio's
`q` lives in configuration space (`nq_` = 19: 3 base-pos + 4 quaternion + 12
joints) — naively subtracting two `q`s is both the wrong size *and*
meaningless for the quaternion block (component-wise quaternion difference
isn't a rotation error). This is the same class of mistake the old
`Kp_ * out_delta_q` bug was (`doc/DynWBC_review_and_plan.md`), just on the
other operand.

Fix: `q_`/`q_des_` are stored as `n_dof_` (18)-dim **minimal-coordinate**
vectors — `[base_pos_W(3), base-orientation-deviation-from-upright(3),
joint_pos(na_)]` — using `diffRot(Identity, R)` for the orientation block,
**the exact same convention `KinWBC::updateCurrent()` already uses** for
`task_base_rpy.X_cur`/`X_des`. No new cross-class API was needed (originally
considered adding a `RobotWrapper::difference()` wrapping
`pin::difference`, but reusing `diffRot()` matched existing precedent and
needed no new plumbing).

- `updateRobotState()` populates `q_` from `robot_wrapper.q`.
- `computeTorque()` populates `q_des_` from `kin_wbc_sol.q_des` (nq_-dim,
  converted the same way) and `dq_des_` from `kin_wbc_sol.out_dq` (already
  tangent-space, no conversion needed).
- `setupQPproblem()` computes `ddq_cmd_ = Kp_*(q_des_-q_) + Kd_*(dq_des_-dq_)`
  (eq. 17) directly — this needed no changes once both operands were
  consistently 18-dim.

## 5. `setupQPproblem()` state so far

Per-tick, for DSt:
- Builds `Jc_`/`dJc_` (stacked L/R foot Jacobians, 12 x 18).
- Expands `Wr_`, `Wc_`, `U_` block-diagonally (§3).
- Builds Hessian `H` (54x54) = `blkdiag(Wr_, Wc_, W_ddq_, W_tau_)`.
- Builds `A_fc` (10x54): friction cone rows, `U_` in the `Fr` columns.
- Builds `A_Fzmax`/`S_` (2x54): one row per contact selecting that
  contact's fz (global column `i*contact_dim_ + 2`), NOT one row per `Fr`
  component (bug #4 below).
- Computes `ddq_cmd_` (eq. 17).
- Builds `A_ddxc` (12x54): eq. 14's linearized equality (`I` on the `ddxc`
  block, `-Jc_` on the `delta_ddq` block), RHS `b_ddxc = Jc_*ddq_cmd_ +
  dJc_*dq_`.

## 6. Known gaps / not yet implemented

- **Eq. 15 (floating-base + actuated dynamics equality) not built yet** —
  no `A_dyn`/`b_dyn` block using `Mq_`, `h_nl_`, `S_tau_`, `Jc_^T`.
- **Eq. 18 (torque bounds) not wired** — `tau_lim_` exists but isn't yet
  applied as a simple bound on the `tau` block of `x`.
- **No stacking step yet** — `A_fc`, `A_Fzmax`, `A_ddxc` (and the still-
  missing dynamics block) are each built as their own `QP_numOfvars_`-wide
  matrix but never vertically concatenated into the single `qp_A_` (+
  `qp_lbA_`/`qp_ubA_`) qpOASES actually needs. Plan (per earlier
  discussion): stack rows in the same order they're built; friction-cone
  rows get `lbA=0, ubA=+inf` (currently `fc_ub` wrongly reuses `Fz_max_` —
  needs revisiting once stacking happens); Fz_max rows get `lbA=-inf,
  ubA=Fz_max_ub`; equality rows (`A_ddxc`, future dynamics) get
  `lbA=ubA=b`.
- **`qp_lb_`/`qp_ub_` (simple bounds on `x` itself) not set** — `tau_lim_`
  belongs here, not in `qp_A_`/`qp_lbA_`/`qp_ubA_`.
- **`qpOASES::QProblem` (`qp_prob_`) never constructed/`init()`/`hotstart()`
  called** — `setupQPproblem()` only assembles matrices and prints; nothing
  is actually solved yet.
- **`computeTorque()` doesn't call `setupQPproblem()`** — it populates
  `q_des_`/`dq_des_` and returns an all-zero torque; wiring the two
  together (build → solve → extract `tau` from `x`) is still TODO.
- **LSt/RSt branches in `setupQPproblem()`'s switch are stale/incomplete**
  (still reference the old single-block `Wr_ = Wr_single_` pattern, don't
  set `n_Fr_`/`n_ddxc_`, and `H`'s block offsets below the switch are
  hardcoded for the DSt 2-contact layout) — not a current concern since
  `contact_state_` defaults to `DSt` and the target is double-support only,
  but will need a real rewrite before single-stance/walking is attempted.

## 7. Bugs found and fixed during this rewrite

1. **`contact_state_` uninitialized** (`LegState` enum member with no
   default) — `setupQPproblem()` called before `updateRobotState()` ever
   ran would read garbage that happened to alias `LSt`/`RSt`'s bit pattern.
   Fixed: constructor now sets `contact_state_ = LegState::DSt;`.
2. **`H`'s block-assembly hardcoded 2-contact offsets regardless of which
   switch branch ran** — in the (now largely moot, see §6) LSt/RSt case
   `QP_numOfvars_` was only 6, but `H.block(0,0,12,12)=Wr_` etc. requested
   blocks far outside `H`'s actual 6x6 buffer. With `eigen_assert` compiled
   out (`-O3 -DNDEBUG`), this was a silent out-of-bounds write — the
   `malloc(): unsorted double linked list corrupted` crash observed while
   testing. Root cause was bug #1 (garbage `contact_state_` landing in the
   LSt/RSt branch); defaulting to `DSt` avoids hitting this specific path
   for now, but the LSt/RSt branches remain unfixed (§6).
3. **`U_`'s DSt expansion sized `2*contact_dim_ x 2*contact_dim_` (12x12,
   square)** instead of `2*5 x 2*contact_dim_` (10x12) — `U_single_` is
   5 rows x 6 cols (friction cone: 5 inequalities per contact, only acting
   on the 6-dim force/moment wrench), not square, so the block-diagonal
   expansion isn't square either. Also used `.topRows(5)`/`.bottomRows(5)`
   (full-width) instead of `.block()` at the correct column offset, which
   wouldn't have been block-diagonal even at the right size. Fixed with
   explicit `.block(row, col, 5, contact_dim_)` placement.
4. **Same shape bug independently in `Wc_`** (never block-expanded for DSt
   at all — stayed at the constructor's single-contact 6x6 while `H`'s
   Wc_ block expected 12x12) — fixed by giving `Wc_` a `Wc_single_` +
   expansion step mirroring `Wr_`/`Wr_single_`.
5. **`A_fc` (friction-cone constraint row block) sized `n_Fr_ x
   QP_numOfvars_`** (12 rows) instead of `U_.rows() x QP_numOfvars_` (10
   rows) — `n_Fr_` (`Fr`'s own dimension, 6/contact) and `U_.rows()` (5/
   contact) aren't the same quantity; they only happened to share a value
   in other blocks. Fixed to use `U_.rows()`/`U_.cols()` explicitly.
6. **`A_Fzmax` (normal-force upper-bound constraint) built with `n_Fr_`
   (12) rows, touching only column 2** — this asserted the SAME constraint
   (left foot's fz) twelve times and never constrained the right foot's fz
   (column 8) at all. The constraint needs **one row per contact**, not one
   row per `Fr` component. Fixed: `S_`/`A_Fzmax` now `nContacts x
   QP_numOfvars_`, row `i` selecting global column `i*contact_dim_ + 2`.
7. **`q_`/`q_des_` dimensional mismatch** — see §4.

## 8. Test-file changes made alongside this rewrite

`tests/test_DynWBC.cpp`:
- Fixed a double-command bug: a joint-space ramp-to-`qIniDes` block ran
  *unconditionally* every tick, after the KinWBC-driven PD block had already
  called `mj_interface.setMotorsTorque()` — so the KinWBC command was
  immediately clobbered. Restructured into mutually-exclusive `if
  (joystick_control_start) {...} else {ramp}` branches, mirroring the
  already-working pattern in `test_Assembling.cpp` ("control base height
  ok" commit).
- Added a repeating base-height oscillation demo (0.75m ↔ 0.80m, 3s ramps
  each way via `joyStick.setPzRef()`, retriggered each time
  `joyStick.PzLGen.isReachDes()` fires) to exercise the base-height control
  path continuously instead of a single one-shot move.

## 9. Next steps (in rough order)

1. Build eq. 15's dynamics-equality block (`A_dyn`/`b_dyn` from `Mq_`,
   `h_nl_`, `S_tau_`, `Jc_^T`).
2. Wire `tau_lim_` into `qp_lb_`/`qp_ub_` (eq. 18, simple bounds — not a
   constraint-matrix row).
3. Stack `A_fc`/`A_Fzmax`/`A_ddxc`/`A_dyn` into `qp_A_` +
   corresponding `qp_lbA_`/`qp_ubA_`; fix `fc_ub`'s placeholder value
   (should be `lbA=0, ubA=+inf`, not `Fz_max_`).
4. Construct/`init()`/`hotstart()` `qp_prob_` (`qpOASES::QProblem`) and
   extract `tau` from the solved `x`.
5. Wire `computeTorque()` to actually call `setupQPproblem()` + solve,
   instead of returning zero.
6. Re-run `test_DynWBC`/`test_Assembling` with DynWBC actually driving
   torque (currently PD/KinWBC drives the robot; DynWBC is comparison-only
   where exercised at all) and verify base-height tracking in DSt.
7. Revisit LSt/RSt branches for single-stance/walking, once DSt is solid.

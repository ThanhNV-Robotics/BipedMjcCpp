# WBC_priority: Whole-Body Control Framework

`WBC_priority` (`wbc_priority.h`/`wbc_priority.cpp`) turns a set of prioritized
Cartesian/joint tasks into joint torques. It works in two stages:

1. **Kinematic task-priority stage** (`computeDdq()`) — combines several
   prioritized tasks (base orientation, base height, swing foot tracking,
   stance foot fixed, etc.) into a single desired joint acceleration `ddq`
   using null-space projection, so that lower-priority tasks never disturb
   higher-priority ones.
2. **Dynamic QP correction stage** (`computeTau()`) — takes that kinematic
   `ddq` (and an MPC-provided reference contact force `Fr`) and solves a small
   QP for the minimal correction `(delta_ddq, delta_Fr)` needed so the result
   actually satisfies rigid-body dynamics and physical contact constraints
   (friction cone, unilateral force, foot moment/CoP limits). Final joint
   torques are recovered from inverse dynamics.

This mirrors the two-stage "kinematic WBC + dynamic QP" design common in
legged-robot WBC (e.g. MIT Cheetah-style whole body control), split here into
`PriorityTasks` (stage 1 math) and `WBC_priority` (stage 1 task setup + stage 2 QP).

---

## Stage 1: `PriorityTasks` — null-space task hierarchy

Defined in `priority_tasks.h`/`priority_tasks.cpp`. Each `Task` bundles:

- `J` — task Jacobian (rows = task dimension, cols = `model_nv`)
- `Kp`, `Kd` — proportional/derivative gains (task-space PD)
- `pError`, `vError` — desired-minus-current position/velocity error
- `ddx` — desired task-space feedforward acceleration
- `weight` — per-row task weighting (0 disables a row without removing it)
- `dim` — number of active rows

Tasks are added, highest priority first, via `addTask()`, then combined by
`computeAll()`.

### Null-space projection math

For each task `i` in priority order, its Jacobian is projected into the
null space of the **stacked, higher-priority** Jacobian `Jpre`:

```
Jpre_i   = [J_0; J_1; ...; J_{i-1}]              (stack of all higher-priority tasks)
N_i      = I - pinv(Jpre_i) * Jpre_i             (null-space projector of Jpre_i)
J_proj_i = J_i * N_i
```

The desired joint acceleration is built up incrementally:

```
ddq_0 = pinv(J_0) * (ddx_0 + Kp_0*pError_0 + Kd_0*vError_0)
ddq_i = ddq_{i-1} + pinv(J_proj_i) * ( (ddx_i + Kp_i*pError_i + Kd_i*vError_i) - J_i*ddq_{i-1} )
```

i.e. each task only corrects the *residual* task-space error left over after
all higher-priority corrections are applied, and its correction is
constrained to the null space of everything above it — so it can never
undo a higher-priority task's result.

Two pseudo-inverse flavors are used:
- `pseudoInv_right_weighted` — plain (optionally weighted) Moore-Penrose
  pseudo-inverse, used for the position/velocity-level null-space
  projections above.
- `dyn_pseudoInv` — **mass-matrix-weighted** pseudo-inverse
  (`J# = M⁻¹Jᵀ(JM⁻¹Jᵀ)⁻¹`), used where the projection should respect the
  robot's inertia rather than pure kinematics (dynamically-consistent
  null-space projection).

`computeAll()` returns the final `ddq` after folding in every task in the
hierarchy.

---

## Stage 1 setup: `WBC_priority::computeDdq()` — task hierarchies

The constructor builds two `PriorityTasks` instances, one per gait phase,
each with its own task list and priority order:

- `kin_tasks_walk` — used while walking
- `kin_tasks_stand` — used while standing

`computeDdq()` fills in each task's `J`, `pError`, `vError`, `ddx`, gains for
the current tick and calls `computeAll()`.

### Walk hierarchy (priority high → low), 5 of 7 defined tasks active

1. Stance-foot fixed (no slip) — position + orientation, 6 rows
2. Base orientation (roll/pitch upright, yaw free or tracked)
3. Swing-foot tracking (position + orientation, 6 rows) — driven by
   `FootPlacement`'s trajectory
4. Base height/position tracking
5. Joint-space posture regularization (nullspace "resting" pose)

Two additional task definitions exist in the source (arm/waist related) but
are **not added to the active hierarchy** for the walk case — they're dead
code carried over from OpenLoong's full-humanoid list.

### Stand hierarchy (priority high → low), 5 of 9 defined tasks active

Similar structure but with **both feet fixed** (double support) instead of
one stance + one swing foot, plus base orientation/height/posture tasks.
Four additional task definitions (again arm/waist-related) exist but are
unused for this project.

---

## Stage 2: `computeTau()` — QP dynamic correction

Given the kinematic `ddq` from stage 1 and a reference contact-force vector
`Fr` (nominally from an MPC/force-distribution module, or zero if unused),
`computeTau()` solves a QP for the smallest correction that makes the result
dynamically and physically consistent:

**Decision variables** (`QP_nv = 18`):
- `delta_ddq` (6) — correction to the floating-base (6-DOF) acceleration only;
  the joint-acceleration part of `ddq` is trusted as-is from stage 1
- `delta_Fr` (12) — correction to the 6-DOF contact wrench at each foot
  (3 force + 3 moment, left + right)

**Cost**: minimize `‖delta_Fr‖²` and `‖delta_ddq‖²` (separately weighted,
`Q1`/`Q2`) — i.e. stay as close as possible to the kinematically/MPC-desired
solution while satisfying the constraints below.

**Equality constraints** (6 rows) — the floating-base rows of the Newton-Euler
equation of motion must hold exactly:

```
S_f * (M*ddq + Non) = S_f * Jc^T * Fr
```

where `S_f` selects the 6 floating-base rows, `ddq = ddq_cmd + [delta_ddq; 0]`,
and `Fr = Fr_ref + delta_Fr`. (The joint rows are not constrained here — they
are satisfied afterward via the torque recovery below, since joint torques
are actuated and can supply whatever residual is needed there.)

**Inequality constraints** (16 rows, 8 per foot) — physical limits on the
corrected contact wrench `Fr`:
- Linearized (pyramidal) friction cone: 4 rows per foot approximating the
  circular Coulomb cone `√(Fx²+Fy²) ≤ μ*Fz` with 4 linear half-planes,
  scaled by `cos(45°) = √2/2 * μ` so the pyramid inscribes the circle
- Unilateral normal force bound: `Fz ≥ 0` (foot can only push, not pull)
- Foot moment/CoP bounds: keep the contact wrench's moment within the
  physical foot-sole support region (prevents the foot from "tipping")

During swing (one foot not in contact), that foot's force/moment rows are
overridden to force `Fr = 0` for that foot rather than leaving it
QP-constrained as if in contact.

The QP is solved with `qpOASES::QProblem` (`QP_nv = 18` variables,
`QP_nc = 22` constraint rows = 6 equality + 16 inequality).

### Final torque recovery

Once `delta_ddq`/`delta_Fr` come back from the QP:

```
ddq_opt = ddq_cmd + [delta_ddq; 0]
Fr_opt  = Fr_ref + delta_Fr
tau     = M * ddq_opt + Non - Jfe^T * Fr_opt
```

`tau`'s floating-base rows are (by construction of the equality constraint)
~zero and discarded; the joint rows are written out as the commanded joint
torques.

---

## Known issue: not yet adapted for this legs-only 12-DOF robot

`computeDdq()` was copied from OpenLoong-Dyn-Control, written for their full
~31-DOF humanoid (arms, head, waist, legs). It still contains **hardcoded
`q`-vector indices** assuming that full joint layout, e.g. `q(20)`,
`q(21..25)`, `q(28)`, `q(34)`, `q.block<14,1>(7,0)` — these index into the
arm/head/waist portion of OpenLoong's `q` vector (roughly: floating base 0-6,
arms 7-20, head 21-22, waist 23-25, legs 26-37, indices approximate).

This project's `q` vector is only **19 elements** (7 floating-base + 12 leg
joints). Any code path in `computeDdq()` that touches those higher indices is
**out of bounds** for this robot and would crash or read garbage if actually
exercised.

This is the same bug class already found and fixed multiple times in
`pino_kin_dyn.cpp` (see the `pino_kin_dyn_legs_only_adaptation` memory) —
hand/waist/arm task rows need to be stripped out of `computeDdq()`'s active
task list and any indexing into `q`/`dq`/`ddq` needs to be re-derived from
this project's actual 19-element floating-base + 12-DOF-leg layout before
`WBC_priority` can be safely invoked. **`WBC_solv.computeDdq()`/`computeTau()`
are currently commented out in `demo/walk_wbc.cpp` and have not been called
yet in this project.**

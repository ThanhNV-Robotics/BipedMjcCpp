# Operational-Space Theory Behind `PriorityTasks::computeAll()`

This note explains the control theory implemented by
`PriorityTasks::computeAll()` in `algorithm/priority_tasks.cpp`, and maps
every term in the code back to the underlying equations. It complements
`doc/wbc_priority.md` (which covers the surrounding `WBC_priority` module,
QP correction stage, and task setup) by focusing specifically on the theory
inside this one function.

---

## 1. The problem: many Cartesian tasks, one set of joints

A legged robot has one joint-acceleration vector `ddq` (dimension =
`model_nv`) but several simultaneous Cartesian objectives competing for it:
keep the stance foot planted, keep the torso upright, track a swing-foot
trajectory, hold a CoM height, etc. Each task `i` lives in its own
task space with its own Jacobian:

```
x_i = f_i(q)          (task-space position, e.g. foot position, torso RPY)
dx_i  = J_i * dq
ddx_i = J_i * ddq + dJ_i * dq
```

If every task tried to solve for `ddq` independently, they would conflict.
**Task-priority control** resolves this by ranking tasks and solving them in
order, each one only using the joint motion left over ("null space") after
all higher-priority tasks have taken what they need — so a lower-priority
task can literally never disturb a higher-priority one.

---

## 2. Khatib's operational-space formulation

This comes from Oussama Khatib's operational-space control framework
(Khatib, 1987), which asks: given the robot's rigid-body dynamics

```
M(q) * ddq + Non(q, dq) = tau + J^T * F        (Non = Coriolis/centrifugal + gravity)
```

what is the "correct" way to map a desired task-space behavior into joint
space so that the result is consistent with the robot's actual inertia,
not just its kinematics?

### 2.1 Kinematic vs. dynamically-consistent pseudo-inverse

The plain Moore-Penrose pseudo-inverse

```
J+ = J^T * (J*J^T)^-1
```

resolves redundancy by minimizing joint-space velocity/acceleration norm.
It is purely kinematic — it has no notion of the robot's mass distribution.

Khatib's key result is the **dynamically-consistent generalized inverse**:

```
J# = M^-1 * J^T * (J * M^-1 * J^T)^-1
```

Using `J#` instead of `J+` to map a task force/acceleration into joint
space minimizes *kinetic energy* rather than raw joint-space norm, and —
critically — the associated null-space projector

```
N = I - J# * J
```

is **dynamically consistent**: joint motion inside this null space produces
*zero* reaction force/acceleration at the task point, even though the
joints are physically coupled through the mass matrix `M`. A kinematic
null-space projector (`I - J+*J`) only guarantees zero *velocity* coupling,
not zero *force/inertial* coupling — so at the acceleration/torque level it
is the wrong tool; only `J#` gives a hierarchy that is truly
non-interfering once dynamics are involved.

This is exactly `dyn_pseudoInv()` in `math/useful_math.cpp:71`:

```cpp
Eigen::MatrixXd dyn_pseudoInv(const Eigen::MatrixXd &M, const Eigen::MatrixXd &dyn_M, bool isMinv)
{
    Eigen::MatrixXd Minv = isMinv ? dyn_M : dyn_M.llt().solve(Identity);
    Eigen::MatrixXd temp = M * Minv * M.transpose();
    return Minv * M.transpose() * temp.completeOrthogonalDecomposition().pseudoInverse();
}
```
i.e. `J# = Minv * J^T * (J * Minv * J^T)^+`, called in `computeAll()` with
`isMinv=true` and `dyn_M_inv = M^-1` already precomputed — this is precisely
`J# = M^-1 J^T (J M^-1 J^T)^-1`.

By contrast, `pseudoInv_right_weighted()` (`math/useful_math.cpp:51`) is the
plain **kinematic** weighted pseudo-inverse:

```
J+_W = W^-1 * J^T * (J * W^-1 * J^T)^+
```

(`W` a diagonal per-row task weight, not the mass matrix). It is used for
the position (`delta_q`) and velocity (`dq`) levels, where there is no
dynamics/force-coupling concern — only at the acceleration level does the
code need the dynamically-consistent `J#`, since that's where forces and
inertia actually enter.

---

## 3. Recursive prioritized null-space projection

For a stack of tasks ranked `0` (highest) … `n` (lowest), the textbook
formulation (Siciliano & Slotine, 1991; extended to the dynamically
consistent case by Khatib et al./Sentis & Khatib, 2005) builds the null
space of *all* higher-priority tasks stacked together:

```
Jpre_i = [J_0; J_1; ...; J_{i-1}]                (stack of everything above task i)
N_i    = I - pinv(Jpre_i) * Jpre_i
```

Recomputing this stack and its pseudo-inverse at every level is O(n²) work.
`computeAll()` instead uses the equivalent **recursive** update (line
86-88), which reaches the same null space incrementally in O(n):

```
N_i    = N_{i-1} * (I - J#_{i-1} * Jpre_{i-1})
Jpre_i = J_i * N_i
```

i.e. "the previous task's null space, intersected with the null space of
the previous task's own row space." Each task's effective Jacobian
`Jpre_i = J_i * N_i` is therefore already invisible to every task ranked
above it — solving with it can never undo a higher-priority task's result.

---

## 4. The three quantities resolved per task: `delta_q`, `dq`, `ddq`

`computeAll()` resolves a task at three levels simultaneously — position
increment, velocity, and dynamically-consistent acceleration — walking the
priority chain built by `buildPriority()` via each task's `parentId`.

### 4.1 Highest-priority task (`parentId == -1`, lines 77-84)

```
N        = I                                   (nothing above it — full DOF available)
Jpre     = J
delta_q  = des_delta_q + J+_W(Jpre) * errX                       (position IK correction)
dq       = des_dq                                                 (passed through)
ddx_cmd  = ddxDes + Kp*errX + Kd*derrX                             (task-space PD accel. command)
ddq      = des_ddq + J#(Jpre) * (ddx_cmd - dJ*dq)                  (dynamically-consistent)
```

`errX`/`derrX` are the task's position/velocity error (desired − current);
`ddx_cmd` is a standard operational-space PD tracking law — feedforward
acceleration plus proportional/derivative error correction in task space.

### 4.2 Every lower-priority task (`parentId != -1`, lines 86-97)

```
N        = N_parent * (I - J#_W(Jpre_parent) * Jpre_parent)
Jpre     = J * N
delta_q  = delta_q_parent + J+_W(Jpre) * (errX  - J*delta_q_parent)
dq       = dq_parent      + J+_W(Jpre) * (dxDes - J*dq_parent)
ddx_cmd  = ddxDes + Kp*errX + Kd*derrX
ddq      = ddq_parent + J#(Jpre) * (ddx_cmd - dJ*dq - J*ddq_parent)
```

The pattern in each line is the same: **take the parent's already-solved
value, subtract off how much of *this* task is already (accidentally)
satisfied by that parent motion (`J*delta_q_parent`, `J*dq_parent`,
`J*ddq_parent`), and resolve only the residual** through the null-space-
projected Jacobian `Jpre`, guaranteeing the correction is invisible to every
task above it.

### 4.3 Final output (lines 110-112)

After walking the chain to its end (`childId == -1`), the last task's
`delta_q`/`dq`/`ddq` already contains the folded-in contribution of every
task above it, so it is returned directly as the solver's output —
no further stacking/summation is needed.

---

## 5. Summary table

| Quantity   | Pseudo-inverse used        | Why                                                              |
|------------|-----------------------------|-------------------------------------------------------------------|
| `delta_q`  | `pseudoInv_right_weighted`  | Position-level IK — purely kinematic redundancy resolution        |
| `dq`       | `pseudoInv_right_weighted`  | Velocity-level IK — same, no dynamics involved                    |
| `ddq`      | `dyn_pseudoInv`              | Acceleration-level — must respect inertia (Khatib operational-space) so higher-priority tasks are dynamically, not just kinematically, protected |

---

## 6. Note: root-task `dq` asymmetry

At line 81, the highest-priority task's `dq` is copied directly from
`des_dq` rather than resolved via `J+_W(Jpre) * dxDes` the way every
lower-priority task's `dq` is (line 91-92). This is inconsistent with how
`delta_q` and `ddq` are computed for the *same* root task, and is worth
confirming against the original OpenLoong source is intentional (e.g. root
velocity assumed externally driven) rather than an oversight — see also the
adaptation caveats in `doc/wbc_priority.md`.

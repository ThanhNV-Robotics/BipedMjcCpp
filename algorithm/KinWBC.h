#pragma once

#include <Eigen/Dense>
#include "useful_math.h"
#include <utility>
#include <vector>
#include <string>
#include <iostream>
#include "data_type.h"

#include "robot_wrapper.h"


#include "joystick_interpreter.h" // for base reference motion
#include "foot_placement.h" // for foot reference motion
#include "CP_Planning.h" // for CoM XY reference motion
#include "my_gait_scheduler.h" // for motion-state/leg-state task-list switching

struct Task {

    std::string taskName;

    VectorXd X_cur, dX_cur; // feedback in task space
    VectorXd deltaX_des, X_des, dX_des, ddX_des; //task space desired velocity and acceleration 
    VectorXd errX, derrX;

    VectorXd delta_q, dq, ddq; //joint space velocity and acceleration
    MatrixXd J, dJ, Jpre; // Jacobian, Jacobian derivative,...
    MatrixXd N; // Null space projection matrix of J
    MatrixXd kp, kd; // pd gain in task space
    Eigen::DiagonalMatrix<double, -1> W; //weighted matrix for pseudo inverse

    Task(std::string name) {taskName = name;}; // constructor
};

class KinWBC {
public:
    // Constructor
    KinWBC (const std::string &wbc_config_yaml_path);
    // member tasks
    Task task_left_contact  = Task("left_contact"); // input 1 to access to left_feet (jacobian, position, vel)
    Task task_right_contact  = Task("right_contact"); // input 2 to access to right_feet (jacobian, position, vel)
    Task task_CoMXY = Task("CoMXY");  // input 3 to access to Jcom_W in robot_wrapper
    Task task_CoMZc = Task("CoMZc");  // CoM Height task
    Task task_base_height = Task("base_height"); // input 0 to access to base end-effector
    Task task_base_rpy = Task("base_rpy"); // control base orientation

    Task task_static_contact = Task("static_contact"); // specifically for walking
    Task task_swing_leg = Task("swing_leg");
    Task task_lift_foot = Task("lift_foot");

    Task task_PosRot = Task("PosRot");

    // Joint-space postural task -- lowest priority, only in
    // kin_task_walk/kin_task_init_walk (single-stance lists). Ported from
    // OpenLoong's "RedundantJoints" task, adapted for this robot: OpenLoong
    // locks a handful of genuinely-spare joints (waist/head) to zero;
    // this robot has none spare (all 12 actuated joints are already load-
    // bearing leg joints), so this instead softly biases ALL 12 toward a
    // nominal bent-knee configuration (posture_nominal_) rather than
    // rigidly locking anything. Confirmed earlier this session as the
    // actual fix for the stance-knee collapsing-to-singularity failure
    // mode during single-stance WALK/WARM_UP: nothing else in the task
    // list biases the leg toward a bent-knee posture, so the null-space of
    // higher-priority tasks can drift the stance leg toward full-extension
    // singularity over a swing cycle.
    Task task_posture = Task("posture");
    // Nominal joint configuration task_posture.X_des targets -- set this
    // once (e.g. from computeInitial_Stand()'s result) before the first
    // computeWBC_IK() call. If left at its default empty size, updateCurrent()
    // falls back to X_des=X_cur (inert: no postural pull at all) rather than
    // chasing a bogus all-zero target.
    VectorXd posture_nominal_;

    std::vector<Task*> kin_task_stand; //using pointer to access the member tasks
    std::vector<Task*> kin_task_walk; //forward walking
    std::vector<Task*> kin_task_walk_test; //forward walking
    std::vector<Task*> kin_task_init_walk; // init walking task 
    // std::vector<Task> kin_task_walk;
    VectorXd out_delta_q, out_dq, out_ddq;
    VectorXd q_des; // full integrated generalized configuration [base_pos(3), quat(4), joint_pos(na)]

    VectorXd integrateDIY(const VectorXd &qI, const VectorXd &dqI);
    VectorXd getMotorPosDes() const { return q_des.segment(7, q_des.size() - 7); }

    const double dt = 0.001; // sampling time

    
    void printTaskInfo();
    void updateReference(const JoyStickInterpreter& joyStick, FootPlacement& footPlanner, const CP_Planning& cp_planning); // get referece from task planner
    // CP_Planning-free overload -- task_CoMXY tracks the joystick's OWN
    // integrated position (joyStick.px_W/py_W, already updated every
    // JoyStickInterpreter::step() tick from vx_W/vy_W) instead of a
    // capture-point/ZMP plan, matching OpenLoong-Dyn-Control's approach
    // (their WBC tracks a position target that's a direct velocity
    // integral -- see PosRot/CoMXY_HipRPY in wbc_priority.cpp -- not a
    // closed-form LIPM boundary-value blend). In STAND, task_CoMXY.X_des
    // is instead the midpoint of both feet (same override OpenLoong's own
    // demo applies only while motionState==Stand -- see walk_wbc_joystick.cpp's
    // pCoMDes(0)/(1) = (fe_l_pos_W+fe_r_pos_W)*0.5 -- not used during WALK,
    // where PosRot/js_pos_des takes over instead), hence the extra
    // robot_wrapper parameter (needed for pos_L_feet_W/pos_R_feet_W).
    void updateReference(const JoyStickInterpreter& joyStick, FootPlacement& footPlanner, const RobotWrapper& robot_wrapper);
    void updateCurrent (const RobotWrapper& rb_wrapper, const FootPlacement& footPlanner); // update current task space estimation
    void computeWBC_IK (const JoyStickInterpreter &joyStick, FootPlacement &footPlanner, const RobotWrapper& robot_wrapper, const CP_Planning& cp_planning, const MyGaitScheduler& gait_scheduler);
    // CP_Planning-free overload, pairs with the updateReference() overload above.
    void computeWBC_IK (const JoyStickInterpreter &joyStick, FootPlacement &footPlanner, const RobotWrapper& robot_wrapper, const MyGaitScheduler& gait_scheduler);

private:
    // Shared tail of computeWBC_IK() (task-list selection + recursive
    // null-space solve) -- both the CP_Planning and CP_Planning-free
    // overloads call this after running their own updateReference().
    void solveTasks(const RobotWrapper& robot_wrapper, const FootPlacement& footPlanner, const MyGaitScheduler& gait_scheduler);

    // Tried feeding task_swing_leg/task_lift_foot's dX_des from a finite-
    // difference of footPlanner_cmd.getSwingDesPos() instead of a hardcoded
    // 0 (to stop the recursive solve's dq = parent.dq + pseudoInv(...)*
    // (dX_des - J*parent.dq) from fighting the foot's real non-constant
    // cycloid/Raibert swing velocity). Reverted: it didn't fix
    // test_DynWalk_joystick.cpp's WALK-transition instability (root cause
    // turned out to be the missing weight-shift phase before single
    // support, not this), and it broke test_WBCKin_walk_joystick.cpp (a
    // previously-stable pure-kinematic baseline) -- that test sets
    // robot_wrapper.dq = kin_wbc.out_dq directly every tick with zero
    // physical damping, so feeding back a finite-difference derivative of a
    // target that's ITSELF a function of measured velocity (the Raibert
    // heuristic in posDes_W) created an undamped algebraic feedback loop
    // that diverged to NaN within ~1s of WALK starting.
};
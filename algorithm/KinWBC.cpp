// Compute desired 

#include "KinWBC.h"
#include "data_type.h"
#include "robot_wrapper.h"
#include "yaml-cpp/yaml.h"
#include <algorithm>
#include <cstdio>
#include <stdexcept>

// constructor
KinWBC::KinWBC(const std::string &wbc_config_yaml_path)
{
    // // construct stand and walk task, in priority order (index 0 = highest)

    //---------------Stand task---------------------------
    // kin_task_stand is a vector of pointers, so it stores the address of each task
    kin_task_stand.push_back(&task_left_contact);
    kin_task_stand.push_back(&task_right_contact);
    kin_task_stand.push_back(&task_CoMXY); 
    kin_task_stand.push_back(&task_base_rpy);   
    // kin_task_stand.push_back(&task_CoMZc);
    kin_task_stand.push_back(&task_base_height);

    //---------------Warm Up task---------------------------
    kin_task_init_walk.push_back(&task_static_contact);
    kin_task_init_walk.push_back(&task_lift_foot);
    kin_task_init_walk.push_back(&task_CoMXY);
    kin_task_init_walk.push_back(&task_base_rpy);
    kin_task_init_walk.push_back(&task_base_height);

    //---------------Walking task test 1---------------------------
 
    kin_task_walk.push_back(&task_static_contact);
    kin_task_walk.push_back(&task_CoMXY);
    kin_task_walk.push_back(&task_base_rpy);
    kin_task_walk.push_back(&task_base_height);
    kin_task_walk.push_back(&task_swing_leg);

    //---------------Walking task test 2---------------------------    
    kin_task_walk_test.push_back(&task_static_contact); //forward walking
    kin_task_walk_test.push_back(&task_PosRot);
    kin_task_walk_test.push_back(&task_swing_leg);

    //---------------Per-task operational-space PD gains-----
    // Used only for the acceleration-level solve (out_ddq, see
    // computeWBC_IK()): ddxcmd = ddX_des + kp*errX + kd*derrX.
    YAML::Node wbc_config = YAML::LoadFile(wbc_config_yaml_path);
    const YAML::Node &gains = wbc_config["kin_task_gain"];

    auto loadTaskGain = [&gains](Task &task, int dim) {
        const YAML::Node &g = gains[task.taskName];
        task.kp = MatrixXd::Identity(dim, dim) * g["kp"].as<double>();
        task.kd = MatrixXd::Identity(dim, dim) * g["kd"].as<double>();
    };

    loadTaskGain(task_left_contact,   6);
    loadTaskGain(task_right_contact,  6);
    loadTaskGain(task_static_contact, 6);
    loadTaskGain(task_CoMXY,          2);
    loadTaskGain(task_base_rpy,       3);
    loadTaskGain(task_base_height,    1);
    loadTaskGain(task_CoMZc,          1);
    loadTaskGain(task_swing_leg,      6);
    loadTaskGain(task_lift_foot,      6);
    loadTaskGain(task_posture,        12);
}

void KinWBC::printTaskInfo() {
    for (int i=0;i<kin_task_stand.size();i++)
    {
        printf("-------------\n");
        printf("taskName=%s\n",kin_task_stand[i]->taskName.c_str());
        printf("Priority Order: %d \n", i);
    }
}

void KinWBC::computeWBC_IK (const JoyStickInterpreter &joyStick, FootPlacement &footPlanner, const RobotWrapper& robot_wrapper, const CP_Planning& cp_planning, const MyGaitScheduler& gait_scheduler)
{
    updateReference(joyStick, footPlanner, cp_planning);
    solveTasks(robot_wrapper, footPlanner, gait_scheduler);
}

void KinWBC::computeWBC_IK (const JoyStickInterpreter &joyStick, FootPlacement &footPlanner, const RobotWrapper& robot_wrapper, const MyGaitScheduler& gait_scheduler)
{
    updateReference(joyStick, footPlanner, robot_wrapper);
    solveTasks(robot_wrapper, footPlanner, gait_scheduler);
}

void KinWBC::solveTasks(const RobotWrapper& robot_wrapper, const FootPlacement& footPlanner, const MyGaitScheduler& gait_scheduler)
{
    // get feedback
    updateCurrent(robot_wrapper, footPlanner);

    // select the task based on gait_scheduler MotionState

    std::vector<Task*>* kin_task_ptr;
    switch (gait_scheduler.motionState) {
        case MotionState::WARM_UP:
            kin_task_ptr = &kin_task_init_walk;
            break;
        case MotionState::WALK:
            kin_task_ptr = &kin_task_walk;
            break;
        case MotionState::STAND:
            kin_task_ptr = &kin_task_stand;
            break;
        case MotionState::WALK_TO_STAND:
            kin_task_ptr = &kin_task_stand;
            break;
        default:
            kin_task_ptr = &kin_task_stand;
            break;
    }
    std::vector<Task*> &kin_task = *kin_task_ptr;
    // recursive null-space priority solver
    const int nv = robot_wrapper.model_nv_;
    const VectorXd &dq_cur = robot_wrapper.dq;

    const bool haveDynMInv = (robot_wrapper.dyn_M_inv.rows() == nv && robot_wrapper.dyn_M_inv.cols() == nv);

    // Compute des_delta_q, des_dq, des_ddq
    const VectorXd des_delta_q = VectorXd::Zero(nv);
    const VectorXd des_dq = VectorXd::Zero(nv);
    const VectorXd des_ddq = VectorXd::Zero(nv);

    for (size_t i = 0; i < kin_task.size(); i++)
    {
        Task &task = *kin_task[i];

        VectorXd ddxcmd = task.ddX_des + task.kp * task.errX + task.kd * task.derrX;

        if (i == 0) // 1st task in the list has the highest priority
        {
            task.N = MatrixXd::Identity(nv, nv);
            task.Jpre = task.J * task.N;
            task.delta_q = des_delta_q + pseudoInv_right_weighted(task.Jpre, task.W) * (task.errX - task.J * des_delta_q);
            task.dq = des_dq + pseudoInv_right_weighted(task.Jpre, task.W) * (task.dX_des - task.J * des_dq);
            if (haveDynMInv)
                task.ddq = des_ddq + dyn_pseudoInv(task.Jpre, robot_wrapper.dyn_M_inv, true)
                               * (ddxcmd - task.dJ * dq_cur - task.J * des_ddq);
            else
                task.ddq = VectorXd::Zero(nv);
        }
        else
        {
            Task &parent = *kin_task[i - 1];
            task.N = parent.N * (MatrixXd::Identity(parent.Jpre.cols(), parent.Jpre.cols()) - pseudoInv_right_weighted(parent.Jpre, parent.W) * parent.Jpre);
            task.Jpre = task.J * task.N;
            task.delta_q = parent.delta_q + pseudoInv_right_weighted(task.Jpre, task.W) * (task.errX - task.J * parent.delta_q);
            task.dq = parent.dq + pseudoInv_right_weighted(task.Jpre, task.W) * (task.dX_des - task.J * parent.dq);

            if (haveDynMInv)
                task.ddq = parent.ddq + dyn_pseudoInv(task.Jpre, robot_wrapper.dyn_M_inv, true) * (ddxcmd - task.dJ * dq_cur - task.J * parent.ddq);
            else
                task.ddq = VectorXd::Zero(nv);
        }
        // Diagnostic: kinematic-level correction only (delta_q/errX/dq/
        // derrX, what test_WBCKin_stand.cpp actually integrates), gated to
        // fire only once something's already elevated above the normal
        // ~0.001-0.003 baseline, so it traces an instability's onset
        // without flooding routine ticks.
        if (task.delta_q.norm() > 0.01)
            std::cout << "  [" << i << "] " << task.taskName
                      << "  |errX|=" << task.errX.norm()
                      << "  |derrX|=" << task.derrX.norm()
                      << "  |delta_q|=" << task.delta_q.norm()
                      << "  |dq|=" << task.dq.norm() << std::endl;
    }

    out_delta_q = kin_task.back()->delta_q;
    out_dq = kin_task.back()->dq;
    out_ddq = kin_task.back()->ddq;
    q_des = integrateDIY(robot_wrapper.q, out_delta_q);
    return;

}

void KinWBC::updateReference(const JoyStickInterpreter& joyStick_cmd, FootPlacement& footPlanner_cmd, const CP_Planning& cp_planning)
{
    // update robot reference task space

    //---------------------Stand----------------------------------------------
    // left_contact
   
    task_left_contact.X_des = VectorXd::Zero(6);
    task_left_contact.dX_des = VectorXd::Zero(6);
    task_left_contact.ddX_des = VectorXd::Zero(6);

    // right_contact
   
    task_right_contact.X_des = VectorXd::Zero(6);
    task_right_contact.dX_des = VectorXd::Zero(6);
    task_right_contact.ddX_des = VectorXd::Zero(6);

    // CoMXY -- tracks CP_Planning's generated CoM x,y trajectory 
    
    task_CoMXY.X_des = Vector2d(cp_planning.xc_, cp_planning.yc_);
    task_CoMXY.dX_des = Vector2d(cp_planning.d_xc_, cp_planning.d_yc_);
    task_CoMXY.ddX_des = VectorXd::Zero(2);

    // base heigh
    task_base_height.X_des = VectorXd::Constant(1, joyStick_cmd.pz_W);
    task_base_height.dX_des = VectorXd::Constant(1, joyStick_cmd.vz_W);
    task_base_height.ddX_des = VectorXd::Zero(1); // feedforward acceleration is 0

    // CoM height -- same commanded reference as base_height (joystick's
    // pz_W/vz_W), just tracked via the CoM's own Z instead of the base
    // link's. See updateCurrent() for why: constrains actual CoM height
    // directly rather than base height, which only matches CoM height if
    // the legs/torso mass distribution keeps them coincident.
    task_CoMZc.X_des = VectorXd::Constant(1, joyStick_cmd.pz_W);
    task_CoMZc.dX_des = VectorXd::Constant(1, joyStick_cmd.vz_W);
    task_CoMZc.ddX_des = VectorXd::Zero(1);

    // base rpy
    task_base_rpy.X_des = Vector3d(0.0, joyStick_cmd.thetaY, joyStick_cmd.thetaZ);
    task_base_rpy.dX_des = VectorXd::Zero(3);
    task_base_rpy.ddX_des = VectorXd::Zero(3);

    // static_contact
    task_static_contact.X_des = VectorXd::Zero(6);
    task_static_contact.dX_des = VectorXd::Zero(6);
    task_static_contact.ddX_des = VectorXd::Zero(6);

    // lift foot (6-dim: position + orientation)
    task_lift_foot.X_des = VectorXd::Zero(6);
    task_lift_foot.X_des.head<3>() = footPlanner_cmd.getSwingDesPos();
    task_lift_foot.dX_des = VectorXd::Zero(6);
    task_lift_foot.ddX_des = VectorXd::Zero(6);

    // swing leg (6-dim: position + orientation) of the swing feet
    task_swing_leg.X_des = VectorXd::Zero(6); // init
    task_swing_leg.X_des.head<3>() = footPlanner_cmd.getSwingDesPos();
    task_swing_leg.dX_des = VectorXd::Zero(6);
    task_swing_leg.ddX_des = VectorXd::Zero(6);

    // posture -- X_des (with its posture_nominal_-or-hold-current fallback)
    // is set in updateCurrent(), not here, same reasoning as
    // left_contact/static_contact above.
    task_posture.dX_des = VectorXd::Zero(12);
    task_posture.ddX_des = VectorXd::Zero(12);

    return;
}

void KinWBC::updateReference(const JoyStickInterpreter& joyStick_cmd, FootPlacement& footPlanner_cmd, const RobotWrapper& robot_wrapper)
{
    // Identical to the CP_Planning overload above, except task_CoMXY tracks
    // the joystick's OWN integrated position/velocity (px_W/py_W/vx_W/vy_W,
    // already updated every JoyStickInterpreter::step() tick) instead of a
    // capture-point plan -- see this overload's declaration comment in
    // KinWBC.h.
    task_left_contact.X_des = VectorXd::Zero(6);
    task_left_contact.dX_des = VectorXd::Zero(6);
    task_left_contact.ddX_des = VectorXd::Zero(6);

    task_right_contact.X_des = VectorXd::Zero(6);
    task_right_contact.dX_des = VectorXd::Zero(6);
    task_right_contact.ddX_des = VectorXd::Zero(6);

    // In STAND, OpenLoong's own demo overrides its CoM-xy target to the
    // midpoint of both feet (walk_wbc_joystick.cpp's pCoMDes(0)/(1) =
    // (fe_l_pos_W+fe_r_pos_W)*0.5, applied only while motionState==Stand)
    // instead of tracking js_pos_des -- during WALK that override never
    // happens, PosRot/js_pos_des takes over instead, matching the
    // joystick-tracking branch below.
    if (joyStick_cmd.getMotionState() == MotionState::STAND)
    {
        task_CoMXY.X_des = 0.5 * (robot_wrapper.pos_L_feet_W.head<2>() + robot_wrapper.pos_R_feet_W.head<2>());
        task_CoMXY.dX_des = Vector2d::Zero();
    }
    else
    {
        task_CoMXY.X_des = Vector2d(joyStick_cmd.px_W, joyStick_cmd.py_W);
        task_CoMXY.dX_des = Vector2d(joyStick_cmd.vx_W, joyStick_cmd.vy_W);
    }
    task_CoMXY.ddX_des = VectorXd::Zero(2);

    task_base_height.X_des = VectorXd::Constant(1, joyStick_cmd.pz_W);
    task_base_height.dX_des = VectorXd::Constant(1, joyStick_cmd.vz_W);
    task_base_height.ddX_des = VectorXd::Zero(1);

    task_CoMZc.X_des = VectorXd::Constant(1, joyStick_cmd.pz_W);
    task_CoMZc.dX_des = VectorXd::Constant(1, joyStick_cmd.vz_W);
    task_CoMZc.ddX_des = VectorXd::Zero(1);

    task_base_rpy.X_des = Vector3d(0.0, joyStick_cmd.thetaY, joyStick_cmd.thetaZ);
    task_base_rpy.dX_des = VectorXd::Zero(3);
    task_base_rpy.ddX_des = VectorXd::Zero(3);

    task_static_contact.X_des = VectorXd::Zero(6);
    task_static_contact.dX_des = VectorXd::Zero(6);
    task_static_contact.ddX_des = VectorXd::Zero(6);

    task_lift_foot.X_des = VectorXd::Zero(6);
    task_lift_foot.X_des.head<3>() = footPlanner_cmd.getSwingDesPos();
    task_lift_foot.dX_des = VectorXd::Zero(6);
    task_lift_foot.ddX_des = VectorXd::Zero(6);

    task_swing_leg.X_des = VectorXd::Zero(6);
    task_swing_leg.X_des.head<3>() = footPlanner_cmd.getSwingDesPos();
    task_swing_leg.dX_des = VectorXd::Zero(6);
    task_swing_leg.ddX_des = VectorXd::Zero(6);

    task_posture.dX_des = VectorXd::Zero(12);
    task_posture.ddX_des = VectorXd::Zero(12);

    return;
}

void KinWBC::updateCurrent (const RobotWrapper& rb_wrapper, const FootPlacement& footPlanner) // update current task space estimation
{
    // joint-space weight matrices for pseudoInv_right_weighted(), M^+_W =
    // W^-1 M^T (M W^-1 M^T)^-1 -- W must be sized to M's COLUMN count
    // (joint/nv space, model_nv_), not the task's own row/output dimension.
    // Equal weighting across all joints for now.
    const int nv = rb_wrapper.model_nv_;
    task_left_contact.W = Eigen::VectorXd::Ones(nv).asDiagonal();
    task_right_contact.W = Eigen::VectorXd::Ones(nv).asDiagonal();
    task_CoMXY.W = Eigen::VectorXd::Ones(nv).asDiagonal();
    task_base_height.W = Eigen::VectorXd::Ones(nv).asDiagonal();
    task_CoMZc.W = Eigen::VectorXd::Ones(nv).asDiagonal();
    task_base_rpy.W = Eigen::VectorXd::Ones(nv).asDiagonal();
    task_static_contact.W = Eigen::VectorXd::Ones(nv).asDiagonal();
    task_lift_foot.W = Eigen::VectorXd::Ones(nv).asDiagonal();
    task_swing_leg.W = Eigen::VectorXd::Ones(nv).asDiagonal();
    task_posture.W = Eigen::VectorXd::Ones(nv).asDiagonal();

    // base height (task is 1-dim: z only -- J_base_W's row 2 is the base's z-row,
    // since J_base_W.block<3,3>(0,0) = I maps base linear-vel dof straight to rows 0-2)
    task_base_height.X_cur = VectorXd::Constant(1, rb_wrapper.pos_base_W(2));
    task_base_height.dX_cur = VectorXd::Constant(1, rb_wrapper.vel_base_W(2));
    task_base_height.J = rb_wrapper.J_base_W.row(2);
    task_base_height.dJ = rb_wrapper.dJ_base_W.row(2);
    task_base_height.errX = task_base_height.X_des - task_base_height.X_cur;
    task_base_height.derrX = task_base_height.dX_des - task_base_height.dX_cur;

    // CoM height (task is 1-dim: z only -- Jcom_W's row 2 is the CoM's z-row).
    // No dJcom_W is available from RobotWrapper (only Jcom_W, no time-
    // derivative), so dJ is left at an explicit zero here -- a "zero drift"
    // approximation for the dynamically-consistent ddq solve's dJ*dq term,
    // same effective assumption task_CoMXY's own (never-assigned, always-
    // default) dJ already makes.
    task_CoMZc.X_cur = VectorXd::Constant(1, rb_wrapper.pos_CoM_W(2));
    task_CoMZc.dX_cur = VectorXd::Constant(1, rb_wrapper.vel_CoM_W(2));
    task_CoMZc.J = rb_wrapper.Jcom_W.row(2);
    task_CoMZc.dJ = MatrixXd::Zero(1, nv);
    task_CoMZc.errX = task_CoMZc.X_des - task_CoMZc.X_cur;
    task_CoMZc.derrX = task_CoMZc.dX_des - task_CoMZc.dX_cur;

    // base rpy (3-dim: orientation error toward upright, world frame).
    // J_base_W's rows 3-5 come from dq's angular-vel dof, which pinocchio
    // keeps in the BASE's local frame (see the frame note at the top of
    // robot_wrapper.cpp), so they're rotated into world frame here by Rcur --
    // otherwise they wouldn't be expressed in the same frame as X_cur/errX
    // below (diffRot() returns its result in world frame).
    Eigen::Quaterniond quat_base_W(rb_wrapper.q(6), rb_wrapper.q(3), rb_wrapper.q(4), rb_wrapper.q(5)); // (w,x,y,z) from Pinocchio's (x,y,z,w) coeffs order
    Eigen::Matrix3d Rcur_base = quat_base_W.toRotationMatrix();
    task_base_rpy.X_cur = diffRot(Eigen::Matrix3d::Identity(), Rcur_base); // how far current orientation has tilted away from upright
    task_base_rpy.dX_cur = Rcur_base * rb_wrapper.dq.segment<3>(3);
    task_base_rpy.J = Rcur_base * rb_wrapper.J_base_W.bottomRows(3);
    task_base_rpy.dJ = Rcur_base * rb_wrapper.dJ_base_W.bottomRows(3);
    task_base_rpy.errX = task_base_rpy.X_des - task_base_rpy.X_cur;
    task_base_rpy.derrX = task_base_rpy.dX_des - task_base_rpy.dX_cur;

    // left contact (6-dim: position + orientation)
    task_left_contact.X_cur = rb_wrapper.pos_L_feet_W;
    task_left_contact.dX_cur = rb_wrapper.vel_L_feet_W;
    task_left_contact.J = rb_wrapper.J_Lfeet_W;
    task_left_contact.dJ = rb_wrapper.dJ_Lfeet_W; 
    task_left_contact.errX = VectorXd::Zero(6);
    task_left_contact.derrX = VectorXd::Zero(6);

    // right contact (6-dim: position + orientation)
    task_right_contact.X_cur = rb_wrapper.pos_R_feet_W;
    task_right_contact.dX_cur = rb_wrapper.vel_R_feet_W;    
    task_right_contact.J = rb_wrapper.J_Rfeet_W;
    task_right_contact.dJ = rb_wrapper.dJ_Rfeet_W;
    task_right_contact.errX = VectorXd::Zero(6);
    task_right_contact.derrX = VectorXd::Zero(6);

    // CoMXY (task is 2-dim: x,y only -- Jcom_W's top 2 rows). dJ: no
    // dJcom_W available from RobotWrapper (only Jcom_W, no time-derivative),
    // so an explicit zero-drift approximation, same as task_CoMZc's --
    // MUST be a properly-sized zero matrix, not left default-constructed
    // (0x0): task.dJ * dq_cur in computeWBC_IK()'s ddq loop is a dimension
    // mismatch against a 0x0 matrix, which is UB in a release build
    // (assertions compiled out) -- confirmed as the source of the garbage
    // ddq_cmd_/QP-infeasibility DynWBC saw in double-support stand, since
    // that's the only caller that reaches this dynamically-consistent path
    // (pure-kinematic tests skip it via haveDynMInv).
    task_CoMXY.X_cur = rb_wrapper.pos_CoM_W.head(2);
    task_CoMXY.dX_cur = rb_wrapper.vel_CoM_W.head(2);
    task_CoMXY.J = rb_wrapper.Jcom_W.topRows(2);
    task_CoMXY.dJ = MatrixXd::Zero(2, nv);

    // CoMXY CLIK gain step (scales displacement error to per-step deltaX)
    const double dt = 0.001;
    const double kp_com = 100.0;
    Vector2d deltaX_com = task_CoMXY.dX_des * dt + kp_com * (task_CoMXY.X_des - task_CoMXY.X_cur) * dt;
    for (int k = 0; k < 2; ++k) {
        if (std::fabs(deltaX_com(k)) > 0.002)
            deltaX_com(k) = 0.002 * ((deltaX_com(k) > 0) ? 1.0 : -1.0);
    }
    task_CoMXY.errX = deltaX_com;
    task_CoMXY.derrX = task_CoMXY.dX_des - task_CoMXY.dX_cur;
    for (int k = 0; k < 2; ++k) {
        if (std::fabs(task_CoMXY.derrX(k)) > 0.5)
            task_CoMXY.derrX(k) = 0.5 * ((task_CoMXY.derrX(k) > 0) ? 1.0 : -1.0);
    }

    // static_contact (6-dim: position + orientation) -- whichever foot is
    // currently the stance leg, "hold current pose" convention (errX/derrX
    // forced to Zero(6) below regardless of X_des, matching task_left_
    // contact/task_right_contact above). DSt (before walking starts / mid-
    // transition) defaults to the left foot.
    if (footPlanner.legState == LegState::RSt)
    {
        task_static_contact.X_cur = rb_wrapper.pos_R_feet_W;
        task_static_contact.dX_cur = rb_wrapper.vel_R_feet_W;
        task_static_contact.J = rb_wrapper.J_Rfeet_W;
        task_static_contact.dJ = rb_wrapper.dJ_Rfeet_W;
    }
    else // LSt or DSt
    {
        task_static_contact.X_cur = rb_wrapper.pos_L_feet_W;
        task_static_contact.dX_cur = rb_wrapper.vel_L_feet_W;
        task_static_contact.J = rb_wrapper.J_Lfeet_W;
        task_static_contact.dJ = rb_wrapper.dJ_Lfeet_W;
    }
    // NOTE: previously relaxed ONE rotational DOF (ankle roll) out of this
    // task's Jacobian here, via a rank-deficient projector, specifically to
    // free null-space room for task_posture. task_posture was reverted from
    // both kin_task_walk/kin_task_init_walk ("Park Phase 1a", see its
    // declaration comment in KinWBC.h) but this relaxation was left active
    // -- with nothing downstream to consume the freed DOF, it just left the
    // stance foot's ankle-roll rotation completely unconstrained by any
    // task. Confirmed as the actual cause of test_DynWalk_joystick.cpp's
    // WALK-transition blow-up: left_ankle_roll_joint (the stance ankle)
    // spiking from 0 to 14 rad/s within 10ms while every other joint
    // stayed under 1 rad/s, right when the task list switched from
    // kin_task_stand's rigid task_left_contact/task_right_contact (no
    // relaxation) to kin_task_walk's task_static_contact (this relaxed
    // task). Removed -- restore the full rigid 6-dim stance-foot hold.
    task_static_contact.errX = VectorXd::Zero(6);
    task_static_contact.derrX = VectorXd::Zero(6);

    if (footPlanner.legState == LegState::RSt) // right stance -> left swinging
    {
        Eigen::Matrix3d Rcur_L = rb_wrapper.rot_L_feet_W; // diffRot's 2nd arg is a non-const ref, needs an lvalue
        task_lift_foot.X_cur = VectorXd::Zero(6);
        task_lift_foot.X_cur.head<3>() = rb_wrapper.pos_L_feet_W;
        task_lift_foot.X_cur.tail<3>() = diffRot(Eigen::Matrix3d::Identity(), Rcur_L);
        task_lift_foot.dX_cur = rb_wrapper.J_Lfeet_W * rb_wrapper.dq;
        task_lift_foot.J = rb_wrapper.J_Lfeet_W;
        task_lift_foot.dJ = rb_wrapper.dJ_Lfeet_W;
    }
    else // LSt or DSt -> right swinging
    {
        Eigen::Matrix3d Rcur_R = rb_wrapper.rot_R_feet_W;
        task_lift_foot.X_cur = VectorXd::Zero(6);
        task_lift_foot.X_cur.head<3>() = rb_wrapper.pos_R_feet_W;
        task_lift_foot.X_cur.tail<3>() = diffRot(Eigen::Matrix3d::Identity(), Rcur_R);
        task_lift_foot.dX_cur = rb_wrapper.J_Rfeet_W * rb_wrapper.dq;
        task_lift_foot.J = rb_wrapper.J_Rfeet_W;
        task_lift_foot.dJ = rb_wrapper.dJ_Rfeet_W;
    }
    task_lift_foot.errX = task_lift_foot.X_des - task_lift_foot.X_cur;
    task_lift_foot.derrX = task_lift_foot.dX_des - task_lift_foot.dX_cur;

    // task_swing_leg: same Jacobian/state setup as task_lift_foot -- the
    // "walking" task list (kin_task_walk) uses task_swing_leg instead of
    // task_lift_foot; both track the same swinging foot.
    if (footPlanner.legState == LegState::RSt) // right stance -> left swinging
    {
        Eigen::Matrix3d Rcur_L = rb_wrapper.rot_L_feet_W;
        task_swing_leg.X_cur = VectorXd::Zero(6);
        task_swing_leg.X_cur.head<3>() = rb_wrapper.pos_L_feet_W;
        task_swing_leg.X_cur.tail<3>() = diffRot(Eigen::Matrix3d::Identity(), Rcur_L);
        task_swing_leg.dX_cur = rb_wrapper.J_Lfeet_W * rb_wrapper.dq;
        task_swing_leg.J = rb_wrapper.J_Lfeet_W;
        task_swing_leg.dJ = rb_wrapper.dJ_Lfeet_W;
    }
    else // LSt or DSt -> right swinging
    {
        Eigen::Matrix3d Rcur_R = rb_wrapper.rot_R_feet_W;
        task_swing_leg.X_cur = VectorXd::Zero(6);
        task_swing_leg.X_cur.head<3>() = rb_wrapper.pos_R_feet_W;
        task_swing_leg.X_cur.tail<3>() = diffRot(Eigen::Matrix3d::Identity(), Rcur_R);
        task_swing_leg.dX_cur = rb_wrapper.J_Rfeet_W * rb_wrapper.dq;
        task_swing_leg.J = rb_wrapper.J_Rfeet_W;
        task_swing_leg.dJ = rb_wrapper.dJ_Rfeet_W;
    }
    task_swing_leg.errX = task_swing_leg.X_des - task_swing_leg.X_cur;
    task_swing_leg.derrX = task_swing_leg.dX_des - task_swing_leg.dX_cur;

    // posture (12-dim, JOINT space not Cartesian) -- see its declaration
    // comment in KinWBC.h. J is Identity on the actuated-joint tangent-
    // space columns (6..6+12-1, base occupies 0-5) and zero elsewhere, so
    // this task only ever asks for actuated-joint motion, never touches
    // the floating base directly.
    task_posture.X_cur = rb_wrapper.q.segment(7, 12);
    task_posture.dX_cur = rb_wrapper.dq.segment(6, 12);
    task_posture.J = MatrixXd::Zero(12, nv);
    task_posture.J.rightCols(12) = MatrixXd::Identity(12, 12);
    task_posture.dJ = MatrixXd::Zero(12, nv);
    // Fall back to X_des=X_cur (inert, zero pull) if the caller never set
    // posture_nominal_ -- see its comment in KinWBC.h.
    task_posture.X_des = (posture_nominal_.size() == 12) ? posture_nominal_ : task_posture.X_cur;
    task_posture.errX = task_posture.X_des - task_posture.X_cur;
    task_posture.derrX = task_posture.dX_des - task_posture.dX_cur;
}


VectorXd KinWBC::integrateDIY(const VectorXd &qI, const VectorXd &dqI)
{
    VectorXd qRes = VectorXd::Zero(qI.size());
    Vector3d wDes;
    wDes << dqI(3), dqI(4), dqI(5);
    Eigen::Quaterniond quatNow;
    quatNow.x() = qI(3);
    quatNow.y() = qI(4);
    quatNow.z() = qI(5);
    quatNow.w() = qI(6);
    Eigen::Quaterniond quatNew = intQuat(quatNow, wDes);
    qRes = qI;
    qRes(0) += dqI(0);
    qRes(1) += dqI(1);
    qRes(2) += dqI(2);
    qRes(3) = quatNew.x();
    qRes(4) = quatNew.y();
    qRes(5) = quatNew.z();
    qRes(6) = quatNew.w();
    for (int i = 0; i < dqI.size() - 6; i++)
        qRes(7 + i) += dqI(6 + i);
    return qRes;
}

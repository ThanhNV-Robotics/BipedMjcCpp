// Compute desired 

#include "KinWBC.h"
#include "data_type.h"
#include "robot_wrapper.h"
#include <cstdio>
#include <stdexcept>

// constructor
KinWBC::KinWBC()
{
    // // construct stand and walk task, in priority order (index 0 = highest)

    //---------------Stand task---------------------------
    // kin_task_stand is a vector of pointers, so it stores the address of each task
    kin_task_stand.push_back(&task_left_contact);
    kin_task_stand.push_back(&task_right_contact);
    kin_task_stand.push_back(&task_CoMXY); 
    kin_task_stand.push_back(&task_base_rpy);   
    kin_task_stand.push_back(&task_base_height);

    //---------------Init Walk task---------------------------
    kin_task_init_walk.push_back(&task_static_contact);
    kin_task_init_walk.push_back(&task_lift_foot);
    kin_task_init_walk.push_back(&task_CoMXY);    
    kin_task_init_walk.push_back(&task_base_rpy);
    kin_task_init_walk.push_back(&task_base_height);

    //---------------Walking task---------------------------
    kin_task_walk.push_back(&task_static_contact);
    kin_task_walk.push_back(&task_swing_leg);
    kin_task_walk.push_back(&task_CoMXY);
    kin_task_walk.push_back(&task_base_rpy);
    kin_task_walk.push_back(&task_base_height);
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
    // get reference
    updateReference(joyStick, footPlanner, cp_planning);
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
    for (size_t i = 0; i < kin_task.size(); i++)
    {
        Task &task = *kin_task[i];
        if (i == 0) // 1st task in the list has the highest priority
        {
            task.N = MatrixXd::Identity(nv, nv);
            task.Jpre = task.J * task.N;
            task.delta_q = pseudoInv_right_weighted(task.Jpre, task.W) * task.errX;
            task.dq = pseudoInv_right_weighted(task.Jpre, task.W) * task.derrX;
        }
        else
        {
            Task &parent = *kin_task[i - 1];
            task.N = parent.N * (MatrixXd::Identity(parent.Jpre.cols(), parent.Jpre.cols())
                                  - pseudoInv_right_weighted(parent.Jpre, parent.W) * parent.Jpre);
            task.Jpre = task.J * task.N;
            task.delta_q = parent.delta_q + pseudoInv_right_weighted(task.Jpre, task.W)
                                                 * (task.errX - task.J * parent.delta_q);
            task.dq = parent.dq + pseudoInv_right_weighted(task.Jpre, task.W)
                                       * (task.derrX - task.J * parent.dq);
        }
    }

    out_delta_q = kin_task.back()->delta_q;
    out_dq = kin_task.back()->dq;

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

    // base rpy
    task_base_rpy.X_des = VectorXd::Zero(3);
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
    task_base_rpy.W = Eigen::VectorXd::Ones(nv).asDiagonal();
    task_static_contact.W = Eigen::VectorXd::Ones(nv).asDiagonal();
    task_lift_foot.W = Eigen::VectorXd::Ones(nv).asDiagonal();
    task_swing_leg.W = Eigen::VectorXd::Ones(nv).asDiagonal();

    // base height (task is 1-dim: z only -- J_base_W's row 2 is the base's z-row,
    // since J_base_W.block<3,3>(0,0) = I maps base linear-vel dof straight to rows 0-2)
    task_base_height.X_cur = VectorXd::Constant(1, rb_wrapper.pos_base_W(2));
    task_base_height.dX_cur = VectorXd::Constant(1, rb_wrapper.vel_base_W(2));
    task_base_height.J = rb_wrapper.J_base_W.row(2);
    task_base_height.dJ = rb_wrapper.dJ_base_W.row(2);
    task_base_height.errX = task_base_height.X_des - task_base_height.X_cur;
    task_base_height.derrX = task_base_height.dX_des - task_base_height.dX_cur;

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
    // dJ*dq approximates d(Rcur_base)/dt as zero (only valid at the
    // position/velocity IK level this class currently solves at, not yet an
    // acceleration/ddq stage)
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

    // CoMXY (task is 2-dim: x,y only -- Jcom_W's top 2 rows)
    task_CoMXY.X_cur = rb_wrapper.pos_CoM_W.head(2);
    task_CoMXY.dX_cur = rb_wrapper.vel_CoM_W.head(2);
    task_CoMXY.J = rb_wrapper.Jcom_W.topRows(2);
    task_CoMXY.errX = task_CoMXY.X_des - task_CoMXY.X_cur;
    task_CoMXY.derrX = task_CoMXY.dX_des - task_CoMXY.dX_cur;

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
}


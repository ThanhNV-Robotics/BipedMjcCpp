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
    kin_task_stand.push_back(&task_left_contact);
    kin_task_stand.push_back(&task_right_contact);
    kin_task_stand.push_back(&task_base_rpy);
    kin_task_stand.push_back(&task_CoMXY);    
    kin_task_stand.push_back(&task_base_height);
}

void KinWBC::printTaskInfo() {
    for (int i=0;i<kin_task_stand.size();i++)
    {
        printf("-------------\n");
        printf("taskName=%s\n",kin_task_stand[i]->taskName.c_str());
        printf("Priority Order: %d \n", i);
    }
}

void KinWBC::computeWBC_IK (const JoyStickInterpreter &joyStick, FootPlacement &footPlanner, const RobotWrapper& robot_wrapper)
{
    // // Input: robot_wrapper provide computed robot state and kinematic quantity

    // get reference
    updateReference(joyStick, footPlanner);
    // get feedback
    updateCurrent(robot_wrapper);

    // left/right contact + CoMXY only shape the null space -- X_des for these
    // is a placeholder, not a meaningful absolute target, so they inject no
    // position/velocity correction of their own ("hold here, don't disturb"
    // contact-task convention in WBC).
    task_left_contact.errX = VectorXd::Zero(6);
    task_left_contact.derrX = VectorXd::Zero(6);
    task_right_contact.errX = VectorXd::Zero(6);
    task_right_contact.derrX = VectorXd::Zero(6);
    task_CoMXY.errX = VectorXd::Zero(2);
    task_CoMXY.derrX = VectorXd::Zero(2);

    // base_height/base_rpy have real targets, so track the actual error
    task_base_height.errX = task_base_height.X_des - task_base_height.X_cur;
    task_base_height.derrX = task_base_height.dX_des - task_base_height.dX_cur;
    task_base_rpy.errX = task_base_rpy.X_des - task_base_rpy.X_cur;
    task_base_rpy.derrX = task_base_rpy.dX_des - task_base_rpy.dX_cur;

    // recursive null-space priority solve -- position/velocity level only
    // (this is a *kinematic* WBC; no dynamically-consistent/mass-matrix
    // stage here, unlike PriorityTasks::computeAll()). kin_task_stand's
    // vector order IS the priority order, index 0 = highest.
    const int nv = robot_wrapper.model_nv_;
    for (size_t i = 0; i < kin_task_stand.size(); i++)
    {
        Task &task = *kin_task_stand[i];
        if (i == 0)
        {
            task.N = MatrixXd::Identity(nv, nv);
            task.Jpre = task.J * task.N;
            task.delta_q = pseudoInv_right_weighted(task.Jpre, task.W) * task.errX;
            task.dq = pseudoInv_right_weighted(task.Jpre, task.W) * task.derrX;
        }
        else
        {
            Task &parent = *kin_task_stand[i - 1];
            task.N = parent.N * (MatrixXd::Identity(parent.Jpre.cols(), parent.Jpre.cols())
                                  - pseudoInv_right_weighted(parent.Jpre, parent.W) * parent.Jpre);
            task.Jpre = task.J * task.N;
            task.delta_q = parent.delta_q + pseudoInv_right_weighted(task.Jpre, task.W)
                                                 * (task.errX - task.J * parent.delta_q);
            task.dq = parent.dq + pseudoInv_right_weighted(task.Jpre, task.W)
                                       * (task.derrX - task.J * parent.dq);
        }
    }

    out_delta_q = kin_task_stand.back()->delta_q;
    out_dq = kin_task_stand.back()->dq;

    return;

}

void KinWBC::updateReference(const JoyStickInterpreter& joyStick_cmd, FootPlacement& footPlanner_cmd)
{
    // update robot reference task space

    //---------------------Stand----------------------------------------------
    // left_contact
    task_left_contact.deltaX_des = VectorXd::Zero(6);
    task_left_contact.X_des = VectorXd::Zero(6);
    task_left_contact.dX_des = VectorXd::Zero(6);
    task_left_contact.ddX_des = VectorXd::Zero(6);

    // right
    task_right_contact.deltaX_des = VectorXd::Zero(6);
    task_right_contact.X_des = VectorXd::Zero(6);
    task_right_contact.dX_des = VectorXd::Zero(6);
    task_right_contact.ddX_des = VectorXd::Zero(6);

    // CoMXY
    task_CoMXY.deltaX_des = VectorXd::Zero(2); // hold at 0 for a stable standing
    task_CoMXY.X_des = VectorXd::Zero(2);
    task_CoMXY.dX_des = VectorXd::Zero(2);
    task_CoMXY.ddX_des = VectorXd::Zero(2);

    // base heigh    
    task_base_height.deltaX_des = VectorXd::Constant(1, joyStick_cmd.vz_W * this->dt); // scalar
    task_base_height.X_des = VectorXd::Constant(1, joyStick_cmd.pz_W);
    task_base_height.dX_des = VectorXd::Constant(1, joyStick_cmd.vz_W);
    task_base_height.ddX_des = VectorXd::Zero(1); // feedforward acceleration is 0

    // base rpy
    task_base_rpy.deltaX_des = VectorXd::Zero(3);
    task_base_rpy.X_des = VectorXd::Zero(3);
    task_base_rpy.dX_des = VectorXd::Zero(3);
    task_base_rpy.ddX_des = VectorXd::Zero(3);

    return;
}

void KinWBC::updateCurrent (const RobotWrapper& rb_wrapper) // update current task space estimation
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

    // base height (task is 1-dim: z only -- J_base_W's row 2 is the base's z-row,
    // since J_base_W.block<3,3>(0,0) = I maps base linear-vel dof straight to rows 0-2)
    task_base_height.X_cur = VectorXd::Constant(1, rb_wrapper.pos_base_W(2));
    task_base_height.dX_cur = VectorXd::Constant(1, rb_wrapper.vel_base_W(2));
    task_base_height.J = rb_wrapper.J_base_W.row(2);
    task_base_height.dJ = rb_wrapper.dJ_base_W.row(2);

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

    // left contact (6-dim: position + orientation)
    task_left_contact.X_cur = rb_wrapper.pos_L_feet_W;
    task_left_contact.dX_cur = rb_wrapper.vel_L_feet_W;
    task_left_contact.J = rb_wrapper.J_Lfeet_W;
    task_left_contact.dJ = rb_wrapper.dJ_Lfeet_W;

    // right contact (6-dim: position + orientation)
    task_right_contact.X_cur = rb_wrapper.pos_R_feet_W;
    task_right_contact.dX_cur = rb_wrapper.vel_R_feet_W;
    task_right_contact.J = rb_wrapper.J_Rfeet_W;
    task_right_contact.dJ = rb_wrapper.dJ_Rfeet_W;

    // CoMXY (task is 2-dim: x,y only -- Jcom_W's top 2 rows)
    // TODO: dJ not set here -- RobotWrapper doesn't track a CoM Jacobian
    // time-derivative (no dJcom_W member), only dJ_base_W/dJ_Lfeet_W/dJ_Rfeet_W.
    // Fine for position/velocity-level IK; needed if this task reaches the
    // acceleration (ddq) level.
    task_CoMXY.X_cur = rb_wrapper.pos_CoM_W.head(2);
    task_CoMXY.dX_cur = rb_wrapper.vel_CoM_W.head(2);
    task_CoMXY.J = rb_wrapper.Jcom_W.topRows(2);
}


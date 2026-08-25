// Compute desired 

#include "KinWBC.h"
#include "data_type.h"
#include "robot_wrapper.h"
#include <cstdio>
#include <stdexcept>

// constructor
KinWBC::KinWBC()
{
    // // construct stand and walk task
    kin_task_stand.emplace_back(task_left_contact);
    kin_task_stand.emplace_back(task_right_contact);
    kin_task_stand.emplace_back(task_CoMXY);
    kin_task_stand.emplace_back(task_base_height);
}

void KinWBC::printTaskInfo() {
    for (int i=0;i<kin_task_stand.size();i++)
    {
        printf("-------------\n");
        printf("taskName=%s\n",kin_task_stand[i].taskName.c_str());
        printf("Priority Order: %d \n", i);
        printf("Jacobian Index: %d \n", kin_task_stand[i].ee_index);
    }
}

void KinWBC::computeWBC_IK (const JoyStickInterpreter &joyStick, FootPlacement &footPlanner, const RobotWrapper& robot_wrapper)
{
    // // Input: robot_wrapper provide computed robot state and kinematic quantity

    // // get reference
    // updateReference(joyStick, footPlanner);
    // // get feedback
    // updateCurrent(robot_wrapper);

    // // base hieght task
    // task_base_height.errX = task_base_height.X_des - task_base_height.X_cur;
    // task_base_height.derrX = task_base_height.dX_des - task_base_height.dX_cur;


    return;

}

void KinWBC::updateReference(const JoyStickInterpreter& joyStick_cmd, FootPlacement& footPlanner_cmd)
{
    // update robot reference task space

    //---------------------Stand----------------------------------------------
    // base heigh    
    task_base_height.deltaX_des = VectorXd::Constant(1, joyStick_cmd.vz_W * this->dt); // scalar
    task_base_height.X_des = VectorXd::Constant(1, joyStick_cmd.pz_W);
    task_base_height.dX_des = VectorXd::Constant(1, joyStick_cmd.vz_W);
    task_base_height.ddX_des = VectorXd::Zero(1); // feedforward acceleration is 0

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
    task_CoMXY.deltaX_des = VectorXd(0,0); // hold at 0 for a stable standing
    task_CoMXY.X_des = VectorXd(0,0);
    task_CoMXY.dX_des = VectorXd(0,0);
    task_CoMXY.ddX_des = VectorXd(0,0);

    return;
}

void KinWBC::updateCurrent (const RobotWrapper& rb_wrapper) // update current task space estimation
{
    // base height
    task_base_height.X_cur = rb_wrapper.pos_base_W;
    task_base_height.dX_cur = rb_wrapper.vel_base_W;
    // left contact
    task_left_contact.X_cur = rb_wrapper.pos_L_feet_W;
    task_left_contact.dX_cur = rb_wrapper.vel_L_feet_W;
    // right contact
    task_right_contact.X_cur = rb_wrapper.pos_R_feet_W;
    task_right_contact.dX_cur = rb_wrapper.vel_R_feet_W;

    // CoMXY
    task_CoMXY.X_cur = rb_wrapper.pos_CoM_W;
    task_CoMXY.dX_cur = rb_wrapper.vel_CoM_W;
}


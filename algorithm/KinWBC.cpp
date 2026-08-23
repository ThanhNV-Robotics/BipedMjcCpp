// Compute desired 

#include "KinWBC.h"
#include <cstdio>
#include <stdexcept>

// constructor
KinWBC::KinWBC()
{
    // construct stand and walk task
    Task task_left_contact  = Task("left_contact", 1); // input 1 to access to left_feet (jacobian, position, vel)
    Task task_right_contact  = Task("right_contact", 2); // input 2 to access to right_feet (jacobian, position, vel)

    Task task_CoM = Task("CoMXY", 3);  // input 3 to access to Jcom_W in robot_wrapper
    Task base_height = Task("base_height", 0); // input 0 to access to base end-effector
    kin_task_stand.emplace_back(task_left_contact);
    kin_task_stand.emplace_back(task_right_contact);
    kin_task_stand.emplace_back(task_CoM);
    kin_task_stand.emplace_back(base_height);
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

void KinWBC::computeWBC_IK (const RobotWrapper& robot_wrapper)
{
    // Input: robot_wrapper provide computed robot state and kinematic quantity

    // Task definition

    // ---------------------Stand----------------------------------------------

}

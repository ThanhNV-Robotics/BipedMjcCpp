#pragma once
#include "KinWBC.h"

#include <iostream>
#include "gait_scheduler.h"
#include "robot_wrapper.h"


using namespace std;

const string URDF_PATH = "models/urdf/biped_robot_12dof.urdf";
const string XML_PATH = "models/mjcf/biped_robot_floatingbase_12dof.xml";
const string FREE_JOINT_NAME = "floating_base_joint";

int main()
{
    RobotWrapper robot_wrapper = RobotWrapper(URDF_PATH);
    
    KinWBC kin_wbc = KinWBC();

    kin_wbc.printTaskInfo();
    
    return 0;
}
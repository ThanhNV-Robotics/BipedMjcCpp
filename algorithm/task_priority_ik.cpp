#include "task_priority_ik.h"

TaskPriorityIK::TaskPriorityIK()
{

    //  WBC task defined and order build
    ///------------ walk --------------
    kin_tasks_walk.addTask("static_contact"); // contact feet
    kin_tasks_walk.addTask("Base_Pose"); // base height and rpy
    kin_tasks_walk.addTask("swing_leg"); // swing feet

    std::vector<std::string> taskOrder_walk;

    taskOrder_walk.emplace_back("static_contact");
    taskOrder_walk.emplace_back("base_pose");
    taskOrder_walk.emplace_back("swing_leg");

    kin_tasks_walk.buildPriority(taskOrder_walk);
    
    this->kin_tasks_walk.printTaskInfo();

}
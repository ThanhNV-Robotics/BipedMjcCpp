#include "task_priority_ik.h"

TaskPriorityIK::TaskPriorityIK()
{

    //  WBC task defined and order build
    ///------------ stand --------------
    std::vector<std::string> taskOrder_walk = {"static_contact", "CoMXY_HipRPY", "Pz"};

    for (int i = 0 ; i < taskOrder_walk.size(); i++)
    {
        kin_tasks_walk.addTask(taskOrder_walk[i]);
    }
    kin_tasks_walk.buildPriority(taskOrder_walk);
    
    this->kin_tasks_walk.printTaskInfo();

}
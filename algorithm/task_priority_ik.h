#include "priority_tasks.h"
#include "robot_wrapper.h"
#include <iostream>

class TaskPriorityIK

{
    public:
        PriorityTasks kin_tasks_stand, kin_tasks_walk;
        TaskPriorityIK();

    private:
        const double timeStep{0.001};
};
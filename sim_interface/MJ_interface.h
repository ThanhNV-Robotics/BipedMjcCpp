#pragma once

#include <mujoco/mujoco.h>

#include <string>
#include <vector>

// This class use to read robot states (sensors, motor states) from mujoco simulator

class MJ_Interface {

    public:
        int jointNum {0}; // number of joints

    private:

        mjModel *mj_model; // pointer to mjModel struct
        mjData *mj_data; // pointer to mjData struct
};
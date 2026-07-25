#pragma once

#include <mujoco/mujoco.h>
#include <fstream>
#include <string>
#include <vector>
#include "json/json.h" // to process json config file
#include <iostream>

#include "data_bus.h"
// This class use to read robot states (sensors, motor states) or write control force from/to mujoco simulator

class MJ_Interface
{

public:
    int jointNum{0};
    std::vector<double> motor_pos;
    std::vector<double> motor_pos_Old;
    std::vector<double> motor_vel;

    std::vector<std::string> JointName = {}; // this will be initialized with constructor by parsing the json config file

    MJ_Interface(mjModel *mj_modelIn, mjData *mj_dataIn, const char *jsonPath); // constructor
    void updateSensorValues(); // get motor joint states 
    void setMotorsTorque(std::vector<double> &tauIn); // set joint torque
    void printInfo ();
    void printJointPos ();

    std::vector<double> getJointPos ();
    std::vector<double> getJointVel ();
    void dataBusWrite (DataBus &busIn);


private:
    mjModel *mj_model; // pointer to mjModel struct to read/write the data
    mjData *mj_data;   // pointer to mjData struct to read/write the data
    std::vector<int> jntId_qpos, jntId_qvel, jntId_dctl;
};
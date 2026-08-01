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
    std::vector<double> motor_accel;
    std::vector<double> motor_torque;

    double rpy[3]{0}; // roll,pitch and yaw of baselink
    double yaw_simgle;
	int	   yaw_N = 0;
    double baseQuat[4]{0}; // in quat, mujoco order is [w,x,y,z], here we rearrange to [x,y,z,w]
    double f3d[3][2]{0}; // 3D foot-end contact force, L for 1st col, R for 2nd col
    double basePos[3]{0}; // position of baselink, in world frame
    double baseAcc[3]{0};  // acceleration of baselink, in body frame
    double baseAngVel[3]{0}; // angular velocity of baselink, in body frame
    double baseLinVel[3]{0}; // linear velocity of baselink, in body frame
    const std::string baseName="base_link";
    const std::string orientationSensorName="baselink-quat"; // in quat, mujoco order is [w,x,y,z], here we rearrange to [x,y,z,w]
    const std::string velSensorName="baselink-velocity";
    const std::string gyroSensorName="baselink-gyro";
    const std::string accSensorName="baselink-baseAcc";
    const std::string touchSensorName_L="lf-touch"; // touch sensor of left foot
    const std::string touchSensorName_R="rf-touch"; // touch sensor of right foot

    double touch_lf{0}, touch_rf{0}; // touch sensor value (measure the norminal contact force)

    std::vector<std::string> JointName = {}; // this will be initialized with constructor by parsing the json config file

    MJ_Interface(mjModel *mj_modelIn, mjData *mj_dataIn, const char *jsonPath); // constructor
    void updateSensorValues(); // get motor joint states 
    void setMotorsTorque(std::vector<double> &tauIn); // set joint torque
    void printInfo ();
    void printJointPos ();

    std::vector<double> getJointPos ();
    std::vector<double> getJointVel ();
    std::vector<double> getJointAccel ();
    std::vector<double> getJointTorque ();
    void dataBusWrite (DataBus &busIn);


private:
    mjModel *mj_model; // pointer to mjModel struct to read/write the data
    mjData *mj_data;   // pointer to mjData struct to read/write the data
    std::vector<int> jntId_qpos, jntId_qvel, jntId_dctl; // joint id to read position, vel, acceleration, torque

    int orientataionSensorId;
    int velSensorId;
    int gyroSensorId;
    int accSensorId;
    int baseBodyId;

    int touchSensorId_L, touchSensorId_R; // touch sensor id for left and right foot

    double timeStep{0.001}; // second
    bool isIni{false};
};
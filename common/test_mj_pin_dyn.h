#pragma once

#include "data_type.h"
#include "robot_wrapper.h"
#include "mujoco/mujoco.h"
#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>

#include <random>

class TestDyn
{
    public:
        TestDyn(const std::string& urdf_path, const std::string& xml_path);
        void loadXml(const std::string& xml_in, mjModel*& model, mjData*& data);
        

        bool testPinMjcJacobians (const std::string link_name);
        void updateMujocoState (RobotConfiguration q, RobotSpatialVelocity v);
        Vector6d mjcComputeBodyLinkVel (RobotConfiguration q, RobotSpatialVelocity v, const std::string link_name);

        // supporting methods
        double generate_random (double min, double max);
        RobotConfiguration generateRandomConfiguration (const RobotWrapper& robot_wrapper);
        RobotSpatialVelocity generateRandomRobotSpatialVelocity (const RobotWrapper& robot_wrapper);
    
    private:
        RobotWrapper robot_wrapper_;
        mjModel* mj_model_;
        mjData* mj_data_;
};
//pinocchio
#pragma once

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <ostream>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

// mujoco
#include <mujoco/mujoco.h>
#include <cstdio>
#include <iostream>
#include <string>

// robot wrapper
#include "data_type.h"
#include "robot_wrapper.h"

#include "matplotlibcpp.h"

using namespace std;
namespace pin = pinocchio;
namespace plt = matplotlibcpp;

void loadXml(const std::string& xml_in, mjModel*& model, mjData*& data)
{
    // Delete old data/model
    if (data != nullptr)
    {
        mj_deleteData(data);
        data = nullptr;
    }

    if (model != nullptr)
    {
        mj_deleteModel(model);
        model = nullptr;
    }

    // Error buffer
    char error[1000] = {0};

    // Load new model
    model = mj_loadXML(xml_in.c_str(), nullptr, error, sizeof(error));

    // Check loading result
    if (model == nullptr)
    {
        std::cerr << "Failed to load MuJoCo XML:\n" << error << std::endl;
    }

    // Create corresponding data
    data = mj_makeData(model);

    if (data == nullptr)
    {
        std::cerr << "Failed to create mjData." << std::endl;
        mj_deleteModel(model);
        model = nullptr;
    }
}

void printXmlInfo (const mjModel* model)
{
    for (int i = 0; i < model->njnt; i++)
    {
        const char* joint_name = mj_id2name(model, mjOBJ_JOINT, i);
        const int jnt_id = mj_name2id(model, mjOBJ_JOINT, joint_name);
        const int qpos_address = model->jnt_qposadr[i];

        std::cout
            << "Joint ID: " << jnt_id
            << " | Name: " << (joint_name ? joint_name : "unnamed")
            << " | qpos address: " << qpos_address
            << " | qvel_adr: "<< model->jnt_dofadr[jnt_id]
            << '\n';
    }
    double total_mass = 0;
    for (int i = 0; i < model->nbody; i++)
    {
        total_mass += model->body_mass[i];
    }
    std::cout << "Total robot mass: "
          << total_mass
          << " kg" << std::endl;
}

const string URDF_PATH = "models/urdf/biped_robot_12dof.urdf";
const string XML_PATH = "models/mjcf/biped_robot_floatingbase_12dof.xml";

mjModel* mj_model;
mjData* mj_data;

using namespace std;

int main ()
{
    // load urdf by pinocchio
    RobotWrapper robot_wrapper(URDF_PATH);
    robot_wrapper.printModelInfo();

    // load xml by mujoco
    // cout<<"Mujoco model info: "<<endl;
    loadXml(XML_PATH, mj_model, mj_data);
    printXmlInfo(mj_model);

    return 0;
}
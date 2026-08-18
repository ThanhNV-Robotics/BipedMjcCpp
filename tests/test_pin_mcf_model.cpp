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
    for (int joint_id = 0; joint_id<= model->njnt; joint_id++)
    {
        const char* joint_name = mj_id2name(model, mjOBJ_JOINT, joint_id);
        const int qpos_address = model->jnt_qposadr[joint_id];

        std::cout
            << "Joint ID: " << joint_id
            << " | Name: " << (joint_name ? joint_name : "unnamed")
            << " | qpos address: " << qpos_address
            << '\n';
    }
    double total_mass = 0;
    for (int i = 0; i< model->nbody; i++)
    {
        total_mass += model->body_mass[i];
    }
    std::cout << "Total robot mass: "
          << total_mass
          << " kg" << std::endl;
}

double generate_random (double min, double max)
{
    // generate a random legs' joint position and velocity within the limit
    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_real_distribution<double> value_distance(min, max);
    double output = value_distance(gen);

    return output;
}

RobotConfiguration generateRandomConfiguration (const RobotWrapper& robot_wrapper)
{
    const double PI = 3.14159;
    const Vector3d base_pos_min = {-0.3,-0.3,0.8};
    const Vector3d base_pos_max = {0.3,0.3,1};
    const Vector3d base_rpy_min = {-PI/6,-PI/6,-PI/6};
    const Vector3d base_rpy_max = {PI/6,PI/6,PI/6};

    const Vector3d base_linear_vel_max = {2,2,2};
    const Vector3d base_angular_vel_max = {10,10,10};

    const int nv = robot_wrapper.model_nv_;

    // generate a random base position
    double qb_x = generate_random(base_pos_min[0], base_pos_max[0]);
    double qb_y = generate_random(base_pos_min[1], base_pos_max[1]);
    double qb_z = generate_random(base_pos_min[2], base_pos_max[2]);

    // generate a random base RPY/orientation
    double qb_R = generate_random(base_rpy_min[0], base_rpy_max[0]);
    double qb_P = generate_random(base_rpy_min[1], base_rpy_max[1]);
    double qb_Y = generate_random(base_rpy_min[2], base_rpy_max[2]);
    Quat base_quat = AngleAxis(qb_Y, Vector3d::UnitZ())
                    * AngleAxis(qb_P, Vector3d::UnitY())
                    * AngleAxis(qb_R, Vector3d::UnitX()); // convert to quaternion

    // generate a randome joint position
    VectorXd qj = VectorXd::Zero(robot_wrapper.model_na_);
    for (int j = 0; j < robot_wrapper.model_na_; j++)
    {
        const double joint_pos_min = robot_wrapper.min_joint_pos_[j];
        const double joint_pos_max = robot_wrapper.max_joint_pos_[j];
        qj[j] = generate_random(joint_pos_min, joint_pos_max);
    }    

    // pack to RobotConfiguration
    RobotConfiguration q_config(robot_wrapper.model_na_);
    q_config.pos_b_W = Vector3d(qb_x, qb_y, qb_z);
    q_config.quat_b_W = base_quat;
    q_config.qj = qj;

    return q_config;
}

RobotSpatialVelocity generateRandomRobotSpatialVelocity (const RobotWrapper& robot_wrapper)
{
    // generate random base linear velocity
    Vector3d base_lin_vel;
    base_lin_vel[0] = generate_random(-2, 2);
    base_lin_vel[1] = generate_random(-2, 2);
    base_lin_vel[2] = generate_random(-2, 2);

    // generate randome base angular velocity
    Vector3d base_ang_vel;
    base_ang_vel[0] = generate_random(-10, 10);
    base_ang_vel[1] = generate_random(-10, 10);
    base_ang_vel[2] = generate_random(-10, 10);

    // generate random joint velocity
    VectorXd dqj = VectorXd::Zero(robot_wrapper.model_na_);
    for (int i = 0; i <= robot_wrapper.model_na_; i++)
    {
        dqj[i] = generate_random(-robot_wrapper.joint_vel_limit_[i], robot_wrapper.joint_vel_limit_[i]);
    }

    // stack to RobotSpatialVelocity
    RobotSpatialVelocity v(robot_wrapper.model_na_);
    v.vb_W = base_lin_vel;
    v.wb_W = base_ang_vel;
    v.dq_j = dqj;
    return v;
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
    // robot_wrapper.printModelInfo();

    // load xml by mujoco
    // cout<<"Mujoco model info: "<<endl;
    loadXml(XML_PATH, mj_model, mj_data);
    printXmlInfo(mj_model);

    RobotConfiguration robot_configuration(robot_wrapper.model_na_);
    RobotSpatialVelocity robot_spatial_vel(robot_wrapper.model_nv_);

    // Test Jacobian computation
    const int n_sample = 50;
    std::vector<double> v_Lf_x_W, v_Lf_y_W, v_Lf_z_W;
    std::vector<int> sample_plot;

    for (int i = 0 ; i < n_sample; i++)
    {
         // Generate a random configuration
        robot_configuration = generateRandomConfiguration(robot_wrapper);
         // Generate a random robot spatial velocity    
        robot_spatial_vel = generateRandomRobotSpatialVelocity(robot_wrapper);
         // Update robot wrapper state
        robot_wrapper.updateRobotState(robot_configuration, robot_spatial_vel);
         // Compute Jacobians
        robot_wrapper.computeJacobians();
        Jacobian6 J_L_Feet_W = robot_wrapper.J_Lfeet_W; // Left feet Jacobian in World frame

        // Compute global feet velocity using Jacobian camputed by pin
        VectorXd v_L_feet_W = VectorXd::Zero(6);
        v_L_feet_W = J_L_Feet_W * robot_spatial_vel.getFlatVelocityVector();


        // Compute velocity using mj_forward
        const string FREE_JOINT_NAME = "floating_base_joint";
        const int free_joint_id = mj_name2id(mj_model, mjOBJ_JOINT, FREE_JOINT_NAME.c_str());
        const int free_qpos_adr = mj_model->jnt_qposadr[free_joint_id];
        const int free_qvel_adr = mj_model->jnt_dofadr[free_joint_id]; 
        
        // assign qpos
        mj_data->qpos[free_qpos_adr + 0] = robot_configuration.pos_b_W[0];
        mj_data->qpos[free_qpos_adr + 1] = robot_configuration.pos_b_W[1];
        mj_data->qpos[free_qpos_adr + 2] = robot_configuration.pos_b_W[2];
        // MuJoCo free-joint quat order is (w,x,y,z); Pinocchio's coeffs() is (x,y,z,w)
        mj_data->qpos[free_qpos_adr + 3] = robot_configuration.quat_b_W.w();
        mj_data->qpos[free_qpos_adr + 4] = robot_configuration.quat_b_W.x();
        mj_data->qpos[free_qpos_adr + 5] = robot_configuration.quat_b_W.y();
        mj_data->qpos[free_qpos_adr + 6] = robot_configuration.quat_b_W.z();

        for (int j = 0; j < robot_wrapper.model_na_; j++)
        {
            mj_data->qpos[7+j] = robot_configuration.qj[j];
        }
        
        // assign qvel

        Vector3d base_lin_vel_world = base_quat * base_lin_vel;
        mj_data->qvel[free_qvel_adr + 0] = base_lin_vel_world.x();
        mj_data->qvel[free_qvel_adr + 1] = base_lin_vel_world.y();
        mj_data->qvel[free_qvel_adr + 2] = base_lin_vel_world.z();
        mj_data->qvel[free_qvel_adr + 3] = base_ang_vel.x();
        mj_data->qvel[free_qvel_adr + 4] = base_ang_vel.y();
        mj_data->qvel[free_qvel_adr + 5] = base_ang_vel.z();
        for (int j = 0; j < robot_wrapper.model_na_; j++)
        {
            mj_data->qvel[act_qvel_adr[j]] = dqj[j];
        }
        mj_forward(mj_model, mj_data);

        v_Lf_x_W.push_back(v_L_feet_W[0]);
        v_Lf_y_W.push_back(v_L_feet_W[1]);
        v_Lf_z_W.push_back(v_L_feet_W[2]);      
        sample_plot.push_back(i); 
    }

    // Ploting stuffs
    plt::figure();
    plt::named_plot("pinocchio", sample_plot, v_Lf_x_W, "--");
    plt::xlabel("sample");
    plt::ylabel(std::string("left foot v_x-W"));
    plt::legend();
    plt::show();
    

    // // Compute left feet velocity in global frame using Jacobian
    // VectorXd L_feet_vel_W = VectorXd::Zero(6);

    
    // // // Test some computing function of robot wrapper
    // // cout<<"Compute robot Dynamic terms"<<endl;
    // // robot_wrapper.computeDyn();
    // // cout<<"M(q)"<<endl<<robot_wrapper.dyn_M<<endl;
    // // cout<<"G(q)"<<endl<<robot_wrapper.dyn_G<<endl;


    // // cout<<"Random robot configuration: "<< endl;
    // // cout<<"Random base position: "<<endl<<robot_configuration.pos_b_W <<endl;
    // // cout<<"Random base quaternion: "<<endl<<robot_configuration.quat_b_W <<endl;
    // // cout<<"Random joint position: "<<endl<<robot_configuration.qj<<endl;

    // // cout<<"Random robot spatial velocity: "<< endl;
    // // cout<<"Random base linear velocity: "<<endl<<robot_spatial_vel.vb_W <<endl;
    // // cout<<"Random base angular velocity: "<<endl<<robot_spatial_vel.wb_W <<endl;
    // // cout<<"Random joint velocity: "<<endl<<robot_spatial_vel.dq_j<<endl;

    return 0;
}
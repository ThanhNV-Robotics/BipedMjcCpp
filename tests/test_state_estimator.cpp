/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024-2025 Humanoid Robot (Shanghai) Co., Ltd.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <algorithm>
#include <cstdio>
#include <iostream>
#include "useful_math.h"
#include "GLFW_callbacks.h"
#include "MJ_interface.h"
#include "PVT_ctrl.h"
#include "pino_kin_dyn.h"
#include "data_logger.h"
#include "useful_math.h"

#include "MyStateEstimator.h"


//************************
// main function
int main(int argc, const char** argv)
{
    std::cout << "Program Starts, Loading Mujoco xml model\n";
    const std::string MODEL_DIR = "models/mjcf"; // path to mujoco xml model

    //-------------------------------------------------------------------
    // Compile mujoco xml model
    //-------------------------------------------------------------------
    const std::string model_path = MODEL_DIR + "/scene_floatingbase_12dof.xml";
    // const std::string model_path = (argc > 1) ? argv[1] : model_path; // if input model path in the arg then use that path
    std::cout << "Input model path: " + model_path + "\n";
    char loadError[1024] = ""; // character array, size 1024
    // load/compile xml model
    // model_path.c_str() return a read-only pointer to const std::string model_path
    mjModel *mj_model = mj_loadXML(model_path.c_str(), nullptr, loadError, sizeof(loadError)); // pointer to mjModel struct
    
    

    if (!mj_model)
    {
        std::fprintf(stderr, "failed to load %s: %s\n", model_path.c_str(), loadError);
        return 1;
    }
    mjData *mj_data = mj_makeData(mj_model); // pointer to mjData struct

    mjModel *stateUI_model = mj_loadXML(model_path.c_str(), nullptr, loadError, sizeof(loadError)); // pointer to mjModel struct
    mjData *stateUI_data = mj_makeData(stateUI_model); // pointer to mjData struct

    std::cout << "Compile mujoco xml done\n";
    

    // ini classes
    const std::string joint_ctrl_config_path = "config/12dof_joint_config.json";
    UIctr uiController(mj_model,mj_data);   // UI control for Mujoco

    // ui for state estimation
    UIctr stateUI(stateUI_model, stateUI_data);

    MJ_Interface mj_interface(mj_model, mj_data, joint_ctrl_config_path.c_str()); // data interface for Mujoco
    // print out xml model info
    std::printf("MuJoCo xml model info: \n");
    mj_interface.printInfo();

    const std::string urdf_path = "models/urdf/biped_robot_12dof.urdf";
    Pin_KinDyn kinDynSolver(urdf_path.c_str()); // kinematics and dynamics solver

    DataBus RobotState(kinDynSolver.model_nv); // data bus

    PVT_Ctr pvtCtr(mj_model->opt.timestep,joint_ctrl_config_path.c_str());// PVT joint control
    
    // FootPlacement footPlacement; // foot-placement planner
    // JoyStickInterpreter jsInterp(mj_model->opt.timestep); // desired baselink velocity generator
    DataLogger logger("record/wbc_walk_control.log"); // data logger
    StateEstimator state_estimator = StateEstimator(mj_model->opt.timestep, true);

    // variables ini
    double stand_legLength = 0.75; // desired baselink height
    double foot_height = 0.07; // distance between the foot ankel joint and the bottom
    double  xv_des = 0.7;  // desired velocity in x direction

    RobotState.width_hips = 0.334;
    
    //mju_copy(mj_data->qpos, mj_model->key_qpos, mj_model->nq*1); // set ini pos in Mujoco
    int model_nv=kinDynSolver.model_nv;

    // // ini position and posture for foot-end
    std::vector<double> motor_pos_des(model_nv - 6, 0); //=12, -6 to exclude the floating base dof in pinocchio model
    std::vector<double> motor_pos_cur(model_nv - 6, 0);
    std::vector<double> motor_vel_des(model_nv - 6, 0);
    std::vector<double> motor_vel_cur(model_nv - 6, 0);
    std::vector<double> motor_tau_des(model_nv - 6, 0);
    std::vector<double> motor_tau_cur(model_nv - 6, 0);
    Eigen::Vector3d fe_l_pos_L_des = {0.0, RobotState.width_hips / 2, -stand_legLength};  // desired left feet pos
    Eigen::Vector3d fe_r_pos_L_des = {0.0, -RobotState.width_hips / 2, -stand_legLength}; // desired right feet pos

    Eigen::Vector3d fe_l_eul_L_des = {0.0, 0.0, 0.0};
    Eigen::Vector3d fe_r_eul_L_des = {0.0, 0.0, 0.0};
    Eigen::Matrix3d fe_l_rot_des = eul2Rot(fe_l_eul_L_des(0), fe_l_eul_L_des(1), fe_l_eul_L_des(2));
    Eigen::Matrix3d fe_r_rot_des = eul2Rot(fe_r_eul_L_des(0), fe_r_eul_L_des(1), fe_r_eul_L_des(2));

    std::cout << "Init variable done\n";
    // solving inverse kinematics
    auto resLeg = kinDynSolver.computeInK_Leg(fe_l_rot_des, fe_l_pos_L_des, fe_r_rot_des, fe_r_pos_L_des);

    Eigen::VectorXd qIniDes=Eigen::VectorXd::Zero(mj_model->nq,1);
    qIniDes.block(7, 0, mj_model->nq - 7, 1) = resLeg.jointPosRes;
    
    // Precompute each joint's qpos address on both mj_model (the real,
    // simulated robot) and stateUI_model (a separate mjData used purely to
    // visualize the state estimate) by name, via kinDynSolver.ikJointNames --
    // rather than trusting RobotState.q(7..18), which comes from
    // DataBus::updateQ() assuming motor_pos_cur is already in Pin_KinDyn's
    // joint order, when MJ_Interface's order is actually alphabetical (from
    // its JSON config's keys).
    std::vector<int> mjQposAdr(kinDynSolver.ikJointNames.size()), stateUiQposAdr(kinDynSolver.ikJointNames.size());
    for (size_t i = 0; i < kinDynSolver.ikJointNames.size(); i++)
    {
        int jid = mj_name2id(mj_model, mjOBJ_JOINT, kinDynSolver.ikJointNames[i].c_str());
        mjQposAdr[i] = mj_model->jnt_qposadr[jid];
        int jidState = mj_name2id(stateUI_model, mjOBJ_JOINT, kinDynSolver.ikJointNames[i].c_str());
        stateUiQposAdr[i] = stateUI_model->jnt_qposadr[jidState];
    }

    // // register variable name for data logger
    // logger.addIterm("simTime", 1);
    // logger.addIterm("motor_pos_cur",model_nv-6);
    // logger.addIterm("motor_vel_cur",model_nv-6);
    // logger.addIterm("rpy",3);
    // logger.addIterm("fL",3);
    // logger.addIterm("fR",3);
    // logger.addIterm("basePos",3);
    // logger.addIterm("baseLinVel",3);
    // logger.addIterm("baseAcc",3);
    // logger.addIterm("baseAngVel",3);
    // logger.finishItermAdding();

    /// ----------------- sim Loop ---------------
    double simEndTime=30;
    mjtNum simstart = mj_data->time;
    double simTime = mj_data->time;
    double startSteppingTime=3;
    double startWalkingTime=5;
    int count = 0;

    std::vector<int> stateUiQposAdr_motorOrder(mj_interface.JointName.size());
    std::vector<int> stateUiQvelAdr_motorOrder(mj_interface.JointName.size());
    for (size_t i = 0; i < mj_interface.JointName.size(); i++)
    {
        int jid = mj_name2id(stateUI_model, mjOBJ_JOINT, mj_interface.JointName[i].c_str());
        stateUiQposAdr_motorOrder[i] = stateUI_model->jnt_qposadr[jid];
        stateUiQvelAdr_motorOrder[i] = stateUI_model->jnt_dofadr[jid];
    }

    // for ramping the standing
    const double rampDuration = 2.0;
    double rampFrac = std::min(simTime / rampDuration, 1.0);
    Eigen::VectorXd rampedJointPos = rampFrac * resLeg.jointPosRes;

    // init UI: GLFW
    uiController.iniGLFW();
    uiController.disableTracking(); // enable viewpoint tracking of the body 1 of the robot
    uiController.createWindow("Demo",false);

    // Create another ui to visualize the state estimation
    stateUI.iniGLFW();
    stateUI.disableTracking(); // enable viewpoint tracking of the body 1 of the robot
    stateUI.createWindow("State Estimation",false);

    while( !glfwWindowShouldClose(uiController.window))
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        simstart=mj_data->time;
        while( mj_data->time - simstart < 1.0/60.0 && uiController.runSim) // press "1" to pause and resume, "2" to step the simulation
        {
            mj_step(mj_model, mj_data);
            uiController.applyPerturbation();

            simTime=mj_data->time;
            // update robot state from mujoco simulator
            mj_interface.updateSensorValues();
            mj_interface.dataBusWrite(RobotState); // also calls RobotState.updateQ()

            // forward kinematics: needed to populate RobotState.fe_l_pos_L /
            // fe_r_pos_L / fe_l_vel_L / fe_r_vel_L, which is what
            // state_estimator.getSensorMeansurement() below reads the foot
            // position/velocity measurement from
            kinDynSolver.dataBusRead(RobotState);
            kinDynSolver.computeJ_dJ();
            kinDynSolver.dataBusWrite(RobotState);

            // feed the EKF: touch sensors for contact detection, then the
            // rest of the measurement (imu, joints, foot pos/vel), then step
            state_estimator.getSensorMeansurement(RobotState);
            state_estimator.update(RobotState);

            Eigen::Matrix<double, 4,1> imu_quat = state_estimator.getImuquaternion();
            Eigen::Matrix<double, 3,1> basePos_est = state_estimator.getBasePosEst();
            // visualize the estimate: write it into stateUI_data and run FK
            stateUI_data->qpos[0] = basePos_est(0);
            stateUI_data->qpos[1] = basePos_est(1);
            stateUI_data->qpos[2] = basePos_est(2);
            stateUI_data->qpos[3] = imu_quat(3); // w
            stateUI_data->qpos[4] = imu_quat(0); // x
            stateUI_data->qpos[5] = imu_quat(1); // y
            stateUI_data->qpos[6] = imu_quat(2); // z

            Eigen::Matrix<double, 12,1> qj_est = state_estimator.get_qj();
            Eigen::Matrix<double, 12,1> qjd_est = state_estimator.get_qjd();

            for (size_t i = 0; i < mj_interface.JointName.size(); i++)
            {
                stateUI_data->qpos[stateUiQposAdr_motorOrder[i]] = qj_est(i);
                stateUI_data->qvel[stateUiQvelAdr_motorOrder[i]] = qjd_est(i);
            }

            mj_forward(stateUI_model, stateUI_data); // kinematics only, no dynamics step

            rampFrac = std::min(simTime / rampDuration, 1.0);
            rampedJointPos = rampFrac * resLeg.jointPosRes;
            RobotState.motors_pos_des = kinDynSolver.mapJointVecToOrder(rampedJointPos, pvtCtr.getMotorNames());
            RobotState.motors_vel_des= motor_vel_des;
            RobotState.motors_tor_des= motor_tau_des;

            pvtCtr.dataBusRead(RobotState); // to update joint command
            pvtCtr.calMotorsPVT(); // calculate joint torque

            pvtCtr.dataBusWrite(RobotState); // set to RobotState
            mj_interface.setMotorsTorque(RobotState.motors_tor_out); // Set joint torque to mujoco

            // printing
            count++;
            if (count >= 100)
            {
                count = 0;
                std::printf("base pos est: % .3f % .3f % .3f | quat w: %.3f x: %.3f y: %.3f z: %.3f\n",
                            basePos_est(0), basePos_est(1), basePos_est(2),
                            imu_quat(3), imu_quat(0), imu_quat(1), imu_quat(2));
            }
        }

        uiController.updateScene();
        stateUI.updateScene();
    }

    // free visualization storage
    uiController.Close();
    stateUI.Close();

    return 0;
}
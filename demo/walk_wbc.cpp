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

#include "StateEst.h"


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

    // WBC_priority(int model_nv_In, int QP_nvIn, int QP_ncIn, double miu_In, double dt);
    // QP_nv = 6+12 number of decision variables that the QP optimizes over
    // 6:a correction to the floating base's acceleration (delta_ddq/delta_r — the 6 DOF of the torso's free-flyer joint: 3 linear + 3 angular).
    // 12: a correction to the contact reaction forces at both feet (delta_Fr — 6 per foot: 3 force + 3 moment,
    // Fr_ff is explicitly documented as "12*1, [fe_L, fe_R]" in the header).
    // QP_ncIn: number of constraints
    // double miu_In is the friction coefficient to build friction cone constraint
    // WBC_priority WBC_solv(kinDynSolver.model_nv, 18, 22, 0.7, mj_model->opt.timestep); // WBC solver
    
    
    // GaitScheduler gaitScheduler(0.4, mj_model->opt.timestep); // gait scheduler
    // GaitScheduler::step() re-triggers its "first step" init block every
    // tick while motionState==Stand (isIni gets reset then immediately
    // re-satisfies "if (!isIni && start_walk)"), which would force
    // legState=LSt (single-leg stance) forever unless start_walk is off.
    // Leave this false until actually starting to walk (GaitScheduler::start()
    // sets it back to true) -- so legState stays at its default DSt
    // (double support, both feet in contact) while just standing.
    // gaitScheduler.start_walk = false;

    PVT_Ctr pvtCtr(mj_model->opt.timestep,joint_ctrl_config_path.c_str());// PVT joint control
    
    // FootPlacement footPlacement; // foot-placement planner
    // JoyStickInterpreter jsInterp(mj_model->opt.timestep); // desired baselink velocity generator
    DataLogger logger("record/wbc_walk_control.log"); // data logger
    StateEst StateModule(mj_model->opt.timestep);

    // variables ini
    double stand_legLength = 0.75; // desired baselink height
    double foot_height = 0.07; // distance between the foot ankel joint and the bottom
    double  xv_des = 0.7;  // desired velocity in x direction

    RobotState.width_hips = 0.334;
    
    //mju_copy(mj_data->qpos, mj_model->key_qpos, mj_model->nq*1); // set ini pos in Mujoco
    int model_nv=kinDynSolver.model_nv;

    // // ini position and posture for foot-end
    std::vector<double> joint_pos_des(model_nv - 6, 0); //=12, -6 to exclude the floating base dof in pinocchio model
    std::vector<double> joint_pos_cur(model_nv - 6, 0);
    std::vector<double> joint_vel_des(model_nv - 6, 0);
    std::vector<double> joint_vel_cur(model_nv - 6, 0);
    std::vector<double> joint_tau_des(model_nv - 6, 0);
    std::vector<double> joint_tau_cur(model_nv - 6, 0);
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
    // DataBus::updateQ() assuming joint_pos_cur is already in Pin_KinDyn's
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
    // logger.addIterm("joint_pos_cur",model_nv-6);
    // logger.addIterm("joint_vel_cur",model_nv-6);
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

    // init UI: GLFW
    uiController.iniGLFW();
    uiController.disableTracking(); // enable viewpoint tracking of the body 1 of the robot
    uiController.createWindow("Demo",false);
    // real-time plot of the foot touch sensors (lf-touch, rf-touch)
    const char* touchLineNames[2] = {"lf-touch", "rf-touch"};
    const float touchLineColors[2][3] = {{1, 0, 0}, {0, 0, 1}};
    uiController.initSensorFigure("Foot Touch Sensors", touchLineNames, touchLineColors, 2);
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
            // printf("-------------%.3f s------------\n",simTime);
            mj_interface.updateSensorValues();
            mj_interface.dataBusWrite(RobotState);

            StateModule.set(RobotState);
            StateModule.update();
            StateModule.get(RobotState); // overwrites RobotState.q(0..6)/base_pos/base_rot/base_rpy with the estimate

            // ---- visualize the estimate: write it into stateUI_data and run FK ----
            // position: matches directly (Pinocchio and MuJoCo both list it first)
            stateUI_data->qpos[0] = RobotState.q(0);
            stateUI_data->qpos[1] = RobotState.q(1);
            stateUI_data->qpos[2] = RobotState.q(2);
            // orientation: Pinocchio stores (x,y,z,w) at q(3..6); MuJoCo's qpos wants (w,x,y,z)
            stateUI_data->qpos[3] = RobotState.q(6);
            stateUI_data->qpos[4] = RobotState.q(3);
            stateUI_data->qpos[5] = RobotState.q(4);
            stateUI_data->qpos[6] = RobotState.q(5);
            // joints: the estimator doesn't touch these -- show the real
            // (measured) joint angles, read directly by name off the actual
            // simulated robot, not through RobotState.q's mismatched order.
            for (size_t i = 0; i < kinDynSolver.ikJointNames.size(); i++)
                stateUI_data->qpos[stateUiQposAdr[i]] = mj_data->qpos[mjQposAdr[i]];
            mj_forward(stateUI_model, stateUI_data); // kinematics only, no dynamics step

            // Enter here functions to send actuator commands, like:
            // arm-l: 0-6, arm-r: 7-13, head: 14,15, waist: 16-18, leg-l: 19-24, leg-r: 25-30
            if (simTime > startWalkingTime) {
                // jsInterp.setWzDesLPara(0, 1);
                // jsInterp.setVxDesLPara(xv_des, 2.0); // jsInterp.setVxDesLPara(0.9,1);
                // RobotState.motionState = DataBus::Walk; // start walking
            }

            // // get the final joint command
            if (simTime<=startSteppingTime){
                // Ramp linearly from 0 to the IK-solved standing pose over
                // rampDuration seconds, rather than stepping straight to it
                // at t=0 -- a step here would be a large sudden position
                // error for PVT_Ctr's PD loop, i.e. a torque impulse.
                const double rampDuration = 2.0;
                double rampFrac = std::min(simTime / rampDuration, 1.0);
                Eigen::VectorXd rampedJointPos = rampFrac * resLeg.jointPosRes;
                RobotState.joint_pos_des = kinDynSolver.mapJointVecToOrder(rampedJointPos, pvtCtr.getMotorNames());
                RobotState.joint_vel_des= joint_vel_des;
                RobotState.joint_tor_des= joint_tau_des;
            }
            else
            {

            }

            pvtCtr.dataBusRead(RobotState);
            pvtCtr.calMotorsPVT();
            
            pvtCtr.dataBusWrite(RobotState);
            mj_interface.setMotorsTorque(RobotState.joint_tor_out);

        }

        // update sensor figure
        double touchVals[2] = {mj_interface.touch_lf, mj_interface.touch_rf};
        uiController.updateSensorFigure(mj_data->time, touchVals, 2);

        uiController.updateScene();
        stateUI.updateScene();
    }

//    // free visualization storage
    uiController.Close();
    stateUI.Close();

    return 0;
}
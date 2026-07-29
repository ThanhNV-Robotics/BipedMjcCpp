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
#include "wbc_priority.h"
#include "gait_scheduler.h"
#include "foot_placement.h"
#include "joystick_interpreter.h"
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
    WBC_priority WBC_solv(kinDynSolver.model_nv, 18, 22, 0.7, mj_model->opt.timestep); // WBC solver
    
    
    GaitScheduler gaitScheduler(0.4, mj_model->opt.timestep); // gait scheduler
    // GaitScheduler::step() re-triggers its "first step" init block every
    // tick while motionState==Stand (isIni gets reset then immediately
    // re-satisfies "if (!isIni && start_walk)"), which would force
    // legState=LSt (single-leg stance) forever unless start_walk is off.
    // Leave this false until actually starting to walk (GaitScheduler::start()
    // sets it back to true) -- so legState stays at its default DSt
    // (double support, both feet in contact) while just standing.
    gaitScheduler.start_walk = false;

    PVT_Ctr pvtCtr(mj_model->opt.timestep,joint_ctrl_config_path.c_str());// PVT joint control
    
    FootPlacement footPlacement; // foot-placement planner
    JoyStickInterpreter jsInterp(mj_model->opt.timestep); // desired baselink velocity generator
    DataLogger logger("record/wbc_walk_control.log"); // data logger
    StateEst StateModule(mj_model->opt.timestep);

    // variables ini
    double stand_legLength = 0.75; // desired baselink height
    double foot_height = 0.07; // distance between the foot ankel joint and the bottom
    double  xv_des = 0.7;  // desired velocity in x direction

    RobotState.width_hips = 0.334;
    footPlacement.kp_vx = 0.03;
    footPlacement.kp_vy = 0.035;
    footPlacement.kp_wz = 0.03;
    footPlacement.stepHeight = 0.12;
    footPlacement.legLength=stand_legLength;
    //mju_copy(mj_data->qpos, mj_model->key_qpos, mj_model->nq*1); // set ini pos in Mujoco
    int model_nv=kinDynSolver.model_nv;

    // // ini position and posture for foot-end
    std::vector<double> motors_pos_des(model_nv - 6, 0); //=12, -6 to exclude the floating base dof in pinocchio model
    std::vector<double> motors_pos_cur(model_nv - 6, 0);
    std::vector<double> motors_vel_des(model_nv - 6, 0);
    std::vector<double> motors_vel_cur(model_nv - 6, 0);
    std::vector<double> motors_tau_des(model_nv - 6, 0);
    std::vector<double> motors_tau_cur(model_nv - 6, 0);
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
    WBC_solv.setQini(qIniDes,RobotState.q);

    // Precompute each joint's qpos address on both mj_model (the real,
    // simulated robot) and stateUI_model (a separate mjData used purely to
    // visualize the state estimate) by name, via kinDynSolver.ikJointNames --
    // rather than trusting RobotState.q(7..18), which comes from
    // DataBus::updateQ() assuming motors_pos_cur is already in Pin_KinDyn's
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
    // logger.addIterm("motors_pos_cur",model_nv-6);
    // logger.addIterm("motors_vel_cur",model_nv-6);
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

            // update kinematics/dynamics info (StateEst::set() reads
            // fe_l/r_pos_W and fe_l/r_pos_L/vel_L from here; GaitScheduler's
            // contact-force estimate below needs dyn_M/dyn_Non/J_l/J_r/dJ_l/dJ_r)
            kinDynSolver.dataBusRead(RobotState);
            kinDynSolver.computeJ_dJ();
            kinDynSolver.computeDyn();
            kinDynSolver.dataBusWrite(RobotState);

            // Standing on 2 legs, no walking yet: run GaitScheduler purely for
            // its contact-force estimate (FL_est/FR_est) and to keep
            // RobotState.legState/phi populated for StateEst -- motionState
            // stays Stand, so step() never advances phi or switches legState.
            if (simTime >= startSteppingTime)
            {
                RobotState.motionState = DataBus::Stand;
                gaitScheduler.dataBusRead(RobotState);
                gaitScheduler.step();
                gaitScheduler.dataBusWrite(RobotState);
            }

            if (simTime > 1 && StateModule.flag_init)
            {
                std::cout << "init state module" << std::endl;
                StateModule.init(RobotState);
            }

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
            } else
            jsInterp.setIniPos(RobotState.q(0), RobotState.q(1), RobotState.base_rpy(2));

            // if (simTime >= startSteppingTime) {
            //     jsInterp.step();
            //     jsInterp.setIniPos(RobotState.q(0), RobotState.q(1), stand_legLength + foot_height, RobotState.base_rpy(2));
            //     jsInterp.dataBusWrite(RobotState); // only pos x, pos y, pos_z, theta z, vel x, vel y , omega z are rewrote.
            //     // gait scheduler
            //     gaitScheduler.start();
            //     RobotState.motionState = DataBus::Walk;
            //     gaitScheduler.dataBusRead(RobotState);
            //     gaitScheduler.step();
            //     gaitScheduler.dataBusWrite(RobotState);

            //     footPlacement.dataBusRead(RobotState);
            //     footPlacement.getSwingPos();
            //     footPlacement.dataBusWrite(RobotState);
            // }

            // // ------------- WBC ------------
            // // WBC input
            // RobotState.des_ddq = Eigen::VectorXd::Zero(mj_model->nv);
            // RobotState.des_dq = Eigen::VectorXd::Zero(mj_model->nv);
            // RobotState.des_delta_q = Eigen::VectorXd::Zero(mj_model->nv);
            // RobotState.Fr_ff << 0,0,370,0,0,0,
            //                     0,0,370,0,0,0;

            // // adjust des_delata_q, des_dq and des_ddq to achieve forward walking
            // if (simTime > startWalkingTime + 1) {
            //     RobotState.des_delta_q.block<2, 1>(0, 0) << jsInterp.vx_W * mj_model->opt.timestep, jsInterp.vy_W * mj_model->opt.timestep;
            //     RobotState.des_delta_q(5) = jsInterp.wz_L * mj_model->opt.timestep;
            //     RobotState.des_dq.block<2, 1>(0, 0) << jsInterp.vx_W, jsInterp.vy_W;
            //     RobotState.des_dq(5) = jsInterp.wz_L;

            //     double k = 5;
            //     RobotState.des_ddq.block<2, 1>(0, 0) << k * (jsInterp.vx_W - RobotState.dq(0)), k * (jsInterp.vy_W -
            //                                                                                          RobotState.dq(1));
            //     RobotState.des_ddq(5) = k * (jsInterp.wz_L - RobotState.dq(5));
            // }


            // // WBC Calculation
            // WBC_solv.dataBusRead(RobotState);
            // WBC_solv.computeDdq(kinDynSolver);
            // WBC_solv.computeTau();
            // WBC_solv.dataBusWrite(RobotState);

            // // get the final joint command
            if (simTime<=startSteppingTime){
                // Ramp linearly from 0 to the IK-solved standing pose over
                // rampDuration seconds, rather than stepping straight to it
                // at t=0 -- a step here would be a large sudden position
                // error for PVT_Ctr's PD loop, i.e. a torque impulse.
                const double rampDuration = 2.0;
                double rampFrac = std::min(simTime / rampDuration, 1.0);
                Eigen::VectorXd rampedJointPos = rampFrac * resLeg.jointPosRes;
                RobotState.motors_pos_des = kinDynSolver.mapJointVecToOrder(rampedJointPos, pvtCtr.getMotorNames());
                RobotState.motors_vel_des=motors_vel_des;
                RobotState.motors_tor_des=motors_tau_des;
            }
            else
            {
                // Eigen::VectorXd pos_des=kinDynSolver.integrateDIY(RobotState.q, RobotState.wbc_delta_q_final);
                // RobotState.motors_pos_des = eigen2std(pos_des.block(7,0, model_nv-6,1));
                // RobotState.motors_vel_des = eigen2std(RobotState.wbc_dq_final);
                // RobotState.motors_tor_des = eigen2std(RobotState.wbc_tauJointRes);
            }

            pvtCtr.dataBusRead(RobotState);
            pvtCtr.calMotorsPVT();
            // pvtCtr.calMotorsPVT(100.0/1000.0/180.0*3.1415); // with speed limit
            // if (simTime<=3)
            // {
            //     pvtCtr.calMotorsPVT(100.0/1000.0/180.0*3.1415);
            // }
            // else
            // {
            //     double kp = 1.;
            //     double kd = 1.;

            //     pvtCtr.setJointPD(400 * kp, 15 * kd, "J_hip_l_roll");
            //     pvtCtr.setJointPD(200 * kp, 10 * kd, "J_hip_l_yaw");
            //     pvtCtr.setJointPD(300 * kp, 10 * kd, "J_hip_l_pitch");
            //     pvtCtr.setJointPD(300 * kp, 14 * kd, "J_knee_l_pitch");
            //     pvtCtr.setJointPD(300 * kp, 18 * kd, "J_ankle_l_pitch");
            //     pvtCtr.setJointPD(300 * kp, 16 * kd, "J_ankle_l_roll");

            //     pvtCtr.setJointPD(400 * kp, 15 * kd, "J_hip_r_roll");
            //     pvtCtr.setJointPD(200 * kp, 10 * kd, "J_hip_r_yaw");
            //     pvtCtr.setJointPD(300 * kp, 10 * kd, "J_hip_r_pitch");
            //     pvtCtr.setJointPD(300 * kp, 14 * kd, "J_knee_r_pitch");
            //     pvtCtr.setJointPD(300 * kp, 18 * kd, "J_ankle_r_pitch");
            //     pvtCtr.setJointPD(300 * kp, 16 * kd, "J_ankle_r_roll");

            //     pvtCtr.calMotorsPVT();
            // }
            
            pvtCtr.dataBusWrite(RobotState);
            mj_interface.setMotorsTorque(RobotState.motors_tor_out);

            // logger.startNewLine();
            // logger.recItermData("simTime", simTime);
            // logger.recItermData("motors_pos_cur",RobotState.motors_pos_cur);
            // logger.recItermData("motors_vel_cur",RobotState.motors_vel_cur);
            // logger.recItermData("rpy",RobotState.rpy);
            // logger.recItermData("fL",RobotState.fL);
            // logger.recItermData("fR",RobotState.fR);
            // logger.recItermData("basePos",RobotState.basePos);
            // logger.recItermData("baseLinVel",RobotState.baseLinVel);
            // logger.recItermData("baseAcc",RobotState.baseAcc);
            // logger.recItermData("baseAngVel",RobotState.baseAngVel);
            // logger.finishLine();

            // printf("rpyVal=[%.5f, %.5f, %.5f]\n", RobotState.rpy[0], RobotState.rpy[1], RobotState.rpy[2]);
            // printf("gps=[%.5f, %.5f, %.5f]\n", RobotState.basePos[0], RobotState.basePos[1], RobotState.basePos[2]);
            // printf("vel=[%.5f, %.5f, %.5f]\n", RobotState.baseLinVel[0], RobotState.baseLinVel[1], RobotState.baseLinVel[2]);
        }

        // if (mj_data->time>=simEndTime)
        // {
        //     break;
        // }

        uiController.updateScene();
        stateUI.updateScene();
    }

//    // free visualization storage
    uiController.Close();

    return 0;
}
/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024-2025 Humanoid Robot (Shanghai) Co., Ltd.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <cstdio>
#include <iostream>
#include "GLFW_callbacks.h"
#include "MJ_interface.h"


#include "data_logger.h"
#include "useful_math.h"
#include "MyStateEstimator.h"
#include "pino_kin_dyn.h"
#include "PVT_ctrl.h"
#include <algorithm>
#include "useful_math.h"
#include "gait_scheduler.h"

//************************
// main function
int main(int argc, const char** argv)
{
    std::cout << "Program Starts, Loading Mujoco xml model\n";
    const std::string MODEL_DIR = "models/mjcf"; // path to mujoco xml model

    //-------------------------------------------------------------------
    // Compile mujoco xml model
    //-------------------------------------------------------------------
    const std::string model_path = MODEL_DIR + "/scene_state_est.xml";
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
    std::cout << "Compile mujoco xml done\n";
    
    //************************************************************* */
    // ini classes
    //************************************************************* */
    const std::string joint_ctrl_config_path = "config/12dof_joint_config.json";
    UIctr uiController(mj_model,mj_data);   // UI control for Mujoco
    MJ_Interface mj_interface(mj_model, mj_data, joint_ctrl_config_path.c_str()); // data interface for Mujoco
    // print out xml model info
    std::printf("MuJoCo xml model info: \n");
    mj_interface.printInfo();
    GaitScheduler gaitScheduler(0.4, mj_model->opt.timestep); // gait scheduler

    const std::string urdf_path = "models/urdf/biped_robot_12dof.urdf";
    Pin_KinDyn kinDynSolver(urdf_path.c_str()); // kinematics and dynamics solver

    DataBus RobotState(kinDynSolver.model_nv); // data bus

    PVT_Ctr pvtCtr(mj_model->opt.timestep,joint_ctrl_config_path.c_str());// PVT joint control
    
    // FootPlacement footPlacement; // foot-placement planner
    // JoyStickInterpreter jsInterp(mj_model->opt.timestep); // desired baselink velocity generator
    DataLogger logger("record/wbc_walk_control.log"); // data logger
    StateEstimator state_estimator = StateEstimator(mj_model->opt.timestep, true);

    // scene_state_est.xml merges the real robot and a "_est"-suffixed ghost
    // twin (biped_robot_floatingbase_ghost_12dof.xml) into one model/mjData,
    // so the ghost can be driven every frame by directly overwriting its
    // qpos/qvel with the state estimate -- it has no actuators and no
    // collision geoms, so mj_step() never drives it on its own.
    // state_estimator.get_qj()/get_qjd() return the 12 leg joints in
    // mj_interface.JointName order, so map that same order to each ghost
    // joint's qpos/qvel address here, once, up front.
    std::vector<int> ghostJointQposAdr(mj_interface.JointName.size());
    std::vector<int> ghostJointQvelAdr(mj_interface.JointName.size());
    for (size_t i = 0; i < mj_interface.JointName.size(); i++)
    {
        std::string ghostJointName = mj_interface.JointName[i] + "_est";
        int jid = mj_name2id(mj_model, mjOBJ_JOINT, ghostJointName.c_str());
        ghostJointQposAdr[i] = mj_model->jnt_qposadr[jid];
        ghostJointQvelAdr[i] = mj_model->jnt_dofadr[jid];
    }
    int ghostFreeJointId = mj_name2id(mj_model, mjOBJ_JOINT, "floating_base_joint_est");
    int ghostFreeQposAdr = mj_model->jnt_qposadr[ghostFreeJointId];
    int ghostFreeQvelAdr = mj_model->jnt_dofadr[ghostFreeJointId];

    // seed the EKF's base-position estimate with the real robot's actual
    // starting pose (mj_data->qpos here still holds the XML's default free-
    // joint position, since no mj_step has run yet) -- otherwise xhat_ starts
    // at the origin and the ghost visibly starts buried in the floor before
    // slowly rising to match, since there's no absolute-position sensor to
    // correct that quickly
    int realFreeJointId = mj_name2id(mj_model, mjOBJ_JOINT, "floating_base_joint");
    int realFreeQposAdr = mj_model->jnt_qposadr[realFreeJointId];
    state_estimator.setBasePosEst(Eigen::Map<Eigen::Vector3d>(mj_data->qpos + realFreeQposAdr));


    //************************************************************* */
    // variables ini
    //************************************************************* */
    
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

    // for ramping the standing
    const double rampDuration = 2.0;
    double rampFrac = std::min(simTime / rampDuration, 1.0);
    Eigen::VectorXd rampedJointPos = rampFrac * resLeg.jointPosRes;

    // init UI: GLFW
    uiController.iniGLFW();
    uiController.disableTracking(); // enable viewpoint tracking of the body 1 of the robot
    uiController.createWindow("Demo",false);

    // real-time plot of the foot touch sensors (lf-touch, rf-touch)
    const char* touchLineNames[2] = {"lf-touch", "rf-touch"};
    const float touchLineColors[2][3] = {{1, 0, 0}, {0, 0, 1}};
    uiController.initSensorFigure("Foot Touch Sensors", touchLineNames, touchLineColors, 2);

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

            // RobotState.updateQ() (just called above) copies
            // motors_pos_cur/motors_vel_cur straight into q(7:)/dq(6:),
            // assuming they're already in Pinocchio's joint order -- but
            // MJ_Interface actually returns them alphabetically (from its
            // JSON config's keys), a different order entirely. Left as-is,
            // computeJ_dJ() below would run FK on a shuffled leg
            // configuration, handing the EKF a badly wrong foot-position
            // measurement that biases the base-position estimate by several
            // centimeters. Overwrite with a correctly reordered copy first.
            std::vector<double> q_pin = kinDynSolver.mapJointVecFromOrder(RobotState.motors_pos_cur, mj_interface.JointName);
            std::vector<double> dq_pin = kinDynSolver.mapJointVecFromOrder(RobotState.motors_vel_cur, mj_interface.JointName);
            for (size_t i = 0; i < q_pin.size(); i++) {
                RobotState.q(i + 7) = q_pin[i];
                RobotState.dq(i + 6) = dq_pin[i];
            }

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

            rampFrac = std::min(simTime / rampDuration, 1.0);
            rampedJointPos = rampFrac * resLeg.jointPosRes;
            RobotState.motors_pos_des = kinDynSolver.mapJointVecToOrder(rampedJointPos, pvtCtr.getMotorNames());
            RobotState.motors_vel_des= motor_vel_des;
            RobotState.motors_tor_des= motor_tau_des;

            pvtCtr.dataBusRead(RobotState); // to update joint command
            pvtCtr.calMotorsPVT(); // calculate joint torque

            pvtCtr.dataBusWrite(RobotState); // set to RobotState
            mj_interface.setMotorsTorque(RobotState.motors_tor_out); // Set joint torque to mujoco
        }
        // propagate the state estimate into the ghost overlay's qpos/qvel,
        // then re-run forward kinematics (not mj_step -- the ghost is never
        // simulated, only ever visually puppeted by directly overwriting its
        // pose) so it renders the estimate instead of free-falling under
        // gravity for the ~1/60s until the next overwrite
        Eigen::Matrix<double, 3, 1> basePos_est = state_estimator.getBasePosEst();
        Eigen::Matrix<double, 3, 1> baseVel_est = state_estimator.getBaseVelEst();
        Eigen::Matrix<double, 4, 1> imu_quat = state_estimator.getImuquaternion(); // (x,y,z,w)
        Eigen::Matrix<double, 12, 1> qj_est = state_estimator.get_qj();
        Eigen::Matrix<double, 12, 1> qjd_est = state_estimator.get_qjd();

        mj_data->qpos[ghostFreeQposAdr + 0] = basePos_est(0);
        mj_data->qpos[ghostFreeQposAdr + 1] = basePos_est(1);
        mj_data->qpos[ghostFreeQposAdr + 2] = basePos_est(2);
        // MuJoCo's free-joint quaternion order is (w,x,y,z); the
        // estimator/Pinocchio convention is (x,y,z,w)
        mj_data->qpos[ghostFreeQposAdr + 3] = imu_quat(3); // w
        mj_data->qpos[ghostFreeQposAdr + 4] = imu_quat(0); // x
        mj_data->qpos[ghostFreeQposAdr + 5] = imu_quat(1); // y
        mj_data->qpos[ghostFreeQposAdr + 6] = imu_quat(2); // z

        mj_data->qvel[ghostFreeQvelAdr + 0] = baseVel_est(0);
        mj_data->qvel[ghostFreeQvelAdr + 1] = baseVel_est(1);
        mj_data->qvel[ghostFreeQvelAdr + 2] = baseVel_est(2);
        // angular velocity isn't part of the KF's state (orientation comes
        // straight from the IMU, not the filter), so there's no estimate to
        // write here; zero is harmless since qvel only feeds rendering-
        // irrelevant quantities for a sensor-free, collision-free ghost
        mj_data->qvel[ghostFreeQvelAdr + 3] = 0;
        mj_data->qvel[ghostFreeQvelAdr + 4] = 0;
        mj_data->qvel[ghostFreeQvelAdr + 5] = 0;

        for (size_t i = 0; i < mj_interface.JointName.size(); i++)
        {
            mj_data->qpos[ghostJointQposAdr[i]] = qj_est(i);
            mj_data->qvel[ghostJointQvelAdr[i]] = qjd_est(i);
        }

        mj_forward(mj_model, mj_data);

        // touch sensor
        Eigen::Matrix<double, 2, 1> touchVals = state_estimator.getTouchSensorValue();
        uiController.updateSensorFigure(mj_data->time, touchVals.data(), 2);
        uiController.updateScene();
    }

    // free visualization storage
    uiController.Close();

    return 0;
}
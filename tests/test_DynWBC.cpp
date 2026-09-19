#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <cstdio>
#include <iostream>
#include "GLFW_callbacks.h"
#include "MJ_interface.h"

#include <algorithm>
#include "data_type.h"
#include "robot_wrapper.h"
#include "useful_math.h"
#include "MyStateEstimator.h"
#include "PVT_ctrl.h"
#include "KinWBC.h"
#include "DynWBC.h"
#include "joystick_interpreter.h"
#include "foot_placement.h"
#include "my_gait_scheduler.h"
#include "RealtimePlot.h"

using namespace std;

const std::string URDF_PATH = "models/urdf/biped_robot_12dof.urdf";
const std::string XML_PATH = "models/mjcf/scene_floatingbase_12dof.xml";
const std::string YAML_PLANNING_CF_PATH = "config/step_planning_cf.yaml";
const std::string YAML_JOINT_CF_PATH = "config/12dof_joint_config.yaml";
const std::string YAML_QP_WBC_CF_PATH = "config/qp_config.yaml";

char loadError[1024] = ""; // character array, size 1024

int main()
{
    //-------------------------------------------------------------------
    // Compile mujoco xml model
    //-------------------------------------------------------------------
    // load/compile xml model
    // model_path.c_str() return a read-only pointer to const std::string model_path
    mjModel *mj_model = mj_loadXML(XML_PATH.c_str(), nullptr, loadError, sizeof(loadError)); // pointer to mjModel struct
    if (!mj_model)
    {
        std::fprintf(stderr, "failed to load %s: %s\n", XML_PATH.c_str(), loadError);
        return 1;
    }
    mjData *mj_data = mj_makeData(mj_model); // pointer to mjData struct
    std::cout << "Compile mujoco xml done\n";

    //************************************************************* */
    // init classes: exercises the sensor -> state-estimate -> joint-PD
    // control pipeline (MJ_Interface, RobotSensor, RobotWrapper,
    // StateEstimator, PVT_Ctr)
    //************************************************************* */
    RobotWrapper robot_wrapper = RobotWrapper(URDF_PATH, false);
    RobotWrapper robot_wrapper_ref = RobotWrapper(URDF_PATH);
    KinWBC kin_wbc;
    DynWBC dyn_wbc(YAML_JOINT_CF_PATH, YAML_QP_WBC_CF_PATH, robot_wrapper, true);
    RobotSensor rb_sensors(mj_model->na);
    JoyStickInterpreter joyStick(kin_wbc.dt);
    MyGaitScheduler gaitScheduler(YAML_PLANNING_CF_PATH, kin_wbc.dt);
    FootPlacement footPlanner(YAML_PLANNING_CF_PATH);

    const double dt = kin_wbc.dt;
    const double zc = 0.5;
    CP_Planning cp_planner(dt, zc, footPlanner.hip_width);


    UIctr uiController(mj_model, mj_data);   // UI control for Mujoco
    MJ_Interface mj_interface(mj_model, mj_data, YAML_JOINT_CF_PATH.c_str()); // data interface for Mujoco
    // print out xml model info
    // std::printf("MuJoCo xml model info: \n");
    // mj_interface.printInfo();

    PVT_Ctr pvtCtr(mj_model->opt.timestep, YAML_JOINT_CF_PATH.c_str()); // PVT joint control
    StateEstimator state_estimator(mj_model->opt.timestep, false);
    KinWBC kinWBC_solver;

    //

    const double init_base_height = 0.755;
    VectorXd qIniDes = robot_wrapper.computeInitial_Stand(init_base_height);
    printf("Init standing joint config: \n");
    std::cout << qIniDes.transpose() << std::endl;

    // Spawn robot in home configuration (all actuated joints are 0)
    mj_data->qpos[0] = 0.0;
    mj_data->qpos[1] = 0.0;
    mj_data->qpos[2] = 0.80; // straight-leg base height with feet resting on ground
    mj_data->qpos[3] = 1.0;    // quat w
    mj_data->qpos[4] = 0.0;    // quat x
    mj_data->qpos[5] = 0.0;    // quat y
    mj_data->qpos[6] = 0.0;    // quat z
    for (int i = 0; i < robot_wrapper.model_na_; i++) {
        mj_data->qpos[7 + i] = 0.0;
    }
    mju_zero(mj_data->qvel, mj_model->nv);
    mju_zero(mj_data->qacc, mj_model->nv);
    mj_forward(mj_model, mj_data);

    // Initial forward kinematics and state estimation
    mj_interface.updateSensorValues(rb_sensors);
    state_estimator.update(rb_sensors, robot_wrapper);
    robot_wrapper.computeKin();
    footPlanner.legLength = robot_wrapper.pos_base_W(2);
    std::cout << "Initial base height: " << robot_wrapper.pos_base_W(2) << "\n";

    /// ----------------- sim Loop ---------------
    mjtNum simstart = mj_data->time;
    double simTime = mj_data->time;

    // init UI: GLFW
    uiController.iniGLFW();
    uiController.disableTracking(); // enable viewpoint tracking of the body 1 of the robot
    uiController.createWindow("Demo", false);

    // // Signal plotting (must be created after GLFW window initialization)
    // RealtimePlot JoyStickPlot(mj_model, 800, 600, "Base Height & Joystick", 5.0);
    // JoyStickPlot.setYLabel("meters");
    // JoyStickPlot.setYLimit(0.60, 0.90); 
    // JoyStickPlot.setLineWidth(2.5f);

    // RealtimePlot JointRefPlot(mj_model, 800, 600, "Joint Reference Positions", 5.0);
    // JointRefPlot.setYLabel("rad");
    // JointRefPlot.setYLimit(-1.0, 1.0, true);
    // JointRefPlot.setLineWidth(2.0f);

    // RealtimePlot ContactForcePlot(mj_model, 800, 600, "Optimal Contact Force (WBC QP)", 5.0);
    // ContactForcePlot.setYLabel("N");
    // ContactForcePlot.setYLimit(-20, 20.0, true);
    // ContactForcePlot.setLineWidth(2.5f);

    // Duration to ramp joints to qIniDes
    const double rampDuration = 2.0;
    bool joystick_initialized = false;
    bool joystick_control_start = false;
    bool goUp = false;
    bool goDown = false;
    VectorXd tau_wbc;
    while (!glfwWindowShouldClose(uiController.window))
    {
        simstart = mj_data->time;
        while (mj_data->time - simstart < 1.0 / 60.0 && uiController.runSim) // press "1" to pause and resume, "2" to step the simulation
        {
            mj_step(mj_model, mj_data);
            uiController.applyPerturbation();
            simTime = mj_data->time;

            mj_interface.updateSensorValues(rb_sensors); // propagate sensor values to rb_sensors
            state_estimator.update(rb_sensors, robot_wrapper);
            robot_wrapper.computeKin();

            // After 2 seconds, initialize the joystick to hold the CURRENT
            // measured base height (neutral -- no motion commanded yet).
            if (simTime >= 2.0 && !joystick_initialized)
            {
                joyStick.setIniPos(robot_wrapper.pos_base_W(0), robot_wrapper.pos_base_W(1), robot_wrapper.pos_base_W(2), 0.0);
                joyStick.setMotionState(MotionState::STAND);
                joyStick.setVxDesLPara(0.0, 0.1);
                joyStick.setVyDesLPara(0.0, 0.1);
                joyStick.setWzDesLPara(0.0, 0.1);

                joyStick.setPzRef(0.75, 3.0);
                goDown = true;
                goUp = false;

                robot_wrapper_ref.q = robot_wrapper.q;
                robot_wrapper_ref.dq = robot_wrapper.dq;


                footPlanner.legLength = robot_wrapper.pos_base_W(2);
                joystick_initialized = true;
                std::cout << "[t=" << simTime << "] Joystick initialized to measured base height: " << robot_wrapper.pos_base_W(2) << "\n";
                dyn_wbc.updateRobotState(robot_wrapper, state_estimator);
                dyn_wbc.setupQPproblem(robot_wrapper);
            }

            if (joystick_initialized)
            {
                joyStick.step();
                gaitScheduler.step(joyStick);
                // Oscillate the base-height target between 0.75m and 0.80m --
                if (joyStick.PzLGen.isReachDes())
                {
                    if (goDown) {
                        goDown = false;
                        goUp = true;
                        joyStick.setPzRef(0.80, 3.0);
                        // std::cout << "[t=" << simTime << "] base height target -> 0.80m\n";
                    } else {
                        goDown = true;
                        goUp = false;
                        joyStick.setPzRef(0.75, 3.0);
                        // std::cout << "[t=" << simTime << "] base height target -> 0.75m\n";
                    }
                }

                robot_wrapper.computeKin();
                robot_wrapper_ref.computeKin();
                // In standing mode, keep desired CoM centered between the feet
                cp_planner.xc_ = 0.0;
                cp_planner.yc_ = 0.0;
                cp_planner.d_xc_ = 0.0;
                cp_planner.d_yc_ = 0.0;

                kin_wbc.computeWBC_IK(joyStick, footPlanner, robot_wrapper_ref, cp_planner, gaitScheduler);

                const double stepSize = 1.0;
                robot_wrapper_ref.integrateConfig(stepSize * kin_wbc.out_delta_q);
                robot_wrapper_ref.dq = kin_wbc.out_dq;
                robot_wrapper_ref.computeKin();

                // Drive the robot with plain joint-space PD tracking of the
                // KinWBC-generated reference (position + velocity).
                VectorXd ref_pos = kin_wbc.getMotorPosDes(); // kin_wbc.q_des joint segment
                VectorXd ref_vel = kin_wbc.out_dq.tail(robot_wrapper.model_na_);
                VectorXd tau_ff = VectorXd::Zero(robot_wrapper.model_na_);
                pvtCtr.getFeedbackMotorState(robot_wrapper);
                pvtCtr.calMotorsPVT(ref_pos, ref_vel, tau_ff);
                mj_interface.setMotorsTorque(pvtCtr.motor_tor_out_motor);
            }
            else
            {
                // Joint-space ramp from 0 to qIniDes over rampDuration (2 seconds)
                double rampFrac = std::min(simTime / rampDuration, 1.0);
                VectorXd rampedJointPos = rampFrac * qIniDes;

                pvtCtr.motor_pos_des = rampedJointPos;
                pvtCtr.motor_vel_des = VectorXd::Zero(robot_wrapper.model_na_);
                pvtCtr.motor_tor_des = VectorXd::Zero(robot_wrapper.model_na_);
                pvtCtr.getFeedbackMotorState(robot_wrapper);
                pvtCtr.calMotorsPVT(); // PD impedance stand control with LPF
                mj_interface.setMotorsTorque(pvtCtr.motor_tor_out_motor); // set joint torque to mujoco
            }
        }

        uiController.updateScene();
    }

    // free visualization storage
    uiController.Close();
    return 0;
}

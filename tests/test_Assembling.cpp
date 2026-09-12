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
const std::string YAML_WBC_DYN_CF_PATH = "config/12dof_joint_config.yaml";
const std::string QP_WBC_CF_PATH = "config/qp_config.yaml";

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
    RobotWrapper robot_wrapper = RobotWrapper(URDF_PATH);
    KinWBC kin_wbc;
    RobotSensor rb_sensors(mj_model->na);
    JoyStickInterpreter joyStick(kin_wbc.dt);
    MyGaitScheduler gaitScheduler(YAML_PLANNING_CF_PATH, kin_wbc.dt);
    FootPlacement footPlanner(YAML_PLANNING_CF_PATH);
    const double dt = kin_wbc.dt;
    const double zc = 0.5;
    CP_Planning cp_planner(dt, zc, footPlanner.hip_width);

    const std::string joint_ctrl_config_path = "config/12dof_joint_config.yaml";
    UIctr uiController(mj_model, mj_data);   // UI control for Mujoco
    MJ_Interface mj_interface(mj_model, mj_data, joint_ctrl_config_path.c_str()); // data interface for Mujoco
    // print out xml model info
    std::printf("MuJoCo xml model info: \n");
    mj_interface.printInfo();

    PVT_Ctr pvtCtr(mj_model->opt.timestep, joint_ctrl_config_path.c_str()); // PVT joint control
    StateEstimator state_estimator(mj_model->opt.timestep, true);
    KinWBC kinWBC_solver;

    const double init_base_height = 0.755;
    VectorXd qIniDes = robot_wrapper.computeInitial_Stand(init_base_height);
    printf("Init standing joint config: \n");
    std::cout << qIniDes << std::endl;

    // Option 1: Spawn robot directly in bent-knee standing posture to eliminate singularity
    mj_data->qpos[0] = 0.0;
    mj_data->qpos[1] = 0.0;
    mj_data->qpos[2] = 0.78; // nominal standing base height (0.75m leg + foot clearance)
    mj_data->qpos[3] = 1.0;    // quat w
    mj_data->qpos[4] = 0.0;    // quat x
    mj_data->qpos[5] = 0.0;    // quat y
    mj_data->qpos[6] = 0.0;    // quat z
    for (int i = 0; i < robot_wrapper.model_na_; i++) {
        mj_data->qpos[7 + i] = 0*qIniDes(i);
    }
    mju_zero(mj_data->qvel, mj_model->nv);
    mju_zero(mj_data->qacc, mj_model->nv);
    mj_forward(mj_model, mj_data);

    // Initial forward kinematics and state estimation
    mj_interface.updateSensorValues(rb_sensors);
    state_estimator.update(rb_sensors, robot_wrapper);
    robot_wrapper.computeKin();

    /// ----------------- sim Loop ---------------
    mjtNum simstart = mj_data->time;
    double simTime = mj_data->time;

    // init UI: GLFW
    uiController.iniGLFW();
    uiController.disableTracking(); // enable viewpoint tracking of the body 1 of the robot
    uiController.createWindow("Demo", false);

    // Signal plotting (must be created after GLFW window initialization)
    RealtimePlot JoyStickPlot(mj_model, 800, 600, "Joystick Command", 5.0);
    JoyStickPlot.setYLabel("meters");
    JoyStickPlot.setYLimit(-0.35, 1.0); 
    JoyStickPlot.setLineWidth(2.5f);

    RealtimePlot JointRefPlot(mj_model, 800, 600, "KinWBC Joint Reference Positions", 5.0);
    JointRefPlot.setYLabel("rad");
    JointRefPlot.setYLimit(-1.0, 1.0, true);
    JointRefPlot.setLineWidth(2.0f);

    // Joystick handles the reference trajectory from t = 0
    const double rampDuration = 4.0;
    joyStick.setIniPos(robot_wrapper.pos_base_W(0), robot_wrapper.pos_base_W(1), robot_wrapper.pos_base_W(2), 0.0);
    joyStick.setMotionState(MotionState::STAND);
    

    cp_planner.xc_ = robot_wrapper.pos_CoM_W(0);
    cp_planner.yc_ = robot_wrapper.pos_CoM_W(1);
    cp_planner.d_xc_ = 0.0;
    cp_planner.d_yc_ = 0.0;

    while (!glfwWindowShouldClose(uiController.window))
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        simstart = mj_data->time;
        while (mj_data->time - simstart < 1.0 / 60.0 && uiController.runSim) // press "1" to pause and resume, "2" to step the simulation
        {
            mj_step(mj_model, mj_data);
            uiController.applyPerturbation();
            simTime = mj_data->time;

            mj_interface.updateSensorValues(rb_sensors); // propagate sensor values to rb_sensors
            state_estimator.update(rb_sensors, robot_wrapper);
            robot_wrapper.computeKin();
            joyStick.step(); // run the joystick cmd

            gaitScheduler.step(joyStick); // synchronize motion state and gait phase (STAND)

            kin_wbc.computeWBC_IK(joyStick, footPlanner, robot_wrapper, cp_planner, gaitScheduler); // compute reference motion
            
            pvtCtr.getFeedbackMotorState(robot_wrapper); // read back the estimated joint pos/vel            
            pvtCtr.calMotorsPVT(kin_wbc); // KinWBC controls the robot from t = 0

            mj_interface.setMotorsTorque(pvtCtr.motor_tor_out_motor); // set joint torque to mujoco
        }
        
        // JoyStickPlot.addPoint("base height", simTime, robot_wrapper.pos_base_W(2));
        JoyStickPlot.addPoint("target pz_W", simTime, joyStick.pz_W);
        JoyStickPlot.addPoint("vx_ref", simTime, joyStick.vx_W);
        JoyStickPlot.addPoint("vy_ref", simTime, joyStick.vy_W);
        JoyStickPlot.render(); // makes JoyStickPlot's own context current, draws, swaps buffers

        static const std::vector<std::string> jointNamesShort = {
            "L_hip_pitch", "L_hip_roll", "L_hip_yaw", "L_knee", "L_ankle_roll", "L_ankle_pitch",
            "R_hip_pitch", "R_hip_roll", "R_hip_yaw", "R_knee", "R_ankle_roll", "R_ankle_pitch"
        };
        for (size_t j = 0; j < 12; ++j)
        {
            JointRefPlot.addPoint(jointNamesShort[j], simTime, pvtCtr.motor_pos_des(j));
        }
        JointRefPlot.render();

        uiController.updateScene();
    }

    // free visualization storage
    uiController.Close();
    return 0;
}

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
const std::string QP_WBC_CF_PATH = "config/wbc_config.yaml";

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
    RobotWrapper robot_wrapper_ref = RobotWrapper(URDF_PATH);
    KinWBC kin_wbc;
    DynWBC dyn_wbc(YAML_WBC_DYN_CF_PATH, QP_WBC_CF_PATH, robot_wrapper, false);
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
    std::cout << qIniDes.transpose() << std::endl;

    // Initialize reference kinematic model to qIniDes standing pose with feet on ground (z=0)
    robot_wrapper_ref.q.segment(7, robot_wrapper_ref.model_na_) = qIniDes;
    robot_wrapper_ref.q(0) = 0.0;
    robot_wrapper_ref.q(1) = 0.0;
    robot_wrapper_ref.q(2) = 0.0;
    robot_wrapper_ref.q(3) = 0.0;
    robot_wrapper_ref.q(4) = 0.0;
    robot_wrapper_ref.q(5) = 0.0;
    robot_wrapper_ref.q(6) = 1.0;
    robot_wrapper_ref.computeKin();
    robot_wrapper_ref.q(2) = -robot_wrapper_ref.pos_L_feet_W(2); // place feet flat on ground
    robot_wrapper_ref.computeKin();



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

    // Signal plotting (must be created after GLFW window initialization)
    RealtimePlot JoyStickPlot(mj_model, 800, 600, "Base Height & Joystick", 5.0);
    JoyStickPlot.setYLabel("meters");
    JoyStickPlot.setYLimit(0.60, 0.90); 
    JoyStickPlot.setLineWidth(2.5f);

    RealtimePlot JointRefPlot(mj_model, 800, 600, "Joint Reference Positions", 5.0);
    JointRefPlot.setYLabel("rad");
    JointRefPlot.setYLimit(-1.0, 1.0, true);
    JointRefPlot.setLineWidth(2.0f);

    RealtimePlot TorquePlot(mj_model, 800, 600, "DynWBC Commanded Torque", 5.0);
    TorquePlot.setYLabel("N*m");
    TorquePlot.setYLimit(-20.0, 20.0, false);
    TorquePlot.setLineWidth(2.0f);

    // L_ankle_pitch-only comparison: what plain PD (currently driving the
    // robot) commands vs. what DynWBC's QP would command for the same
    // reference -- DynWBC is computed here purely for this comparison, its
    // output is not applied to the robot. mjvFigure has no per-line
    // dash/dot style (see RealtimePlot.h), so PD vs WBC is distinguished by
    // legend name/color instead of line style.
    RealtimePlot LeftLegTorqueCompare(mj_model, 800, 600, "L_ankle_pitch Torque: PD vs WBC", 5.0);
    LeftLegTorqueCompare.setYLabel("N*m");
    LeftLegTorqueCompare.setYLimit(-20.0, 20.0, false);
    LeftLegTorqueCompare.setLineWidth(2.0f);

    // Duration to ramp joints to qIniDes
    const double rampDuration = 2.0;
    bool joystick_initialized = false;
    bool joystick_control_start = false;
    VectorXd tau_pd_latest = VectorXd::Zero(robot_wrapper.model_na_);
    VectorXd tau_dynwbc_latest = VectorXd::Zero(robot_wrapper.model_na_);

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

            // After 2 seconds, initialize the joystick base height command once
            if (simTime >= 2.0 && !joystick_initialized)
            {
                joyStick.setIniPos(robot_wrapper.pos_base_W(0), robot_wrapper.pos_base_W(1), robot_wrapper.pos_base_W(2), 0.0);
                joyStick.setMotionState(MotionState::STAND);
                joyStick.setVxDesLPara(0.0, 0.1);
                joyStick.setVyDesLPara(0.0, 0.1);
                joyStick.setWzDesLPara(0.0, 0.1);
                joyStick.setPzRef(robot_wrapper.pos_base_W(2), 0.01);
                footPlanner.legLength = robot_wrapper.pos_base_W(2);
                cp_planner.xc_ = 0.5 * (robot_wrapper.pos_L_feet_W(0) + robot_wrapper.pos_R_feet_W(0));
                cp_planner.yc_ = 0.5 * (robot_wrapper.pos_L_feet_W(1) + robot_wrapper.pos_R_feet_W(1));
                joystick_initialized = true;
                std::cout << "[t=" << simTime << "] Joystick initialized to measured base height: " << robot_wrapper.pos_base_W(2) << "\n";
            }

            if (simTime >= 5.0 && !joystick_control_start) {
                joystick_control_start = true;
                joyStick.setPzRef(0.76, 3.0);
                joyStick.setPitchRef(0.0, 3.0);
                pvtCtr.getFeedbackMotorState(robot_wrapper);
                pvtCtr.motor_pos_des_old = pvtCtr.motor_pos_cur;

                // Sync the internal reference model to the robot's actual
                // measured state right as feedback tracking control engages,
                // then let it evolve open-loop (self-integrated) from here --
                // see the note at the computeWBC_IK() call below.
                robot_wrapper_ref.q = robot_wrapper.q;
                robot_wrapper_ref.dq = robot_wrapper.dq;
                robot_wrapper_ref.computeKin();

                std::cout << "[t=" << simTime << "] DynWBC QP control started, commanding base height from "
                          << robot_wrapper.pos_base_W(2) << "m to 0.72m in 3s\n";
            }

            if (simTime >= 5.0)
            {
                joyStick.step();
                gaitScheduler.step(joyStick);

                robot_wrapper.computeKin();

                // In standing mode, keep desired CoM centered between the feet
                cp_planner.xc_ = 0.0;
                cp_planner.yc_ = 0.0;
                cp_planner.d_xc_ = 0.0;
                cp_planner.d_yc_ = 0.0;

                // KinWBC solves against robot_wrapper_ref -- its OWN
                // internally-integrated state -- not the feedback state, so
                // the generated reference trajectory (out_delta_q, out_dq,
                // q_des) is a smooth, open-loop evolution driven purely by
                // the task targets (joystick height ramp, etc), independent
                // of whatever the real robot is currently doing. This is
                // what decouples reference generation from feedback: the
                // previous design fed the ACTUAL (possibly diverging) robot
                // state back into the recursive IK every tick, so a real
                // tracking error fed straight back into "the correction
                // needed right now," compounding tick over tick (deadbeat
                // windup, see doc/DynWBC_review_and_plan.md).
                kin_wbc.computeWBC_IK(joyStick, footPlanner, robot_wrapper_ref, cp_planner, gaitScheduler);

                // Advance the reference model by its own solved correction
                // (self-integration), using RobotWrapper's own canonical,
                // Pinocchio-consistent integrator (pin::integrate under the
                // hood) rather than trusting KinWBC's separate q_des (which
                // composes the quaternion itself via intQuat) -- this keeps
                // robot_wrapper_ref's state advanced the same way the rest
                // of RobotWrapper expects a tangent-space step to be applied.
                const double stepSize = 1.0;
                robot_wrapper_ref.integrateConfig(stepSize * kin_wbc.out_delta_q);
                robot_wrapper_ref.dq = kin_wbc.out_dq;
                robot_wrapper_ref.computeKin();

                // Drive the robot with plain joint-space PD tracking of the
                // KinWBC-generated reference (position + velocity) -- this
                // is the confirmed-stable path (see doc/DynWBC_review_and_plan.md).
                VectorXd ref_pos = kin_wbc.getMotorPosDes(); // kin_wbc.q_des joint segment
                VectorXd ref_vel = kin_wbc.out_dq.tail(robot_wrapper.model_na_);
                VectorXd tau_ff = VectorXd::Zero(robot_wrapper.model_na_);
                pvtCtr.getFeedbackMotorState(robot_wrapper);
                pvtCtr.calMotorsPVT(ref_pos, ref_vel, tau_ff);
                mj_interface.setMotorsTorque(pvtCtr.motor_tor_out_motor);
                tau_pd_latest = Eigen::Map<const VectorXd>(pvtCtr.motor_tor_out_link.data(), pvtCtr.motor_tor_out_link.size());

                // DynWBC's QP torque, computed for the SAME reference/
                // feedback state purely for comparison -- not applied to the
                // robot (PD above is what's actually driving it).
                dyn_wbc.updateRobotState(robot_wrapper, state_estimator);
                tau_dynwbc_latest = dyn_wbc.computeTorque(kin_wbc, robot_wrapper);
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
        
        JoyStickPlot.addPoint("base height (est)", simTime, state_estimator.getBasePosEst()(2));
        // JoyStickPlot.addPoint("base height (cmd)", simTime, joyStick.pz_W);
        JoyStickPlot.addPoint("target pz_W", simTime, joyStick.pz_W);
        JoyStickPlot.addPoint("vx_ref", simTime, joyStick.vx_W);
        JoyStickPlot.addPoint("vy_ref", simTime, joyStick.vy_W);
        // JoyStickPlot.addPoint("", double time, double value)
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

        for (size_t j = 0; j < 12; ++j)
        {
            TorquePlot.addPoint(jointNamesShort[j], simTime, tau_pd_latest(j));
        }
        TorquePlot.render();

        {
            const size_t j = 5; // L_ankle_pitch only
            LeftLegTorqueCompare.addPoint(jointNamesShort[j] + " (PD)", simTime, tau_pd_latest(j));
            LeftLegTorqueCompare.addPoint(jointNamesShort[j] + " (WBC)", simTime, tau_dynwbc_latest(j));
        }
        LeftLegTorqueCompare.render();

        uiController.updateScene();
    }

    // free visualization storage
    uiController.Close();
    return 0;
}

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
const std::string YAML_QP_WBC_CF_PATH = "config/wbc_config.yaml";

char loadError[1024] = ""; // character array, size 1024

// Draws a wireframe friction-cone pyramid at a contact point: apex at
// contactPos (Fr=0), base a square at height Fz_max with corners at
// Fr=(±muy*Fz_max, ±muy*Fz_max, Fz_max) -- exactly the 4 inequality planes
// DynWBC::U_single_ builds (eq. 12's box-pyramid approximation). Drawn with
// plain line segments (UIctr::addLine(), no arrowhead) rather than
// addArrow() -- 8 little arrowheads per foot made the wireframe look
// cluttered/spiky instead of reading as a cone. Reusing the same `scale`
// (N -> meters) as the force-vector arrows makes cone size and solved-force
// length directly, visually comparable: the force arrow's tip should stay
// inside this pyramid iff the QP's friction constraint is satisfied.
static void drawFrictionCone(UIctr &uiController, const Eigen::Vector3d &contactPos,
                              double muy, double FzMax, double scale, const float rgba[4])
{
    const double fx = muy * FzMax;
    const double fy = muy * FzMax;
    Eigen::Vector3d corners[4] = {
        Eigen::Vector3d( fx,  fy, FzMax),
        Eigen::Vector3d(-fx,  fy, FzMax),
        Eigen::Vector3d(-fx, -fy, FzMax),
        Eigen::Vector3d( fx, -fy, FzMax),
    };
    Eigen::Vector3d cornerPos[4];
    for (int i = 0; i < 4; ++i)
        cornerPos[i] = contactPos + scale * corners[i];
    for (int i = 0; i < 4; ++i) {
        uiController.addLine(contactPos, cornerPos[i], rgba);              // apex -> corner edge
        uiController.addLine(cornerPos[i], cornerPos[(i + 1) % 4], rgba);  // base square edge
    }
}

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
    KinWBC kin_wbc(YAML_QP_WBC_CF_PATH);
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

    RealtimePlot ContactForcePlot(mj_model, 800, 600, "Optimal Contact Force (WBC QP)", 5.0);
    ContactForcePlot.setYLabel("N");
    ContactForcePlot.setYLimit(-2, 2, true); // baseline only -- auto-extends if Fr goes further
    ContactForcePlot.setLineWidth(2.5f);

    RealtimePlot ContactMomentPlot(mj_model, 800, 600, "Optimal Contact Moment (WBC QP)", 5.0);
    ContactMomentPlot.setYLabel("N*m");
    ContactMomentPlot.setYLimit(-2, 2, true); // baseline only -- auto-extends if Fr's moment components go further
    ContactMomentPlot.setLineWidth(2.5f);

    // Vector/cone visualization colors: left foot force red, right foot
    // force blue, both cones a translucent green (alpha<1 so overlapping
    // cone edges at the two feet stay legible).
    const float colorLeftForce[4]  = {1.0f, 0.1f, 0.1f, 1.0f};
    const float colorRightForce[4] = {0.1f, 0.3f, 1.0f, 1.0f};
    const float colorCone[4]       = {0.2f, 0.8f, 0.2f, 0.35f};
    // Cone scale: N -> meters for drawFrictionCone()'s pyramid only. Cone
    // height = Fz_max * coneScale (eq. 13's bound, 400N) -- 0.002 made it
    // 0.8m tall, about as tall as the robot itself; 0.0004 keeps it a
    // comparable size to a foot instead (0.16m).
    const double coneScale = 0.0004;
    // Force-arrow scale: N -> meters for the actual solved Fr, kept
    // separate (and larger) from coneScale so the arrow stays clearly
    // visible even though actual standing forces are a small fraction of
    // Fz_max -- e.g. Fr_z ~150N draws at ~0.3m here vs. ~0.06m at
    // coneScale. Tradeoff: with arrowScale > coneScale the arrow no longer
    // renders literally to-scale against the cone (it'll look larger
    // relative to the cone than the real Fr/Fz_max ratio), so "tip inside
    // the cone" is a rough visual check, not an exact one -- if that
    // matters more than visibility, set this equal to coneScale instead.
    const double forceArrowScale = 0.002;

    // Duration to ramp joints to qIniDes
    const double rampDuration = 2.0;
    bool joystick_initialized = false;
    bool joystick_control_start = false;
    bool goUp = false;
    bool goDown = false;
    VectorXd tau_wbc;
    double lastQPStatusPrintTime = -1.0;
    const double qpStatusPrintPeriod = 1.0 / 5.0; // 5 Hz

    // Latest QP-solved contact wrench, cached from the physics-tick loop and
    // drawn/rendered only ONCE per rendered frame (below, right before
    // updateScene()) -- the inner while loop runs several mj_step()s per
    // frame (model timestep << 1/60s), and addArrow() calls accumulate into
    // custom_arrows_ until updateScene() clears it, so drawing every tick
    // stacked several ticks' worth of arrows on top of each other each
    // frame (the "weird"/cluttered look).
    Eigen::Vector3d lastContactForcePos_L = Eigen::Vector3d::Zero();
    Eigen::Vector3d lastContactForcePos_R = Eigen::Vector3d::Zero();
    Eigen::Vector3d lastFr_L = Eigen::Vector3d::Zero();
    Eigen::Vector3d lastFr_R = Eigen::Vector3d::Zero();
    bool haveContactForce = false;

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
                        joyStick.setPzRef(0.78, 3.0);
                        // std::cout << "[t=" << simTime << "] base height target -> 0.80m\n";
                    } else {
                        goDown = true;
                        goUp = false;
                        joyStick.setPzRef(0.75, 3.0);
                        // std::cout << "[t=" << simTime << "] base height target -> 0.75m\n";
                    }
                }

                // robot_wrapper.computeKin() already ran earlier this tick
                // (see the top of the outer while loop) -- kinematics are
                // fresh, no need to redo it here.
                // computeWBC_IK() now also solves for out_ddq (dynamically-
                // consistent acceleration-level IK, see KinWBC.cpp), which
                // needs robot_wrapper.dyn_M_inv populated -- otherwise
                // (e.g. on the very first tick, before computeDyn() has ever
                // run) it's an empty 0x0 matrix. NOTE: dyn_wbc.solveWBQP()
                // below calls robot_wrapper.computeDyn() again on its own
                // (inside updateRobotState()) -- computeDyn() currently
                // runs twice per tick on the same state as a result; a real
                // (if currently harmless) inefficiency worth deduplicating
                // later rather than right now.
                robot_wrapper.computeDyn();
                // CoM target stays centered between the feet (xc_/yc_/d_xc_/
                // d_yc_ = 0) -- CP_Planning's constructor already zero-inits
                // these, and nothing in this file ever mutates cp_planner
                // afterward (computeCoM()/planWarmingUp()/planWalking() are
                // never called here), so no per-tick reset is needed.

                kin_wbc.computeWBC_IK(joyStick, footPlanner, robot_wrapper, cp_planner, gaitScheduler);

                // const double stepSize = 1.0;
                // robot_wrapper_ref.integrateConfig(stepSize * kin_wbc.out_delta_q);
                // robot_wrapper_ref.dq = kin_wbc.out_dq;
                // robot_wrapper_ref.computeKin();

                dyn_wbc.solveWBQP(kin_wbc, robot_wrapper, state_estimator);

                if (simTime - lastQPStatusPrintTime >= qpStatusPrintPeriod) {
                    lastQPStatusPrintTime = simTime;
                    std::cout << "[t=" << simTime << "] QP status: "
                              << (dyn_wbc.getQPStatus() ? "OK" : "FAILED")
                              << "  |dq|=" << dyn_wbc.getDqNorm()
                              << "  solve_time=" << dyn_wbc.getLastSolveTimeUs() << "us"
                              << "  nWSR=" << dyn_wbc.getLastNWSR()
                              << "  base_z=" << robot_wrapper.pos_base_W(2)
                              << "  target_z=" << joyStick.pz_W << std::endl;
                }

                // Cache the QP-solved contact wrench for this tick -- the
                // actual drawing happens once per rendered frame (see below,
                // right before updateScene()), not here. Still push the
                // plot points every tick though, so ContactForcePlot gets
                // dense (1kHz-ish) data rather than one point per frame.
                // getOptimalContactWrench() is only meaningful when the QP
                // actually solved this tick -- DSt-only layout, left foot
                // first (see DynWBC::setupQPproblem()'s Jc_ stacking).
                if (dyn_wbc.getQPStatus()) {
                    VectorXd Fr = dyn_wbc.getOptimalContactWrench();
                    lastFr_L = Fr.segment<3>(0);
                    lastFr_R = Fr.segment<3>(6);
                    Eigen::Vector3d Mr_L = Fr.segment<3>(3); // left contact moment (tx,ty,tz)
                    Eigen::Vector3d Mr_R = Fr.segment<3>(9); // right contact moment (tx,ty,tz)
                    lastContactForcePos_L = robot_wrapper.pos_L_feet_W;
                    lastContactForcePos_R = robot_wrapper.pos_R_feet_W;
                    haveContactForce = true;

                    // ContactForcePlot.addPoint("Fx_L", simTime, lastFr_L(0));
                    ContactForcePlot.addPoint("Fy_L", simTime, lastFr_L(1));
                    // ContactForcePlot.addPoint("Fz_L", simTime, lastFr_L(2));
                    // ContactForcePlot.addPoint("Fx_R", simTime, lastFr_R(0));
                    ContactForcePlot.addPoint("Fy_R", simTime, lastFr_R(1));
                    // ContactForcePlot.addPoint("Fz_R", simTime, lastFr_R(2));

                    ContactMomentPlot.addPoint("Tx_L", simTime, Mr_L(0));
                    ContactMomentPlot.addPoint("Ty_L", simTime, Mr_L(1));
                    ContactMomentPlot.addPoint("Tz_L", simTime, Mr_L(2));
                    ContactMomentPlot.addPoint("Tx_R", simTime, Mr_R(0));
                    ContactMomentPlot.addPoint("Ty_R", simTime, Mr_R(1));
                    ContactMomentPlot.addPoint("Tz_R", simTime, Mr_R(2));
                }

                // Drive the robot with joint-space PD tracking of the
                // KinWBC-generated reference (position + velocity), PLUS
                // the DynWBC QP's feedforward joint torque (consistent
                // with the actually-solved contact wrench, not just the
                // kinematic reference) -- same pattern PVT_Ctr::calMotorsPVT()
                // already implements (tauDes = Kp*posErr + Kd*velErr +
                // tau_ff, then clamped to maxTor), matching how OpenLoong-
                // Dyn-Control's WBC output is consumed downstream.
                VectorXd ref_pos = kin_wbc.getMotorPosDes(); // kin_wbc.q_des joint segment
                VectorXd ref_vel = kin_wbc.out_dq.tail(robot_wrapper.model_na_);
                VectorXd tau_ff = dyn_wbc.getQPStatus()
                                       ? dyn_wbc.getOptimalJointTorque()
                                       : VectorXd::Zero(robot_wrapper.model_na_);
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

        // Draw the QP-solved contact wrench once per rendered frame (using
        // whatever the last physics tick this frame produced) -- force
        // vector + friction-cone wireframe at each foot. custom_arrows_ is
        // cleared inside updateScene() right after rendering, so this is a
        // true one-shot-per-frame draw, not accumulated across ticks.
        if (haveContactForce) {
            uiController.addArrow(lastContactForcePos_L, lastFr_L, forceArrowScale, colorLeftForce);
            uiController.addArrow(lastContactForcePos_R, lastFr_R, forceArrowScale, colorRightForce);
            drawFrictionCone(uiController, lastContactForcePos_L, dyn_wbc.getMuy(), dyn_wbc.getFzMax(), coneScale, colorCone);
            drawFrictionCone(uiController, lastContactForcePos_R, dyn_wbc.getMuy(), dyn_wbc.getFzMax(), coneScale, colorCone);
        }
        ContactForcePlot.render();
        ContactMomentPlot.render();
        uiController.updateScene();
        
    }

    // free visualization storage
    uiController.Close();
    return 0;
}

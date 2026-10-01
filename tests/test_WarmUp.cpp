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
#include "CP_Planning.h"
#include "RealtimePlot.h"

using namespace std;

const std::string URDF_PATH = "models/urdf/v2_biped_robot_12dof.urdf";
const std::string XML_PATH = "models/mjcf/scene_floatingbase_12dof_v2.xml";
const std::string YAML_PLANNING_CF_PATH = "config/step_planning_cf.yaml";
const std::string YAML_JOINT_CF_PATH = "config/12dof_joint_config.yaml";
const std::string YAML_QP_WBC_CF_PATH = "config/wbc_config.yaml";

char loadError[1024] = "";

// Draws a wireframe friction-cone pyramid at a contact point -- same as
// test_DynWBC.cpp's helper, kept here since this file also visualizes the
// QP-solved contact wrench and wants the same "does the force stay inside
// the cone" check while a foot is lifting/landing.
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
        uiController.addLine(contactPos, cornerPos[i], rgba);
        uiController.addLine(cornerPos[i], cornerPos[(i + 1) % 4], rgba);
    }
}

// Real MuJoCo physics + PVT torque control + DynWBC's whole-body QP driving
// the actual simulated robot -- same architecture as test_DynWBC.cpp (which
// only ever holds a plain double-support stand), extended here with the
// WARM_UP motion (CoM sway + foot lift) that's already confirmed stable
// KINEMATICALLY in test_WBCKin_stand.cpp. This is the first time that
// motion is driven through real dynamics/the QP instead of a pure
// forward-kinematics integrator, so treat it as exploratory.
//
// DynWBC::solveWBQP()'s current_leg_state must match whatever KinWBC's
// active task list actually assumed that tick (see DynWBC.h's comment) --
// while warmUpStarted, that's cp_planner.leg_state_swing_ (the ONE-CYCLE-
// DELAYED leg state FootPlacement's cp_planner-driven overload also keys
// off, see CP_Planning.h), not gaitScheduler.legState directly.
int main()
{
    mjModel *mj_model = mj_loadXML(XML_PATH.c_str(), nullptr, loadError, sizeof(loadError));
    if (!mj_model)
    {
        std::fprintf(stderr, "failed to load %s: %s\n", XML_PATH.c_str(), loadError);
        return 1;
    }
    mjData *mj_data = mj_makeData(mj_model);
    std::cout << "Compile mujoco xml done\n";

    RobotWrapper robot_wrapper = RobotWrapper(URDF_PATH, false); // real, Kalman-filtered
    // computeWBC_IK() operates on this SEPARATE belief state instead of the
    // real robot_wrapper: seeded once from the real settled state right
    // when warm-up starts, then advanced ONLY by internal integration
    // (integrateConfig(out_delta_q)) each tick, never re-synced from sensor
    // feedback again. DynWBC and the low-level PVT tracking still use the
    // REAL robot_wrapper -- only computeWBC_IK()'s input is blind to
    // feedback. Closing that loop (feeding computeWBC_IK() the real,
    // sensor-fed state directly) destabilizes it -- confirmed earlier this
    // session: the real-feedback version held plain STAND fine but fell
    // within ~1s of the first actual foot lift (dyn_pseudoInv()'s
    // near-singular amplification feeding back on itself once real sensor
    // noise/motion enters the loop every tick).
    RobotWrapper robot_wrapper_belief = RobotWrapper(URDF_PATH, false);
    KinWBC kin_wbc(YAML_QP_WBC_CF_PATH);
    DynWBC dyn_wbc(YAML_JOINT_CF_PATH, YAML_QP_WBC_CF_PATH, robot_wrapper, true);
    RobotSensor rb_sensors(mj_model->na);
    JoyStickInterpreter joyStick(kin_wbc.dt);
    MyGaitScheduler gaitScheduler(YAML_PLANNING_CF_PATH, kin_wbc.dt);
    FootPlacement footPlanner(YAML_PLANNING_CF_PATH, robot_wrapper);

    const double dt = kin_wbc.dt;
    const double zc = 0.5; // close enough to this robot's actual bent-knee CoM height (~0.497, see test_WBCKin_stand.cpp) to size CP_Planning's LIPM frequency
    CP_Planning cp_planner(dt, zc, footPlanner.hip_width);

    UIctr uiController(mj_model, mj_data);
    MJ_Interface mj_interface(mj_model, mj_data, YAML_JOINT_CF_PATH.c_str());

    PVT_Ctr pvtCtr(mj_model->opt.timestep, YAML_JOINT_CF_PATH.c_str());
    StateEstimator state_estimator(mj_model->opt.timestep, false);

    const double init_base_height = 0.8;
    VectorXd qIniDes = robot_wrapper.computeInitial_Stand(init_base_height);
    std::cout << "init joint position: " << qIniDes.transpose() << std::endl;

    // Spawn robot in home configuration (all actuated joints are 0)
    mj_data->qpos[0] = 0.0;
    mj_data->qpos[1] = 0.0;
    mj_data->qpos[2] = 0.80;
    mj_data->qpos[3] = 1.0;
    mj_data->qpos[4] = 0.0;
    mj_data->qpos[5] = 0.0;
    mj_data->qpos[6] = 0.0;
    for (int i = 0; i < robot_wrapper.model_na_; i++)
        mj_data->qpos[7 + i] = 0.0;
    mju_zero(mj_data->qvel, mj_model->nv);
    mju_zero(mj_data->qacc, mj_model->nv);
    mj_forward(mj_model, mj_data);

    mj_interface.updateSensorValues(rb_sensors);
    state_estimator.update(rb_sensors, robot_wrapper);
    robot_wrapper.computeKin();
    footPlanner.legLength = robot_wrapper.pos_base_W(2);
    std::cout << "Initial base height: " << robot_wrapper.pos_base_W(2) << "\n";

    mjtNum simstart = mj_data->time;
    double simTime = mj_data->time;

    uiController.iniGLFW();
    uiController.disableTracking();
    uiController.createWindow("Warm-Up (real dynamics)", false);

    RealtimePlot BaseHeightPlot(mj_model, 800, 600, "Base Height (Reference vs Estimated)", 5.0);
    BaseHeightPlot.setYLabel("meters");
    BaseHeightPlot.setYLimit(0.70, 0.90, true);
    BaseHeightPlot.setLineWidth(2.5f);

    RealtimePlot ContactForcePlot(mj_model, 800, 600, "Optimal Contact Force (WBC QP)", 5.0);
    ContactForcePlot.setYLabel("N");
    ContactForcePlot.setYLimit(-2, 2, true);
    ContactForcePlot.setLineWidth(2.5f);

    RealtimePlot ContactMomentPlot(mj_model, 800, 600, "Optimal Contact Moment (WBC QP)", 5.0);
    ContactMomentPlot.setYLabel("N*m");
    ContactMomentPlot.setYLimit(-2, 2, true);
    ContactMomentPlot.setLineWidth(2.5f);

    const float colorLeftForce[4]  = {1.0f, 0.1f, 0.1f, 1.0f};
    const float colorRightForce[4] = {0.1f, 0.3f, 1.0f, 1.0f};
    const float colorCone[4]       = {0.2f, 0.8f, 0.2f, 0.35f};
    const double coneScale = 0.0004;
    const double forceArrowScale = 0.002;

    const double rampDuration = 3.0;
    const double stepSize = 1.0; // full out_delta_q correction per tick, integrating robot_wrapper_belief
    // Hold plain STAND for a while after the ramp so it's visibly settled
    // (near-zero velocity, QP solving OK every tick) before warm-up ever
    // starts -- isolates "did warm-up itself destabilize it" from "was
    // stand ever actually stable to begin with".
    const double warmUpStartTime = rampDuration + 5.0;
    bool joystick_initialized = false;
    bool warmUpStarted = false;
    double lastStatusPrintTime = -1.0;
    const double statusPrintPeriod = 1.0 / 5.0; // 5 Hz

    // Latest QP-solved contact wrench, cached per physics tick and drawn
    // once per rendered frame -- same pattern as test_DynWBC.cpp, see its
    // comment on why (avoids stacking several ticks' arrows per frame).
    Eigen::Vector3d lastContactForcePos_L = Eigen::Vector3d::Zero();
    Eigen::Vector3d lastContactForcePos_R = Eigen::Vector3d::Zero();
    Eigen::Vector3d lastFr_L = Eigen::Vector3d::Zero();
    Eigen::Vector3d lastFr_R = Eigen::Vector3d::Zero();
    bool haveContactForce = false;

    while (!glfwWindowShouldClose(uiController.window))
    {
        simstart = mj_data->time;
        while (mj_data->time - simstart < 1.0 / 60.0 && uiController.runSim)
        {
            mj_step(mj_model, mj_data);
            uiController.applyPerturbation();
            simTime = mj_data->time;

            mj_interface.updateSensorValues(rb_sensors);
            state_estimator.update(rb_sensors, robot_wrapper);
            robot_wrapper.computeKin();

            if (simTime >= rampDuration && !joystick_initialized)
            {
                // pz_W tracks BASE height directly (not CoM height) --
                // kin_task_stand AND kin_task_init_walk both use
                // task_base_height (see KinWBC.cpp's task lists), so unlike
                // test_WBCKin_stand.cpp there's no height-convention switch
                // at the warm-up transition, and no PzLGen.resetOut() is
                // needed there either.
                joyStick.setIniPos(robot_wrapper.pos_base_W(0), robot_wrapper.pos_base_W(1), robot_wrapper.pos_base_W(2), 0.0);
                joyStick.setMotionState(MotionState::STAND);
                joyStick.setVxDesLPara(0.0, 0.1);
                joyStick.setVyDesLPara(0.0, 0.1);
                joyStick.setWzDesLPara(0.0, 0.1);
                joyStick.setPzRef(robot_wrapper.pos_base_W(2), 3.0);

                // Seed the CoM XY reference at the robot's actual current
                // position instead of leaving it at cp_planner's
                // constructor default (0,0) -- task_CoMXY.X_des reads
                // cp_planner.xc_/yc_ directly, and world-frame (0,0) is
                // essentially never where the robot actually is, so leaving
                // it at 0 made KinWBC continuously push the robot toward a
                // bogus target, fighting the state estimator instead of
                // just holding the current stand -- confirmed as the actual
                // divergence trigger for plain STAND in test_DynWBC.cpp's
                // HEAD baseline (diverges within ~0.2s of joystick init
                // without this seeding).
                cp_planner.xc_ = robot_wrapper.pos_CoM_W(0);
                cp_planner.yc_ = robot_wrapper.pos_CoM_W(1);
                cp_planner.d_xc_ = 0.0;
                cp_planner.d_yc_ = 0.0;
                // Also bias planWarmingUp()'s targets by this same position
                // -- otherwise cxi_xd_/cxi_yd_ are 0/+-0.5*wd_hip in
                // absolute world-frame terms, pulling the CoM toward world
                // (0,0) instead of around the robot's actual stance.
                cp_planner.xBias = robot_wrapper.pos_CoM_W(0);
                cp_planner.yBias = robot_wrapper.pos_CoM_W(1);
                // Capture point state -- separate from xc_/yc_ (CoM), and
                // NEVER otherwise seeded (stays at the constructor's 0
                // until computeCP() integrates it). Because b=e^(w*t_swing)
                // is huge for this robot, the boundary-value blend
                // px_d_=(cxi_xd_-b*cxi_x0_)/(1-b) is dominated by cxi_x0_
                // as b->inf -- so leaving this at 0 pins the ZMP reference
                // to the world origin regardless of xBias/cxi_xd_, and the
                // CoM ODE (which pulls xc_ toward cxi_x_) then drags the
                // correctly-seeded xc_ back toward 0 too.
                cp_planner.cxi_x_ = robot_wrapper.pos_CoM_W(0);
                cp_planner.cxi_y_ = robot_wrapper.pos_CoM_W(1);

                footPlanner.legLength = robot_wrapper.pos_base_W(2);
                joystick_initialized = true;
                std::cout << "[t=" << simTime << "] Joystick initialized to measured base height: " << robot_wrapper.pos_base_W(2) << "\n";
            }

            if (joystick_initialized)
            {
                joyStick.step();
                gaitScheduler.step(joyStick);

                BaseHeightPlot.addPoint("Reference Pz", simTime, joyStick.pz_W);
                BaseHeightPlot.addPoint("Estimated Pz", simTime, robot_wrapper.pos_base_W(2));

                if (!warmUpStarted && simTime >= warmUpStartTime)
                {
                    joyStick.setMotionState(MotionState::WARM_UP);
                    gaitScheduler.start(joyStick);
                    footPlanner.inPlaceOnly = true; // sway/lift in place, no forward stepping

                    // Re-seed the belief-state wrapper from the REAL
                    // current state right at this transition -- see the
                    // comment at robot_wrapper_belief's declaration.
                    robot_wrapper_belief.q = robot_wrapper.q;
                    robot_wrapper_belief.dq = robot_wrapper.dq;
                    robot_wrapper_belief.computeKin();

                    warmUpStarted = true;
                    std::cout << "[t=" << simTime << "] Starting warm-up CoM sway + foot lift\n";
                }

                RobotWrapper &ik_robot = warmUpStarted ? robot_wrapper_belief : robot_wrapper;
                if (warmUpStarted)
                {
                    cp_planner.planWarmingUp(gaitScheduler);
                    footPlanner.StepSwingPlanning(robot_wrapper_belief, joyStick, cp_planner);
                }

                robot_wrapper.computeDyn();

                kin_wbc.computeWBC_IK(joyStick, footPlanner, ik_robot, cp_planner, gaitScheduler);

                if (warmUpStarted)
                {
                    robot_wrapper_belief.integrateConfig(stepSize * kin_wbc.out_delta_q);
                    robot_wrapper_belief.dq = kin_wbc.out_dq;
                    robot_wrapper_belief.computeKin();
                }

                // leg_state_swing_: the ONE-CYCLE-DELAYED leg state (see
                // CP_Planning.h) -- matches whichever foot KinWBC's active
                // task list ACTUALLY treated as the rigid stance foot this
                // same tick (task_static_contact/task_lift_foot, see
                // KinWBC.cpp), unlike gaitScheduler.legState (the CURRENT
                // weight-shift TARGET, not yet reached). Defaults to DSt
                // while !warmUpStarted, matching KinWBC's double-support
                // kin_task_stand list.
                dyn_wbc.solveWBQP(kin_wbc, robot_wrapper, state_estimator,
                                   warmUpStarted ? cp_planner.leg_state_swing_ : LegState::DSt);

                if (simTime - lastStatusPrintTime >= statusPrintPeriod) {
                    lastStatusPrintTime = simTime;
                    std::cout << "[t=" << simTime << "] QP status: "
                              << (dyn_wbc.getQPStatus() ? "OK" : "FAILED")
                              << "  |dq|=" << dyn_wbc.getDqNorm()
                              << "  solve_time=" << dyn_wbc.getLastSolveTimeUs() << "us"
                              << "  nWSR=" << dyn_wbc.getLastNWSR()
                              << "  base_z=" << robot_wrapper.pos_base_W(2)
                              << "  target_z=" << joyStick.pz_W << std::endl;
                }

                if (dyn_wbc.getQPStatus()) {
                    VectorXd Fr = dyn_wbc.getOptimalContactWrench();
                    lastFr_L = Fr.segment<3>(0);
                    Eigen::Vector3d Mr_L = Fr.segment<3>(3);
                    lastContactForcePos_L = robot_wrapper.pos_L_feet_W;
                    haveContactForce = true;

                    ContactForcePlot.addPoint("Fy_L", simTime, lastFr_L(1));
                    ContactMomentPlot.addPoint("Tx_L", simTime, Mr_L(0));
                    ContactMomentPlot.addPoint("Ty_L", simTime, Mr_L(1));
                    ContactMomentPlot.addPoint("Tz_L", simTime, Mr_L(2));

                    // Fr is 6-dim (single stance) once warm-up starts and
                    // DSt (12-dim, left-then-right) before that -- only
                    // read the right-foot block while it's actually there.
                    if (Fr.size() >= 12) {
                        lastFr_R = Fr.segment<3>(6);
                        Eigen::Vector3d Mr_R = Fr.segment<3>(9);
                        lastContactForcePos_R = robot_wrapper.pos_R_feet_W;
                        ContactForcePlot.addPoint("Fy_R", simTime, lastFr_R(1));
                        ContactMomentPlot.addPoint("Tx_R", simTime, Mr_R(0));
                        ContactMomentPlot.addPoint("Ty_R", simTime, Mr_R(1));
                        ContactMomentPlot.addPoint("Tz_R", simTime, Mr_R(2));
                    }
                }

                VectorXd ref_pos = kin_wbc.getMotorPosDes();
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
                double rampFrac = std::min(simTime / rampDuration, 1.0);
                VectorXd rampedJointPos = rampFrac * qIniDes;

                pvtCtr.motor_pos_des = rampedJointPos;
                pvtCtr.motor_vel_des = VectorXd::Zero(robot_wrapper.model_na_);
                pvtCtr.motor_tor_des = VectorXd::Zero(robot_wrapper.model_na_);
                pvtCtr.getFeedbackMotorState(robot_wrapper);
                pvtCtr.calMotorsPVT();
                mj_interface.setMotorsTorque(pvtCtr.motor_tor_out_motor);
            }
        }

        if (haveContactForce) {
            uiController.addArrow(lastContactForcePos_L, lastFr_L, forceArrowScale, colorLeftForce);
            drawFrictionCone(uiController, lastContactForcePos_L, dyn_wbc.getMuy(), dyn_wbc.getFzMax(), coneScale, colorCone);
            if (!warmUpStarted || gaitScheduler.legState != LegState::DSt) {
                uiController.addArrow(lastContactForcePos_R, lastFr_R, forceArrowScale, colorRightForce);
                drawFrictionCone(uiController, lastContactForcePos_R, dyn_wbc.getMuy(), dyn_wbc.getFzMax(), coneScale, colorCone);
            }
        }

        // CoM/ZMP markers -- same LIPM "pendulum" picture as
        // test_WBCKin_stand.cpp's CPPlot, just in 3D.
        static const float colorCoM[4] = {1.0f, 0.85f, 0.1f, 0.9f};
        Eigen::Vector3d comMarkerPos_W = robot_wrapper.pos_CoM_W + Eigen::Vector3d(0.0, 0.0, 0.02);
        uiController.addSphere(comMarkerPos_W, 0.025, colorCoM);

        static const float colorZMP[4] = {0.9f, 0.2f, 0.2f, 0.9f};
        Eigen::Vector3d zmpMarkerPos_W(cp_planner.px_d_, cp_planner.py_d_, 0.02);
        uiController.addSphere(zmpMarkerPos_W, 0.02, colorZMP);
        uiController.addLine(zmpMarkerPos_W, comMarkerPos_W, colorZMP, 2.0);

        BaseHeightPlot.render();
        ContactForcePlot.render();
        ContactMomentPlot.render();
        uiController.updateScene();
    }

    std::cout << "Final base height: " << robot_wrapper.pos_base_W(2) << "\n";
    uiController.Close();
    return 0;
}

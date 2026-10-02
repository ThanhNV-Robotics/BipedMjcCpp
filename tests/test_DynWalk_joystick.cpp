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

const std::string URDF_PATH = "models/urdf/v2_biped_robot_12dof.urdf";
const std::string XML_PATH = "models/mjcf/scene_floatingbase_12dof_v2.xml";
const std::string YAML_PLANNING_CF_PATH = "config/step_planning_cf.yaml";
const std::string YAML_JOINT_CF_PATH = "config/12dof_joint_config.yaml";
const std::string YAML_QP_WBC_CF_PATH = "config/wbc_config.yaml";

char loadError[1024] = "";

// Draws a wireframe friction-cone pyramid at a contact point -- same helper
// as test_DynBaseHeightWBC.cpp, see its comment for the derivation.
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

// Real MuJoCo physics + PVT torque control + DynWBC's delta_r/delta_Fr QP
// (Phase 2) driving the actual simulated robot, with KinWBC's
// CP_Planning-free overload (Phase 1b: task_CoMXY tracks the joystick's
// own integrated position) + the postural task (Phase 1a: biases the
// stance leg toward a nominal bent-knee configuration) + FootPlacement's
// pure Raibert heuristic (CP_Planning-free StepSwingPlanning overload) for
// swing-foot placement. NO CP_Planning anywhere in this file -- this is
// the OpenLoong-Dyn-Control architecture applied as directly as possible:
// STAND (Vcom=0, held) -> WALK, no separate warm-up/sway phase, matching
// walk_wbc_joystick.cpp's own motionState transitions exactly (see
// plan.md's "Confirmed non-issue" section on what OpenLoong's "warm-up"
// actually is).
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

    RobotWrapper robot_wrapper = RobotWrapper(URDF_PATH, false);
    KinWBC kin_wbc(YAML_QP_WBC_CF_PATH);
    DynWBC dyn_wbc(YAML_JOINT_CF_PATH, YAML_QP_WBC_CF_PATH, robot_wrapper, true);
    RobotSensor rb_sensors(mj_model->na);
    JoyStickInterpreter joyStick(kin_wbc.dt);
    MyGaitScheduler gaitScheduler(YAML_PLANNING_CF_PATH, kin_wbc.dt);
    FootPlacement footPlanner(YAML_PLANNING_CF_PATH, robot_wrapper);

    // OpenLoong's own demo (walk_wbc_joystick.cpp) sets these directly,
    // not from yaml -- Raibert velocity-error feedback gain (rotated into
    // world frame by yaw inside StepSwingPlanning()).
    footPlanner.kp_vx = 0.03;
    footPlanner.kp_vy = 0.03;
    footPlanner.kp_wz = 0.03;

    UIctr uiController(mj_model, mj_data);
    MJ_Interface mj_interface(mj_model, mj_data, YAML_JOINT_CF_PATH.c_str());

    PVT_Ctr pvtCtr(mj_model->opt.timestep, YAML_JOINT_CF_PATH.c_str());
    StateEstimator state_estimator(mj_model->opt.timestep, false);

    const double init_base_height = 0.8;
    VectorXd qIniDes = robot_wrapper.computeInitial_Stand(init_base_height);
    printf("Init standing joint config: \n");
    std::cout << qIniDes.transpose() << std::endl;
    // Postural task (Phase 1a, KinWBC.h's task_posture) target -- the
    // nominal bent-knee configuration, so the stance leg gets softly
    // pulled back toward this instead of drifting toward full-extension
    // singularity during single-stance WALK.
    kin_wbc.posture_nominal_ = qIniDes;

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
    uiController.createWindow("OpenLoong joystick+Raibert walk, real dynamics (no CP_Planning)", false);

    RealtimePlot BaseHeightPlot(mj_model, 800, 600, "Base Height (Reference vs Estimated)", 5.0);
    BaseHeightPlot.setYLabel("meters");
    BaseHeightPlot.setYLimit(0.70, 0.90, true);
    BaseHeightPlot.setLineWidth(2.5f);

    RealtimePlot BasePosPlot(mj_model, 800, 600, "Base X Position (Reference vs Estimated)", 5.0);
    BasePosPlot.setYLabel("meters");
    BasePosPlot.setYLimit(-0.1, 0.5, true);
    BasePosPlot.setLineWidth(2.5f);

    const float colorLeftForce[4]  = {1.0f, 0.1f, 0.1f, 1.0f};
    const float colorRightForce[4] = {0.1f, 0.3f, 1.0f, 1.0f};
    const float colorCone[4]       = {0.2f, 0.8f, 0.2f, 0.35f};
    const double coneScale = 0.0004;
    const double forceArrowScale = 0.002;

    const double rampDuration = 3.0;
    bool joystick_initialized = false;
    // Straight STAND -> WALK, no separate warm-up sway phase -- see this
    // file's header comment.
    const double startWalkingTime = rampDuration + 3.0;
    const double walkVx = 0.0; // forward speed command, m/s
    bool walkingStarted = false;
    double lastQPStatusPrintTime = -1.0;
    const double qpStatusPrintPeriod = 1.0 / 5.0;

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
                // task_base_height compares pz_W directly against
                // pos_base_W(2) (see KinWBC.cpp's kin_task_stand/
                // kin_task_walk -- task_CoMZc is never in either list), so
                // pz_W targets base height directly, no CoM-height
                // conversion needed.
                joyStick.setIniPos(robot_wrapper.pos_base_W(0), robot_wrapper.pos_base_W(1), robot_wrapper.pos_base_W(2), 0.0);
                joyStick.setMotionState(MotionState::STAND);
                joyStick.setVxDesLPara(0.0, 0.1);
                joyStick.setVyDesLPara(0.0, 0.1);
                joyStick.setWzDesLPara(0.0, 0.1);
                joyStick.setPzRef(robot_wrapper.pos_base_W(2), 3.0);

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
                BasePosPlot.addPoint("Reference Px", simTime, joyStick.px_W);
                BasePosPlot.addPoint("Estimated Px", simTime, robot_wrapper.pos_base_W(0));

                if (simTime >= startWalkingTime && !walkingStarted)
                {
                    joyStick.setMotionState(MotionState::WALK);
                    joyStick.setVxDesLPara(walkVx, 1.0);
                    gaitScheduler.start(joyStick);

                    // Re-seed the joystick's integrated position to the
                    // robot's ACTUAL current CoM-xy position right at this
                    // transition. During STAND, task_CoMXY.X_des used the
                    // midpoint-of-feet override (see KinWBC.cpp's
                    // updateReference()), NOT px_W/py_W -- those stayed
                    // frozen at whatever they were seeded to back at
                    // joystick_initialized (several seconds earlier) since
                    // vx_W/vy_W are 0 throughout STAND. If the robot's
                    // actual held position (via the midpoint override)
                    // drifted at all from that original seed, switching to
                    // px_W/py_W-based tracking at WALK start creates an
                    // instant CoM-position discontinuity -- task_CoMXY.errX
                    // jumping to whatever that gap is, right as the task
                    // list also switches -- instead of the "it's trying to
                    // track CoM back to a stale/zero-ish reference" tilt
                    // this produces.
                    joyStick.px_W = robot_wrapper.pos_CoM_W(0);
                    joyStick.py_W = robot_wrapper.pos_CoM_W(1);

                    walkingStarted = true;
                    std::cout << "[t=" << simTime << "] Starting forward walk, vx_des=" << walkVx << " m/s\n";
                }

                if (walkingStarted)
                    footPlanner.StepSwingPlanning(robot_wrapper, gaitScheduler, joyStick);

                robot_wrapper.computeDyn();

                kin_wbc.computeWBC_IK(joyStick, footPlanner, robot_wrapper, gaitScheduler);

                dyn_wbc.solveWBQP(kin_wbc, robot_wrapper, state_estimator,
                                   walkingStarted ? gaitScheduler.legState : LegState::DSt);

                if (simTime - lastQPStatusPrintTime >= qpStatusPrintPeriod) {
                    lastQPStatusPrintTime = simTime;
                    std::cout << "[t=" << simTime << "] QP status: "
                              << (dyn_wbc.getQPStatus() ? "OK" : "FAILED")
                              << "  |dq|=" << dyn_wbc.getDqNorm()
                              << "  base_x=" << robot_wrapper.pos_base_W(0)
                              << "  base_z=" << robot_wrapper.pos_base_W(2)
                              << "  target_z=" << joyStick.pz_W
                              << "  legState=" << (int)gaitScheduler.legState << std::endl;
                }

                if (dyn_wbc.getQPStatus()) {
                    VectorXd Fr = dyn_wbc.getOptimalContactWrench();
                    lastFr_L = Fr.segment<3>(0);
                    lastContactForcePos_L = robot_wrapper.pos_L_feet_W;
                    haveContactForce = true;
                    if (Fr.size() >= 12) {
                        lastFr_R = Fr.segment<3>(6);
                        lastContactForcePos_R = robot_wrapper.pos_R_feet_W;
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

                // Gravity-compensation feedforward -- see
                // test_DynBaseHeightWBC.cpp's identical ramp-branch fix:
                // with the lower gist_humanoid_mpc Kp/Kd gains now in
                // 12dof_joint_config.yaml, position-error-only PD let the
                // robot sink during this open-loop ramp (confirmed: base_z
                // already negative by the time WALK started). computeDyn()
                // must run here too since it's normally only called inside
                // the joystick_initialized branch above.
                robot_wrapper.computeDyn();
                VectorXd tau_gravity = robot_wrapper.computeDoubleSupportGravityTorque();

                pvtCtr.getFeedbackMotorState(robot_wrapper);
                pvtCtr.calMotorsPVT(rampedJointPos, VectorXd::Zero(robot_wrapper.model_na_), tau_gravity);
                mj_interface.setMotorsTorque(pvtCtr.motor_tor_out_motor);
            }
        }

        if (haveContactForce) {
            uiController.addArrow(lastContactForcePos_L, lastFr_L, forceArrowScale, colorLeftForce);
            drawFrictionCone(uiController, lastContactForcePos_L, dyn_wbc.getMuy(), dyn_wbc.getFzMax(), coneScale, colorCone);
            if (!walkingStarted || gaitScheduler.legState != LegState::DSt) {
                uiController.addArrow(lastContactForcePos_R, lastFr_R, forceArrowScale, colorRightForce);
                drawFrictionCone(uiController, lastContactForcePos_R, dyn_wbc.getMuy(), dyn_wbc.getFzMax(), coneScale, colorCone);
            }
        }

        static const float colorCoM[4] = {1.0f, 0.85f, 0.1f, 0.9f};
        Eigen::Vector3d comMarkerPos_W = robot_wrapper.pos_CoM_W + Eigen::Vector3d(0.0, 0.0, 0.02);
        uiController.addSphere(comMarkerPos_W, 0.025, colorCoM);

        BaseHeightPlot.render();
        BasePosPlot.render();
        uiController.updateScene();
    }

    std::cout << "Final base height: " << robot_wrapper.pos_base_W(2) << "\n";
    uiController.Close();
    return 0;
}

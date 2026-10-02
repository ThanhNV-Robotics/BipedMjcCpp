#include <cmath>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <vector>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include "GLFW_callbacks.h"
#include "RealtimePlot.h"
#include "data_type.h"
#include "robot_wrapper.h"
#include "KinWBC.h"
#include "joystick_interpreter.h"
#include "foot_placement.h"
#include "my_gait_scheduler.h"

const std::string URDF_PATH = "models/urdf/v2_biped_robot_12dof.urdf";
const std::string XML_PATH = "models/mjcf/scene_floatingbase_12dof_v2.xml";
const std::string STEP_PLANNING_CF_PATH = "config/step_planning_cf.yaml";
const std::string YAML_QP_WBC_CF_PATH = "config/wbc_config.yaml";

// Pure forward-kinematics check of the OpenLoong-Dyn-Control WALKING
// pipeline applied as directly as possible: NO CP_Planning/capture-point
// planner anywhere in this file. Base-xy tracking comes straight from the
// joystick's own integrated position (joyStick.px_W/py_W, updated every
// step() from vx_W/vy_W -- see joystick_interpreter.cpp), and swing-foot
// placement comes from FootPlacement's pure Raibert heuristic (the
// CP_Planning-free StepSwingPlanning() overload, ported from OpenLoong's
// FootPlacement::getSwingPos()), not a boundary-value ZMP blend. Compare
// against test_WBCKin_walk.cpp (same harness, but CP_Planning-driven) to
// see how the two approaches differ in practice on this robot.
int main()
{
    char loadError[1024] = "";
    mjModel *mj_model = mj_loadXML(XML_PATH.c_str(), nullptr, loadError, sizeof(loadError));
    if (!mj_model)
    {
        std::fprintf(stderr, "failed to load %s: %s\n", XML_PATH.c_str(), loadError);
        return 1;
    }
    mjData *mj_data = mj_makeData(mj_model);

    RobotWrapper robot_wrapper(URDF_PATH);
    KinWBC kin_wbc(YAML_QP_WBC_CF_PATH);

    // CSV log of KinWBC's joint-space command (position + velocity) every
    // tick, for offline Python plotting -- see record/README or just load
    // with pandas.read_csv(). record/ is gitignored.
    std::ofstream jointCmdLog("record/test_WBCKin_walk_joystick_joint_cmd.csv");
    jointCmdLog << "time";
    for (const auto &name : robot_wrapper.jointNames_)
        jointCmdLog << "," << name << "_pos_cmd";
    for (const auto &name : robot_wrapper.jointNames_)
        jointCmdLog << "," << name << "_vel_cmd";
    jointCmdLog << "\n";

    JoyStickInterpreter joyStick(kin_wbc.dt);
    MyGaitScheduler gaitScheduler(STEP_PLANNING_CF_PATH, kin_wbc.dt);
    FootPlacement footPlanner(STEP_PLANNING_CF_PATH, robot_wrapper);
    const double dt = 0.001;

    // OpenLoong's own demo (walk_wbc_joystick.cpp) sets these directly,
    // not from yaml -- Raibert velocity-error feedback gain (rotated into
    // world frame by yaw inside StepSwingPlanning()).
    footPlanner.kp_vx = 0.03;
    footPlanner.kp_vy = 0.03;
    footPlanner.kp_wz = 0.03;

    std::vector<int> jointQposAdr(robot_wrapper.jointNames_.size());
    std::vector<int> jointQvelAdr(robot_wrapper.jointNames_.size());
    for (size_t i = 0; i < robot_wrapper.jointNames_.size(); i++)
    {
        int jid = mj_name2id(mj_model, mjOBJ_JOINT, robot_wrapper.jointNames_[i].c_str());
        jointQposAdr[i] = mj_model->jnt_qposadr[jid];
        jointQvelAdr[i] = mj_model->jnt_dofadr[jid];
    }
    int freeJointId = mj_name2id(mj_model, mjOBJ_JOINT, "floating_base_joint");
    int freeQposAdr = mj_model->jnt_qposadr[freeJointId];
    int freeQvelAdr = mj_model->jnt_dofadr[freeJointId];

    UIctr uiController(mj_model, mj_data);
    uiController.iniGLFW();
    uiController.disableTracking();
    uiController.createWindow("OpenLoong joystick+Raibert walk (no CP_Planning)", false);

    RealtimePlot GaitPhasePlot(mj_model, 800, 600, "Gait Phase Plot", 5.0);
    GaitPhasePlot.setYLabel("Phase");
    GaitPhasePlot.setYLimit(0, 1);
    GaitPhasePlot.setLineWidth(2.5f);

    RealtimePlot BasePosPlot(mj_model, 800, 600, "Base X Position (Reference vs Estimated)", 5.0);
    BasePosPlot.setYLabel("meters");
    BasePosPlot.setYLimit(-0.1, 0.5, true);
    BasePosPlot.setLineWidth(2.5f);

    // Mirrors test_WBCKin_walk.cpp's init exactly.
    robot_wrapper.q.segment(7, robot_wrapper.model_na_).setZero();
    robot_wrapper.q(2) = 0.0;
    robot_wrapper.computeKin();
    const double straight_leg_length = -robot_wrapper.pos_L_feet_W(2);

    const double standLegLength = 0.79;
    robot_wrapper.q(2) = standLegLength;
    VectorXd init_joint_config = robot_wrapper.computeInitial_Stand(standLegLength);
    robot_wrapper.q.segment(7, robot_wrapper.model_na_) = init_joint_config;
    std::cout << "init joint position: " << init_joint_config.transpose() << std::endl;
    robot_wrapper.computeKin();

    footPlanner.legLength = robot_wrapper.pos_base_W(2);
    std::cout << "Straight-leg reach: " << straight_leg_length
               << "  Initial bent-knee base height: " << robot_wrapper.pos_base_W(2) << "\n";

    const double stepSize = 1.0;

    // base-height convention: task_base_height compares pz_W directly
    // against pos_base_W(2) (see KinWBC.cpp's kin_task_stand/kin_task_walk
    // -- task_CoMZc is never in either list), so pz_W targets base height
    // directly, no CoM-height conversion needed.
    //
    // px_W/py_W, however, feed task_CoMXY (a CoM-tracking task, see
    // updateReference()'s CP_Planning-free overload), whose X_cur is
    // robot_wrapper.pos_CoM_W -- NOT pos_base_W. During STAND, task_CoMXY's
    // target uses a separate midpoint-of-feet override instead of px_W/
    // py_W, so this mismatch stays latent; but px_W/py_W itself is seeded
    // here from pos_base_W and never touched again before WALK starts
    // tracking it directly. If the bent-knee stand posture puts the CoM
    // anywhere other than exactly above the base link's own origin, that
    // seeds an offset baked into px_W/py_W from tick 0 -- invisible during
    // STAND, but the instant WALK's task list switches to tracking px_W/
    // py_W, task_CoMXY.errX jumps to that offset immediately, producing an
    // impulsive one-tick correction instead of a smooth start. Seed from
    // the actual CoM position instead so px_W/py_W already matches
    // pos_CoM_W when WALK takes over.
    joyStick.setIniPos(robot_wrapper.pos_CoM_W(0), robot_wrapper.pos_CoM_W(1), robot_wrapper.pos_base_W(2), 0.0);
    joyStick.setMotionState(MotionState::STAND);
    joyStick.setVxDesLPara(0.0, 0.1);
    joyStick.setVyDesLPara(0.0, 0.1);
    joyStick.setWzDesLPara(0.0, 0.1);
    joyStick.setPzRef(robot_wrapper.pos_base_W(2), 0.01);

    double simTime = 0.0;
    // Straight STAND -> WALK, no separate warm-up sway phase -- OpenLoong's
    // own demos don't have one either; Raibert foot placement handles the
    // first step directly from a standing start.
    const double startWalkingTime = 2.0;
    const double walkVx = 0.0; // forward speed command, m/s
    bool walkingStarted = false;

    while (!glfwWindowShouldClose(uiController.window))
    {
        double frameStart = simTime;
        while (uiController.runSim && (simTime - frameStart) < 1.0 / 60.0)
        {
            robot_wrapper.computeKin();
            joyStick.step();
            gaitScheduler.step(joyStick);

            if (simTime >= startWalkingTime && !walkingStarted)
            {
                joyStick.setMotionState(MotionState::WALK);
                joyStick.setVxDesLPara(walkVx, 1.0);
                gaitScheduler.start(joyStick);
                walkingStarted = true;
                std::cout << "[t=" << simTime << "] Starting forward walk, vx_des=" << walkVx << " m/s\n";
            }

            if (walkingStarted)
            {
                footPlanner.StepSwingPlanning(robot_wrapper, gaitScheduler, joyStick);
                kin_wbc.computeWBC_IK(joyStick, footPlanner, robot_wrapper, gaitScheduler);

                VectorXd jointPosCmd = kin_wbc.getMotorPosDes();
                VectorXd jointVelCmd = kin_wbc.out_dq.tail(robot_wrapper.model_na_);
                jointCmdLog << simTime;
                for (int j = 0; j < jointPosCmd.size(); ++j)
                    jointCmdLog << "," << jointPosCmd(j);
                for (int j = 0; j < jointVelCmd.size(); ++j)
                    jointCmdLog << "," << jointVelCmd(j);
                jointCmdLog << "\n";
                jointCmdLog.flush(); // survive a SIGTERM/timeout kill, not just a clean exit

                robot_wrapper.integrateConfig(stepSize * kin_wbc.out_delta_q);
                // Feed the solved velocity back into robot_wrapper.dq --
                // without this, dq (hence vel_base_W, hence curV_W in
                // FootPlacement's Raibert formula) stays frozen at zero for
                // the whole run
                robot_wrapper.dq = kin_wbc.out_dq;
            }

            simTime += kin_wbc.dt;

            mj_data->qpos[freeQposAdr + 0] = robot_wrapper.q(0);
            mj_data->qpos[freeQposAdr + 1] = robot_wrapper.q(1);
            mj_data->qpos[freeQposAdr + 2] = robot_wrapper.q(2) + 0.08;
            mj_data->qpos[freeQposAdr + 3] = robot_wrapper.q(6);
            mj_data->qpos[freeQposAdr + 4] = robot_wrapper.q(3);
            mj_data->qpos[freeQposAdr + 5] = robot_wrapper.q(4);
            mj_data->qpos[freeQposAdr + 6] = robot_wrapper.q(5);

            mj_data->qvel[freeQvelAdr + 0] = robot_wrapper.dq(0);
            mj_data->qvel[freeQvelAdr + 1] = robot_wrapper.dq(1);
            mj_data->qvel[freeQvelAdr + 2] = robot_wrapper.dq(2);
            mj_data->qvel[freeQvelAdr + 3] = robot_wrapper.dq(3);
            mj_data->qvel[freeQvelAdr + 4] = robot_wrapper.dq(4);
            mj_data->qvel[freeQvelAdr + 5] = robot_wrapper.dq(5);

            for (size_t j = 0; j < robot_wrapper.jointNames_.size(); j++)
            {
                mj_data->qpos[jointQposAdr[j]] = robot_wrapper.q(7 + j);
                mj_data->qvel[jointQvelAdr[j]] = robot_wrapper.dq(6 + j);
            }

            mj_forward(mj_model, mj_data);
        }

        GaitPhasePlot.addPoint("Phase", simTime, gaitScheduler.phi);
        GaitPhasePlot.render();

        BasePosPlot.addPoint("Reference Px", simTime, joyStick.px_W);
        BasePosPlot.addPoint("Estimated Px", simTime, robot_wrapper.pos_base_W(0));
        BasePosPlot.render();

        static const float colorCoM[4] = {1.0f, 0.85f, 0.1f, 0.9f};
        Eigen::Vector3d comMarkerPos_W = robot_wrapper.pos_CoM_W + Eigen::Vector3d(0.0, 0.0, 0.08);
        uiController.addSphere(comMarkerPos_W, 0.025, colorCoM);

        uiController.updateScene();
    }
    uiController.Close();
    return 0;
}

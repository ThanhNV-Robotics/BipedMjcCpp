#include <cmath>
#include <cstdlib>
#include <iostream>
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
#include "CP_Planning.h"

const std::string URDF_PATH = "models/urdf/biped_robot_12dof.urdf";
const std::string XML_PATH = "models/mjcf/scene_floatingbase_12dof.xml";
const std::string STEP_PLANNING_CF_PATH = "config/step_planning_cf.yaml";

// Pure forward-kinematics check of KinWBC::computeWBC_IK() -- no MuJoCo
// physics (mj_step) involved at all. Each iteration: recompute Jacobians/
// positions from the current q (computeKin), solve the null-space priority
// IK (computeWBC_IK), integrate q by the resulting out_delta_q (a Newton-
// style correction, so it's damped by stepSize below rather than applied in
// full), then puppet the MuJoCo robot's qpos/qvel directly from q/dq and call
// mj_forward (not mj_step) so it's just rendered, not simulated -- this way
// what's on screen is exactly robot_wrapper's kinematic state, nothing else.
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
    KinWBC kin_wbc;
    JoyStickInterpreter joyStick(kin_wbc.dt);
    MyGaitScheduler gaitScheduler(STEP_PLANNING_CF_PATH, kin_wbc.dt);
    FootPlacement footPlanner(STEP_PLANNING_CF_PATH);
    const double dt = 0.001;
    const double zc = 0.35;
    CP_Planning cp_planning(dt, zc, footPlanner.hip_width);

    // per-joint MuJoCo qpos/qvel address, looked up by name in
    // robot_wrapper.jointNames_'s order (Pinocchio/URDF order) -- matches
    // robot_wrapper.q's joint segment 1:1 without assuming XML declaration
    // order lines up with it
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
    uiController.createWindow("KinWBC forward-kinematics check", false);


    // Signal plotting 
    RealtimePlot HeightPlot(mj_model, 600, 400, "Base Height Tracking", 6.0);
    HeightPlot.setYLabel("meters");
    HeightPlot.setYLimit(0.60, 0.80); 
    HeightPlot.setLineWidth(2.5f);

    RealtimePlot FootPlot(mj_model, 600, 400, "Foot Height (Contact Constraint)", 6.0);
    FootPlot.setYLabel("meters");
    FootPlot.setYLimit(-0.05, 0.05);
    FootPlot.setLineWidth(2.5f);

    RealtimePlot CoMPlot(mj_model, 600, 400, "CoM Tracking", 6.0);
    CoMPlot.setYLabel("meters");
    CoMPlot.setYLimit(-0.1, 0.1);
    CoMPlot.setLineWidth(2.5f);

    // Starting at nominal bent configuration to avoid singularity
    const double standLegLength = 0.72;
    robot_wrapper.q(2) = standLegLength; // init initial base height
    robot_wrapper.q.segment(7, robot_wrapper.model_na_) = robot_wrapper.computeInitial_Stand(standLegLength);
    footPlanner.legLength = standLegLength;

    robot_wrapper.computeKin();
    const double initial_height = robot_wrapper.pos_base_W(2);
    std::cout << "Initial base height: " << initial_height << "\n";

    const double stepSize = 1.0;

    // Initialize joystick and planners strictly in STAND mode
    joyStick.setIniPos(robot_wrapper.pos_base_W(0), robot_wrapper.pos_base_W(1), robot_wrapper.pos_base_W(2), 0.0);
    joyStick.setMotionState(MotionState::STAND);
    joyStick.setVxDesLPara(0.0, 0.1); // zero horizontal velocities for standing
    joyStick.setVyDesLPara(0.0, 0.1);
    joyStick.setWzDesLPara(0.0, 0.1);
    joyStick.setPzRef(standLegLength, 0.01); // hold initial height

    cp_planning.xc_ = robot_wrapper.pos_CoM_W(0);
    cp_planning.yc_ = robot_wrapper.pos_CoM_W(1);
    cp_planning.d_xc_ = 0.0;
    cp_planning.d_yc_ = 0.0;

    double simTime = 0.0;
    bool squat_started = false;
    bool stand_up_started = false;

    while (!glfwWindowShouldClose(uiController.window))
    {
        double frameStart = simTime;
        while (uiController.runSim && (simTime - frameStart) < 1.0 / 60.0) // press "1" to pause/resume, "2" to step
        {
            // Trajectory profile:
            // 0s - 1.5s: Hold initial stand at 0.72m
            // 1.5s - 3.5s: Squat down to 0.65m
            // 3.5s - 5.5s: Stand back up to 0.72m
            if (simTime >= 1.5 && !squat_started)
            {
                joyStick.setPzRef(0.65, 2.0); // squat down to 0.65m over 2 seconds
                squat_started = true;
                std::cout << "[t=" << simTime << "] Squatting down to 0.65m\n";
            }
            else if (simTime >= 3.5 && !stand_up_started)
            {
                joyStick.setPzRef(0.72, 2.0); // stand back up to 0.72m over 2 seconds
                stand_up_started = true;
                std::cout << "[t=" << simTime << "] Standing back up to 0.72m\n";
            }

            robot_wrapper.computeKin();

            joyStick.step();
            gaitScheduler.step(joyStick); // keep state in MotionState::STAND

            kin_wbc.computeWBC_IK(joyStick, footPlanner, robot_wrapper, cp_planning, gaitScheduler);
            robot_wrapper.integrateConfig(stepSize * kin_wbc.out_delta_q);

            simTime += kin_wbc.dt;

            // Visualize on MuJoCo: puppet qpos/qvel from robot_wrapper kinematics
            mj_data->qpos[freeQposAdr + 0] = robot_wrapper.q(0);
            mj_data->qpos[freeQposAdr + 1] = robot_wrapper.q(1);
            mj_data->qpos[freeQposAdr + 2] = robot_wrapper.q(2);
            mj_data->qpos[freeQposAdr + 3] = robot_wrapper.q(6); // w
            mj_data->qpos[freeQposAdr + 4] = robot_wrapper.q(3); // x
            mj_data->qpos[freeQposAdr + 5] = robot_wrapper.q(4); // y
            mj_data->qpos[freeQposAdr + 6] = robot_wrapper.q(5); // z

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

        // Plot base height tracking
        HeightPlot.addPoint("Actual Base Z", simTime, robot_wrapper.pos_base_W(2));
        HeightPlot.addPoint("Target Pz", simTime, joyStick.pz_W);
        HeightPlot.render();

        // Plot foot height (proves feet are strictly pinned to ground)
        FootPlot.addPoint("L Foot Z", simTime, robot_wrapper.pos_L_feet_W(2));
        FootPlot.addPoint("R Foot Z", simTime, robot_wrapper.pos_R_feet_W(2));
        FootPlot.render();

        // Plot CoM
        CoMPlot.addPoint("CoM X", simTime, robot_wrapper.pos_CoM_W(0));
        CoMPlot.addPoint("CoM Y", simTime, robot_wrapper.pos_CoM_W(1));
        CoMPlot.render();

        uiController.updateScene();
    }

    std::cout << "Final base height: " << robot_wrapper.pos_base_W(2) << "\n";
    uiController.Close();
    return 0;
}

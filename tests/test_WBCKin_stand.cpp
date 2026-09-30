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

const std::string URDF_PATH = "models/urdf/v2_biped_robot_12dof.urdf";
const std::string XML_PATH = "models/mjcf/scene_floatingbase_12dof_v2.xml";
const std::string STEP_PLANNING_CF_PATH = "config/step_planning_cf.yaml";
const std::string YAML_QP_WBC_CF_PATH = "config/wbc_config.yaml";

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
    KinWBC kin_wbc(YAML_QP_WBC_CF_PATH);
    JoyStickInterpreter joyStick(kin_wbc.dt);
    MyGaitScheduler gaitScheduler(STEP_PLANNING_CF_PATH, kin_wbc.dt);
    FootPlacement footPlanner(STEP_PLANNING_CF_PATH, robot_wrapper);
    const double dt = 0.001;

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
    RealtimePlot CPPlot(mj_model, 600, 400, "CoM-CapturedPoint-ZMP", 5.0);
    CPPlot.setYLabel("meters");
    CPPlot.setYLimit(-0.15, 0.15); 
    CPPlot.setLineWidth(2.5f);

    RealtimePlot PhasePlot(mj_model, 600, 400, "Phase", 5.0);
    PhasePlot.setYLabel("Phase");
    PhasePlot.setYLimit(0, 1); 
    PhasePlot.setLineWidth(2.5f);


    // Initial configuration: load directly at the bent-knee stand pose
    // (computeInitial_Stand()) instead of the straight-leg home pose --
    // computeWBC_IK()'s null-space IK is unstable starting from straight
    // legs (near-singular leg Jacobian), so just start bent, no ramp.
    robot_wrapper.q.segment(7, robot_wrapper.model_na_).setZero();
    robot_wrapper.q(2) = 0.0;
    robot_wrapper.computeKin();
    const double straight_leg_length = -robot_wrapper.pos_L_feet_W(2); // max reach, straight legs

    // computeInitial_Stand(h) solves a closed-form leg IK for a base-to-
    // foot distance of exactly h -- commanding straight_leg_length (the
    // max reach) back into it just returns ~straight legs again, since
    // that's the only way to reach that far. Bending the knees needs a
    // meaningfully SHORTER target distance instead.
    const double standLegLength = 0.78; // 
    robot_wrapper.q(2) = standLegLength;
    VectorXd init_joint_config = robot_wrapper.computeInitial_Stand(standLegLength);
    robot_wrapper.q.segment(7, robot_wrapper.model_na_) = init_joint_config;
    std::cout<<"init joint position: "<<init_joint_config.transpose()<<std::endl;
    robot_wrapper.computeKin();

    footPlanner.legLength = robot_wrapper.pos_base_W(2);
    std::cout << "Straight-leg reach: " << straight_leg_length
               << "  Initial bent-knee base height: " << robot_wrapper.pos_base_W(2) << "\n";

    // Constructed here (after settling into the bent-knee stand pose above)
    // so zc is the robot's REAL measured CoM height at that pose instead of
    // a guessed constant -- w=sqrt(g/zc) (CP_Planning.h) is the LIPM natural
    // frequency, and using the wrong height desyncs the capture-point
    // dynamics from the actual pendulum this robot's CoM behaves like.
    const double zc = robot_wrapper.pos_CoM_W(2);
    std::cout << "Bent-knee stand CoM height (used as CP_Planning zc): " << zc << "\n";
    CP_Planning cp_planning(dt, zc, footPlanner.hip_width);

    const double stepSize = 1.0;

    // Initialize joystick and planners strictly in STAND mode.
    // pz_W (setIniPos's z-arg / setPzRef's target) must be the robot's CoM
    // height, not base height -- kin_task_stand's height task (task_CoMZc)
    // compares this against rb_wrapper.pos_CoM_W(2), and for this robot's
    // bent-knee pose those differ by ~0.28m (base ~0.78 vs CoM ~0.50). Using
    // base height here doesn't blow up in this kinematic-only test (the
    // gain-free delta_q solve's null-space authority over CoMZc happens to
    // stay small), but it's the same latent mismatch that DID blow up
    // DynWBC's QP in test_DynWBC.cpp -- fixed there and here together so
    // both stay consistent with what the task actually measures.
    joyStick.setIniPos(robot_wrapper.pos_base_W(0), robot_wrapper.pos_base_W(1), robot_wrapper.pos_CoM_W(2), 0.0);
    joyStick.setMotionState(MotionState::STAND);
    joyStick.setVxDesLPara(0.0, 0.1); // zero horizontal velocities for standing
    joyStick.setVyDesLPara(0.0, 0.1);
    joyStick.setWzDesLPara(0.0, 0.1);
    joyStick.setPzRef(robot_wrapper.pos_CoM_W(2), 0.01); // hold the bent-knee CoM height

    cp_planning.xc_ = robot_wrapper.pos_CoM_W(0);
    cp_planning.yc_ = robot_wrapper.pos_CoM_W(1);
    cp_planning.d_xc_ = 0.0;
    cp_planning.d_yc_ = 0.0;

    double simTime = 0.0;
    const double warmUpStartTime = 2.0; // let the STAND-mode IK settle at the bent-knee pose first
    bool warmUpStarted = false;
    double lastStatusPrintTime = -1.0;

    while (!glfwWindowShouldClose(uiController.window))
    {
        double frameStart = simTime;
        while (uiController.runSim && (simTime - frameStart) < 1.0 / 60.0) // press "1" to pause/resume, "2" to step
        {

            if (simTime >= warmUpStartTime && !warmUpStarted)
            {
                joyStick.setMotionState(MotionState::WARM_UP);
                gaitScheduler.start(joyStick);
                footPlanner.inPlaceOnly = true; // sway/lift in place, no forward stepping
                // Re-target pz_W: kin_task_init_walk (WARM_UP's task list)
                // uses task_base_height, which compares against BASE height,
                // not task_CoMZc's CoM height -- the two differ by ~0.28m
                // for this robot's bent-knee pose, so pz_W must be
                // re-pointed at the convention the NEW active task list
                // actually measures against.
                joyStick.setPzRef(robot_wrapper.pos_base_W(2), 0.01);
                warmUpStarted = true;
                std::cout << "[t=" << simTime << "] Starting warm-up CoM sway + foot lift\n";
            }

            //--------------------------------------------------------------------
            // Main control task
            //--------------------------------------------------------------------

            if (warmUpStarted)
            {
                robot_wrapper.computeKin();
                joyStick.step();
                gaitScheduler.step(joyStick);
                cp_planning.planWarmingUp(gaitScheduler);
                footPlanner.StepSwingPlanning(robot_wrapper, joyStick, cp_planning);
                
                kin_wbc.computeWBC_IK(joyStick, footPlanner, robot_wrapper, cp_planning, gaitScheduler);
                robot_wrapper.integrateConfig(stepSize * kin_wbc.out_delta_q);
            }
            simTime += kin_wbc.dt;

            //--------------------------------------------------------------------
            // Visualize on MuJoCo: puppet qpos/qvel from robot_wrapper kinematics
            //--------------------------------------------------------------------

            mj_data->qpos[freeQposAdr + 0] = robot_wrapper.q(0);
            mj_data->qpos[freeQposAdr + 1] = robot_wrapper.q(1);
            mj_data->qpos[freeQposAdr + 2] = robot_wrapper.q(2) + 0.08;
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
        CPPlot.addPoint("Capture Point - Y", simTime, cp_planning.cxi_y_);
        CPPlot.addPoint("ZMP - Y", simTime, cp_planning.py_d_);
        CPPlot.addPoint("Desired COM - Y", simTime, cp_planning.yc_);
        CPPlot.render();

        PhasePlot.addPoint("Swing Foot Phase", simTime, cp_planning.phi_swing);
        PhasePlot.addPoint("CoM Phase", simTime, gaitScheduler.phi);
        PhasePlot.render();

        // CoM marker in the 3D scene -- offset by the same +0.08 in Z the
        // puppeted base uses above (mj_data->qpos[...+2] = q(2)+0.08), so
        // the marker lines up with the visualized (shifted) robot instead
        // of floating below it.
        static const float colorCoM[4] = {1.0f, 0.85f, 0.1f, 0.9f};
        Eigen::Vector3d comMarkerPos_W = robot_wrapper.pos_CoM_W + Eigen::Vector3d(0.0, 0.0, 0.08);
        uiController.addSphere(comMarkerPos_W, 0.025, colorCoM);

        // ZMP marker + a line connecting it to the CoM -- same LIPM
        // "pendulum" picture the CPPlot's Capture Point/ZMP/CoM traces show,
        // just in 3D. ZMP sits at ground level, offset by the same +0.08 as
        // the CoM marker above so both stay in the same visual frame as the
        // puppeted robot.
        static const float colorZMP[4] = {0.9f, 0.2f, 0.2f, 0.9f};
        Eigen::Vector3d zmpMarkerPos_W(cp_planning.px_d_, cp_planning.py_d_, 0.08);
        uiController.addSphere(zmpMarkerPos_W, 0.02, colorZMP);
        uiController.addLine(zmpMarkerPos_W, comMarkerPos_W, colorZMP, 2.0);

        uiController.updateScene();
    }

    std::cout << "Final base height: " << robot_wrapper.pos_base_W(2) << "\n";
    uiController.Close();
    return 0;
}

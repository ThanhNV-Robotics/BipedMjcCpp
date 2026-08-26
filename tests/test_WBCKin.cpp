#include <iostream>
#include <iomanip>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include "GLFW_callbacks.h"
#include "robot_wrapper.h"
#include "KinWBC.h"
#include "joystick_interpreter.h"
#include "foot_placement.h"

const std::string URDF_PATH = "models/urdf/biped_robot_12dof.urdf";
const std::string XML_PATH = "models/mjcf/scene_floatingbase_12dof.xml";

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
    FootPlacement footPlanner; // default-constructed: computeWBC_IK's stand
                                // tasks don't currently read from it at all

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

    // RobotWrapper starts at pin::neutral(), i.e. every joint angle is 0 -- a
    // fully straight leg, a classic leg-Jacobian singularity (vertical
    // foot/torso motion vanishes w.r.t. knee angle at full extension). Seed a
    // proper bent-knee stand posture instead, via the same IK
    // (computeInK_Leg) already used/proven elsewhere in this codebase for the
    // "ramped stand" reference, rather than starting exactly on top of the
    // singularity and relying only on pseudoInv_right_weighted()'s damping
    // (useful_math.cpp) to survive it.
    // Note: this only seeds the joint angles -- the floating base's world
    // position (q(0:3)) is independent of joint angles in this model, so
    // pos_base_W's starting value is still 0 either way, unaffected by this.
    const double standLegLength = 0.75;
    robot_wrapper.q.segment(7, robot_wrapper.model_na_) = robot_wrapper.computeInitial_Stand(standLegLength);

    robot_wrapper.computeKin();
    const double initial_height = robot_wrapper.pos_base_W(2) ;
    std::cout << "Initial base height: " << initial_height << "\n";

    const int numSteps = 3000;
    const double lowerRate = 0.05; // m/s, commanded torso-lowering speed
    // Newton-step damping for the integration below -- out_delta_q is sized
    // to eliminate the *entire* current task error in one step (same
    // formulation as PriorityTasks::computeAll()), which is fine near
    // convergence but can overshoot/diverge far from it or near a
    // near-singular Jacobian, so only take a fraction of it per iteration
    // (mirrors computeInK_Leg's DT=0.7 damping on its own Newton step).
    const double stepSize = 0.2;

    joyStick.vz_W = -lowerRate;
    joyStick.pz_W = initial_height;

    int i = 0;
    while (!glfwWindowShouldClose(uiController.window) && i < numSteps)
    {
        if (uiController.runSim) // press "1" to pause/resume, "2" to step
        {
            robot_wrapper.computeKin();
            kin_wbc.computeWBC_IK(joyStick, footPlanner, robot_wrapper);

            if (i % 100 == 0)
            {
                std::cout << std::fixed << std::setprecision(5)
                          << "step " << std::setw(5) << i
                          << " | base height: " << robot_wrapper.pos_base_W(2)
                          << " | target pz_W: " << joyStick.pz_W
                          << " | base_height errX: " << kin_wbc.task_base_height.errX(0)
                          << " | out_delta_q norm: " << kin_wbc.out_delta_q.norm()
                          << " | out_dq norm: " << kin_wbc.out_dq.norm()
                          << std::endl;
            }

            if (!kin_wbc.out_delta_q.allFinite() || !kin_wbc.out_dq.allFinite())
            {
                std::cerr << "out_delta_q/out_dq went non-finite at step " << i
                          << " -- likely the knee-straight singularity noted above.\n";
                for (size_t t = 0; t < kin_wbc.kin_task_stand.size(); t++)
                {
                    Task *task = kin_wbc.kin_task_stand[t];
                    std::cerr << "  task[" << t << "] " << task->taskName
                              << " delta_q finite=" << task->delta_q.allFinite()
                              << " dq finite=" << task->dq.allFinite()
                              << std::endl;
                }
                break;
            }

            robot_wrapper.integrateConfig(stepSize * kin_wbc.out_delta_q);

            // ramp the commanded height down, same "velocity * dt" convention
            // task_base_height.deltaX_des already uses internally
            joyStick.pz_W += joyStick.vz_W * kin_wbc.dt;

            // puppet MuJoCo's qpos/qvel from robot_wrapper's kinematic state
            // and re-run FK -- mj_forward, never mj_step, so nothing here is
            // ever physically simulated, only kinematically displayed
            mj_data->qpos[freeQposAdr + 0] = robot_wrapper.q(0);
            mj_data->qpos[freeQposAdr + 1] = robot_wrapper.q(1);
            mj_data->qpos[freeQposAdr + 2] = robot_wrapper.q(2) + 0.8;
            // MuJoCo's free-joint quaternion order is (w,x,y,z); Pinocchio's
            // q.segment<4>(3) coeffs order is (x,y,z,w)
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
            i++;
        }

        uiController.updateScene(); // once per rendered frame, not per IK step
    }

    std::cout << "Final base height: " << robot_wrapper.pos_base_W(2) << "\n";
    uiController.Close();
    return 0;
}

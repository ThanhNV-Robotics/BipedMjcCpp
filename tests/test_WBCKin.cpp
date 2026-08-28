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
    MyGaitScheduler gaitScheduler(0.5, kin_wbc.dt);
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


    RealtimePlot JoyStickPlot(mj_model, 500, 400, "Joystick Command", 5.0);
    JoyStickPlot.setYLabel("meters");
    JoyStickPlot.setYLimit(-0.3, 0.1); 
    JoyStickPlot.setLineWidth(2.5f);

    // Initially starting at a bended configuration to avoid singularity

    const double standLegLength = 0.75;
    robot_wrapper.q(2) = standLegLength; // init initial base height
    robot_wrapper.q.segment(7, robot_wrapper.model_na_) = robot_wrapper.computeInitial_Stand(standLegLength);

    robot_wrapper.computeKin();
    const double initial_height = robot_wrapper.pos_base_W(2) ;
    std::cout << "Initial base height: " << initial_height << "\n";

    const int numSteps = 3000;
    const double lowerRate = 0.05; // m/s, commanded torso-lowering speed
    const double stepSize = 1;

    
    joyStick.setIniPos(0,0,robot_wrapper.q(2),0);
    joyStick.setPzRef(0.67, 2); // set target reference base height
    joyStick.setVxDesLPara(0.5, 2);

    int i = 0;
    double simTime = 0.0;
    const double startWarmUpTime = 2.5;

    while (!glfwWindowShouldClose(uiController.window))
    {
        double frameStart = simTime;
        while (uiController.runSim && (simTime - frameStart) < 1.0 / 60.0 ) // press "1" to pause/resume, "2" to step
        {
            robot_wrapper.computeKin();
            kin_wbc.computeWBC_IK(joyStick, footPlanner, robot_wrapper);

            if (i % 500 == 0)
            {
                std::cout << "CoM position: " << robot_wrapper.pos_CoM_W[0] << ", "
                          << robot_wrapper.pos_CoM_W[1] << ", "
                          << robot_wrapper.pos_CoM_W[2] << std::endl;
            }

            robot_wrapper.integrateConfig(stepSize * kin_wbc.out_delta_q);

            // joyStick.pz_W += joyStick.vz_W * kin_wbc.dt;
            // joyStick.pz_W = standLegLength - 0.05 * std::abs(std::sin(2*3.1415*0.5*simTime));

            joyStick.step();
            simTime += kin_wbc.dt;

            if (simTime >= startWarmUpTime)
            {
                joyStick.setMotionState(MotionState::WALK);
               
                gaitScheduler.start();

            }
            // Visualize on mujoco

            // puppet MuJoCo's qpos/qvel from robot_wrapper's kinematic state
            // and re-run FK -- mj_forward, never mj_step, so nothing here is
            // ever physically simulated, only kinematically displayed
            mj_data->qpos[freeQposAdr + 0] = robot_wrapper.q(0);
            mj_data->qpos[freeQposAdr + 1] = robot_wrapper.q(1);
            mj_data->qpos[freeQposAdr + 2] = robot_wrapper.q(2) + 0.05;
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


        // JoyStickPlot.addPoint("base height", simTime, robot_wrapper.pos_base_W(2));
        JoyStickPlot.addPoint("target pz_W", simTime, joyStick.pz_W);
        JoyStickPlot.addPoint("vx_ref", simTime, joyStick.vx_W);
        JoyStickPlot.addPoint("vy_ref", simTime, joyStick.vy_W);
        JoyStickPlot.render(); // makes JoyStickPlot's own context current, draws, swaps buffers

        uiController.updateScene();
    }

    std::cout << "Final base height: " << robot_wrapper.pos_base_W(2) << "\n";
    uiController.Close();
    return 0;
}

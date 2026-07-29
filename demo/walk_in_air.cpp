// Sanity-check test: exercise FootPlacement's swing-foot trajectory generator
// and computeInK_Leg() on the fixed-base 12-DOF model, with the torso rigidly
// fixed in the air. No ground contact, no whole-body/MPC balance control --
// legState alternates on a fixed timer instead of GaitScheduler's
// contact-force-based switching (which never trips with nothing to touch).
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <algorithm>
#include <cstdio>
#include <iostream>
#include <vector>
#include "useful_math.h"
#include "GLFW_callbacks.h"
#include "MJ_interface.h"
#include "PVT_ctrl.h"
#include "pino_kin_dyn.h"
#include "foot_placement.h"

const std::string MODEL_DIR = "models/mjcf";

int main(int argc, const char **argv)
{
    std::cout << "Program Starts, Loading Mujoco xml model\n";

    const std::string model_path = MODEL_DIR + "/scene_fixedbase_12dof.xml";
    char loadError[1024] = "";
    mjModel *mj_model = mj_loadXML(model_path.c_str(), nullptr, loadError, sizeof(loadError));
    if (!mj_model)
    {
        std::fprintf(stderr, "failed to load %s: %s\n", model_path.c_str(), loadError);
        return 1;
    }
    mjData *mj_data = mj_makeData(mj_model);
    std::cout << "Compile mujoco xml done\n";

    const std::string joint_ctrl_config_path = "config/12dof_joint_config.json";
    UIctr uiController(mj_model, mj_data);
    MJ_Interface mj_interface(mj_model, mj_data, joint_ctrl_config_path.c_str());

    const std::string urdf_path = "models/urdf/biped_robot_12dof.urdf";
    Pin_KinDyn kinDynSolver(urdf_path);
    DataBus RobotState(kinDynSolver.model_nv);
    PVT_Ctr pvtCtr(mj_model->opt.timestep, joint_ctrl_config_path.c_str());
    FootPlacement footPlacement;
    std::cout << "Init done\n";

    // kinDynSolver.ikJointNames is model_biped_fixed's own joint order (left
    // leg then right leg), read directly from the model -- NOT the same order
    // MJ_Interface/PVT_Ctr use (alphabetical, from JSON keys). Use
    // kinDynSolver.mapJointVecToOrder() to remap between them, never a
    // positional copy.
    const std::vector<std::string> &ikJointNames = kinDynSolver.ikJointNames;

    // Precompute each joint's qpos/qvel address once (avoids a string lookup
    // every tick), used to read the robot's current joint state directly in
    // Pin_KinDyn's own order.
    std::vector<int> ikQposAdr(ikJointNames.size()), ikDofAdr(ikJointNames.size());
    for (size_t i = 0; i < ikJointNames.size(); i++)
    {
        int jid = mj_name2id(mj_model, mjOBJ_JOINT, ikJointNames[i].c_str());
        if (jid == -1)
        {
            std::fprintf(stderr, "joint %s not found in %s\n", ikJointNames[i].c_str(), model_path.c_str());
            return 1;
        }
        ikQposAdr[i] = mj_model->jnt_qposadr[jid];
        ikDofAdr[i] = mj_model->jnt_dofadr[jid];
    }

    // The torso is rigidly welded to the world in this model (no free joint),
    // so its pose is constant. Pin_KinDyn's internal model always carries a
    // floating base regardless of the URDF, so this fixed pose has to be fed
    // to it every tick via DataBus::updateQ() -- there's no real sensor for it.
    const double standHeight = 0.95; // matches biped_robot_fixedbase_12dof.xml's torso pos
    const Eigen::Vector3d torsoOrigin(0.0, 0.0, standHeight);
    RobotState.basePos[0] = 0; RobotState.basePos[1] = 0; RobotState.basePos[2] = standHeight;
    RobotState.rpy[0] = 0; RobotState.rpy[1] = 0; RobotState.rpy[2] = 0;
    RobotState.baseLinVel[0] = RobotState.baseLinVel[1] = RobotState.baseLinVel[2] = 0;
    RobotState.baseAngVel[0] = RobotState.baseAngVel[1] = RobotState.baseAngVel[2] = 0;

    const double stand_legLength = 0.72;
    const double width_hips = 0.334;
    RobotState.width_hips = width_hips;

    const Eigen::Matrix3d identityRot = Eigen::Matrix3d::Identity();
    Eigen::Vector3d fe_l_pos_L_des = {0.0, width_hips / 2, -stand_legLength};
    Eigen::Vector3d fe_r_pos_L_des = {0.0, -width_hips / 2, -stand_legLength};

    // Solve the standing pose once, to know where each foot starts.
    auto resLeg = kinDynSolver.computeInK_Leg(identityRot, fe_l_pos_L_des, identityRot, fe_r_pos_L_des);
    std::printf("Standing IK status=%d itr=%d err=%.6f\n", resLeg.status, resLeg.itr, resLeg.err.norm());

    RobotState.motors_pos_des = kinDynSolver.mapJointVecToOrder(resLeg.jointPosRes, pvtCtr.getMotorNames());
    RobotState.motors_vel_des.assign(pvtCtr.getMotorNames().size(), 0.0);
    RobotState.motors_tor_des.assign(pvtCtr.getMotorNames().size(), 0.0);

    // Gait parameters. legState alternates on a fixed timer here instead of
    // via GaitScheduler's contact-force-based switching (see file header).
    const double tSwing = 0.8;
    footPlacement.tSwing = tSwing;
    footPlacement.stepHeight = 0.08;
    footPlacement.legLength = stand_legLength;
    footPlacement.kp_vx = 0;
    footPlacement.kp_vy = 0;
    footPlacement.kp_wz = 0;
    RobotState.desV_W.setZero();
    RobotState.desWz_W = 0;
    RobotState.js_vel_des.setZero();
    RobotState.js_omega_des.setZero();

    DataBus::LegState legState = DataBus::LSt; // start: left stance / right swing
    Eigen::Vector3d swingStartPos_W, stancePos_W;
    bool haveAnchors = false;
    const double standTime = 2.0; // hold the standing pose before starting to walk

    uiController.iniGLFW();
    uiController.disableTracking();
    uiController.createWindow("Walk in air", false);

    double simstart = mj_data->time;
    double simTime = mj_data->time;

    while (!glfwWindowShouldClose(uiController.window))
    {
        simstart = mj_data->time;
        while (mj_data->time - simstart < 1.0 / 60.0)
        {
            mj_step(mj_model, mj_data);
            simTime = mj_data->time;

            mj_interface.updateSensorValues();
            mj_interface.dataBusWrite(RobotState); // motors_pos_cur/vel_cur, MJ_Interface's own order

            if (simTime > standTime)
            {
                // Read current joint state directly in Pin_KinDyn's order
                // (bypassing MJ_Interface/PVT_Ctr's alphabetical order).
                std::vector<double> qPin(ikJointNames.size()), dqPin(ikJointNames.size());
                for (size_t i = 0; i < ikJointNames.size(); i++)
                {
                    qPin[i] = mj_data->qpos[ikQposAdr[i]];
                    dqPin[i] = mj_data->qvel[ikDofAdr[i]];
                }
                RobotState.motors_pos_cur = qPin;
                RobotState.motors_vel_cur = dqPin;
                RobotState.updateQ(); // builds q/dq for Pin_KinDyn from the fields above

                kinDynSolver.dataBusRead(RobotState);
                kinDynSolver.computeJ_dJ(); // updates fe_l_pos_W, fe_r_pos_W, hip_l_pos_W, hip_r_pos_W, ...
                kinDynSolver.dataBusWrite(RobotState);

                // Restore MJ_Interface/PVT_Ctr's own order for the PD controller.
                mj_interface.dataBusWrite(RobotState);

                // Timer-driven phase and leg state.
                double tWalk = simTime - standTime;
                double phi = std::fmod(tWalk, tSwing) / tSwing;
                int stepIdx = static_cast<int>(tWalk / tSwing);
                DataBus::LegState newLegState = (stepIdx % 2 == 0) ? DataBus::LSt : DataBus::RSt;

                if (!haveAnchors || newLegState != legState)
                {
                    legState = newLegState;
                    swingStartPos_W = (legState == DataBus::LSt) ? RobotState.fe_r_pos_W : RobotState.fe_l_pos_W;
                    stancePos_W = (legState == DataBus::LSt) ? RobotState.fe_l_pos_W : RobotState.fe_r_pos_W;
                    haveAnchors = true;
                }

                RobotState.phi = phi;
                RobotState.tSwing = tSwing;
                RobotState.legState = legState;
                RobotState.swingStartPos_W = swingStartPos_W;
                RobotState.posHip_W = (legState == DataBus::LSt) ? RobotState.hip_r_pos_W : RobotState.hip_l_pos_W;
                RobotState.posST_W = (legState == DataBus::LSt) ? RobotState.fe_l_pos_W : RobotState.fe_r_pos_W;
                RobotState.theta0 = (legState == DataBus::LSt) ? -3.14159265358979 * 0.5 : 3.14159265358979 * 0.5;

                footPlacement.dataBusRead(RobotState);
                footPlacement.getSwingPos();
                footPlacement.dataBusWrite(RobotState);

                // Convert the swing target (world frame) into torso-relative
                // frame for computeInK_Leg (torso pose here is fixed & known).
                Eigen::Vector3d swingTargetRel = RobotState.swing_fe_pos_des_W - torsoOrigin;
                Eigen::Vector3d stanceTargetRel = stancePos_W - torsoOrigin;

                Eigen::Vector3d newLeftPos = (legState == DataBus::LSt) ? stanceTargetRel : swingTargetRel;
                Eigen::Vector3d newRightPos = (legState == DataBus::LSt) ? swingTargetRel : stanceTargetRel;

                auto resLegWalk = kinDynSolver.computeInK_Leg(identityRot, newLeftPos, identityRot, newRightPos);
                RobotState.motors_pos_des = kinDynSolver.mapJointVecToOrder(resLegWalk.jointPosRes, pvtCtr.getMotorNames());
            }

            pvtCtr.dataBusRead(RobotState);
            pvtCtr.calMotorsPVT(100.0 / 1000.0 / 180.0 * 3.1415); // velocity-limited tracking
            pvtCtr.dataBusWrite(RobotState);

            mj_interface.setMotorsTorque(RobotState.motors_tor_out);
        }
        uiController.updateScene();
    }

    uiController.Close();
    mj_deleteData(mj_data);
    mj_deleteModel(mj_model);
    return 0;
}

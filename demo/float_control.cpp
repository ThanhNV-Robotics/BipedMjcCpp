#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <algorithm>
#include <cstdio>
#include <iostream>
#include "useful_math.h"
#include "GLFW_callbacks.h"
#include "MJ_interface.h"
#include "PVT_ctrl.h"
#include "pino_kin_dyn.h"
#include "data_logger.h"

//************************
// main function
int main(int argc, const char **argv)
{
    std::cout << "Program Starts, Loading Mujoco xml model\n";
    const std::string MODEL_DIR = "models/mjcf"; // path to mujoco xml model

    //-------------------------------------------------------------------
    // Compile mujoco xml model
    //-------------------------------------------------------------------
    const std::string model_path = MODEL_DIR + "/scene_fixedbase_12dof.xml";
    // const std::string model_path = (argc > 1) ? argv[1] : model_path; // if input model path in the arg then use that path
    std::cout << "Input model path: " + model_path + "\n";
    char loadError[1024] = ""; // character array, size 1024
    // load/compile xml model
    // model_path.c_str() return a read-only pointer to const std::string model_path
    mjModel *mj_model = mj_loadXML(model_path.c_str(), nullptr, loadError, sizeof(loadError)); // pointer to mjModel struct
    if (!mj_model)
    {
        std::fprintf(stderr, "failed to load %s: %s\n", model_path.c_str(), loadError);
        return 1;
    }
    mjData *mj_data = mj_makeData(mj_model); // pointer to mjData struct
    std::cout << "Compile mujoco xml done\n";

    // ini classes
    const std::string joint_ctrl_config_path = "config/12dof_joint_config.json";
    UIctr uiController(mj_model, mj_data); // UI control for Mujoco
    MJ_Interface mj_interface(mj_model, mj_data, joint_ctrl_config_path.c_str());
    std::cout << "Init MJ_Interface done\n";

    const std::string default_urdf = "models/urdf/biped_robot_12dof.urdf";
    const std::string urdf_path = (argc > 1) ? argv[1] : default_urdf;
    Pin_KinDyn kinDynSolver(urdf_path.c_str()); // kinematics and dynamics solver
    std::cout << "Init Pin_KinDyn done\n";

    DataBus RobotState(kinDynSolver.model_nv);                              // data bus
    PVT_Ctr pvtCtr(mj_model->opt.timestep, joint_ctrl_config_path.c_str()); // PVT joint control
    DataLogger logger("record/float_control.log");                          // data logger

    std::cout << "Init done\n";

    // variables ini
    double stand_legLength = 0.72; // // desired baselink height
    // double foot_height =0.07; // distance between the foot ankel joint and the bottom
    // double  xv_des = 0.7;  // desired velocity in x direction

    RobotState.width_hips = 0.334;
    // //mju_copy(mj_data->qpos, mj_model->key_qpos, mj_model->nq*1); // set ini pos in Mujoco
    int model_nv = kinDynSolver.model_nv;
    std::printf("Model nv: %d \n", model_nv); // 18

    // // ini position and posture for foot-end
    std::vector<double> motors_pos_des(model_nv - 6, 0); //=12, -6 to exclude the floating base dof in pinocchio model
    std::vector<double> motors_pos_cur(model_nv - 6, 0);
    std::vector<double> motors_vel_des(model_nv - 6, 0);
    std::vector<double> motors_vel_cur(model_nv - 6, 0);
    std::vector<double> motors_tau_des(model_nv - 6, 0);
    std::vector<double> motors_tau_cur(model_nv - 6, 0);
    Eigen::Vector3d fe_l_pos_L_des = {0.0, RobotState.width_hips / 2, -stand_legLength};  // desired left feet pos
    Eigen::Vector3d fe_r_pos_L_des = {0.0, -RobotState.width_hips / 2, -stand_legLength}; // desired right feet pos

    Eigen::Vector3d fe_l_eul_L_des = {0.0, 0.0, 0.0};
    Eigen::Vector3d fe_r_eul_L_des = {0.0, 0.0, 0.0};
    Eigen::Matrix3d fe_l_rot_des = eul2Rot(fe_l_eul_L_des(0), fe_l_eul_L_des(1), fe_l_eul_L_des(2));
    Eigen::Matrix3d fe_r_rot_des = eul2Rot(fe_r_eul_L_des(0), fe_r_eul_L_des(1), fe_r_eul_L_des(2));

    std::cout << "Init variable done\n";
    // solving inverse kinematics
    auto resLeg = kinDynSolver.computeInK_Leg(fe_l_rot_des, fe_l_pos_L_des, fe_r_rot_des, fe_r_pos_L_des);
    std::cout << "IK solved \n";

    // print the computed joint angles: model_biped_fixed's joint order is
    // left leg (0-5) then right leg (6-11), matching biped_robot_12dof.urdf's
    // own joint declaration order.
    const std::vector<std::string> ikJointNames = {
        "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
        "left_knee_pitch_joint", "left_ankle_roll_joint", "left_ankle_pitch_joint",
        "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
        "right_knee_pitch_joint", "right_ankle_roll_joint", "right_ankle_pitch_joint"};
    std::printf("IK status=%d itr=%d err_norm=%.6f\n", resLeg.status, resLeg.itr, resLeg.err.norm());
    for (int i = 0; i < resLeg.jointPosRes.size(); i++)
    {
        std::printf("  %-25s % .5f rad (% .3f deg)\n", ikJointNames[i].c_str(),
                    resLeg.jointPosRes(i), resLeg.jointPosRes(i) * 180.0 / 3.14159265358979);
    }

    // // register variable name for data logger
    // logger.addIterm("simTime", 1);
    // logger.addIterm("motors_pos_cur",model_nv-6);
    // logger.addIterm("motors_pos_des",model_nv-6);
    // logger.addIterm("motors_tau_cur",model_nv-6);
    // logger.addIterm("motors_vel_des",model_nv-6);
    // logger.addIterm("motors_vel_cur",model_nv-6);
    // logger.finishItermAdding();

    // Map the IK result onto PVT_Ctr's own joint order by name. ikJointNames
    // (Pin_KinDyn) is hardcoded left-leg-then-right-leg; pvtCtr's order comes
    // from jsoncpp's getMemberNames() on its JSON config, which sorts
    // alphabetically -- these are NOT the same order, so a positional copy
    // would silently send each angle to the wrong joint.
    const std::vector<std::string> &pvtJointNames = pvtCtr.getMotorNames();
    std::vector<double> motors_pos_des_mapped(pvtJointNames.size(), 0.0);
    for (size_t pvtIdx = 0; pvtIdx < pvtJointNames.size(); pvtIdx++)
    {
        auto it = std::find(ikJointNames.begin(), ikJointNames.end(), pvtJointNames[pvtIdx]);
        if (it == ikJointNames.end())
        {
            std::fprintf(stderr, "joint %s (from PVT config) not found in IK joint list\n", pvtJointNames[pvtIdx].c_str());
            continue;
        }
        int ikIdx = std::distance(ikJointNames.begin(), it);
        motors_pos_des_mapped[pvtIdx] = resLeg.jointPosRes(ikIdx);
    }

    /// ----------------- sim Loop ---------------
    mjtNum simstart = mj_data->time;
    double simTime = mj_data->time;

    // Hold at the IK-solved configuration: constant position target, zero
    // feedforward velocity/torque.
    RobotState.motors_pos_des = motors_pos_des_mapped;
    RobotState.motors_vel_des = motors_vel_des;
    RobotState.motors_tor_des = motors_tau_des;

    // init UI: GLFW
    uiController.iniGLFW();
    uiController.disableTracking(); // enable viewpoint tracking of the body 1 of the robot
    uiController.createWindow("Demo", false);

    while (!glfwWindowShouldClose(uiController.window))
    {
        // advance interactive simulation for 1/60 sec
        simstart = mj_data->time;
        while (mj_data->time - simstart < 1.0 / 60.0)
        {
            mj_step(mj_model, mj_data);
            simTime = mj_data->time;

            mj_interface.updateSensorValues();
            mj_interface.dataBusWrite(RobotState);
            if (simTime >= 3)
            {
                pvtCtr.dataBusRead(RobotState);
                if (simTime <= 6)
                    pvtCtr.calMotorsPVT(100.0 / 1000.0 / 180.0 * 3.1415); // limit velocity while approaching the target
                else
                    pvtCtr.calMotorsPVT();
                pvtCtr.dataBusWrite(RobotState);
            }

            mj_interface.setMotorsTorque(RobotState.motors_tor_out);
        }

        uiController.updateScene();
    }

    //    // free visualization storage
    uiController.Close();

    // free MuJoCo model and data, deactivate
    mj_deleteData(mj_data);
    mj_deleteModel(mj_model);

    return 0;
}
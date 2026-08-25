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

using namespace std;

const string URDF_PATH = "models/urdf/biped_robot_12dof.urdf";
const string XML_PATH = "models/mjcf/scene_floatingbase_12dof.xml";
const string FREE_JOINT_NAME = "floating_base_joint";
char loadError[1024] = ""; // character array, size 1024

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
    RobotWrapper robot_wrapper = RobotWrapper(URDF_PATH);
    RobotSensor rb_sensors(mj_model->na);

    const std::string joint_ctrl_config_path = "config/12dof_joint_config.yaml";
    UIctr uiController(mj_model, mj_data);   // UI control for Mujoco
    MJ_Interface mj_interface(mj_model, mj_data, joint_ctrl_config_path.c_str()); // data interface for Mujoco
    // print out xml model info
    std::printf("MuJoCo xml model info: \n");
    mj_interface.printInfo();

    PVT_Ctr pvtCtr(mj_model->opt.timestep, joint_ctrl_config_path.c_str()); // PVT joint control
    StateEstimator state_estimator(mj_model->opt.timestep, true);

    const double init_base_height = 0.75;
    VectorXd qIniDes = robot_wrapper.computeInitial_Stand(init_base_height);
    printf("Init standing joint config: \n");
    std::cout << qIniDes << std::endl;

    /// ----------------- sim Loop ---------------
    mjtNum simstart = mj_data->time;
    double simTime = mj_data->time;

    // for ramping the standing
    const double rampDuration = 2.0;
    double rampFrac = std::min(simTime / rampDuration, 1.0);
    Eigen::VectorXd rampedJointPos = rampFrac * qIniDes;

    // init UI: GLFW
    uiController.iniGLFW();
    uiController.disableTracking(); // enable viewpoint tracking of the body 1 of the robot
    uiController.createWindow("Demo", false);

    while (!glfwWindowShouldClose(uiController.window))
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        simstart = mj_data->time;
        while (mj_data->time - simstart < 1.0 / 60.0 && uiController.runSim) // press "1" to pause and resume, "2" to step the simulation
        {
            mj_step(mj_model, mj_data);
            uiController.applyPerturbation();
            simTime = mj_data->time;

            mj_interface.updateSensorValues(rb_sensors); // propagate sensor values to rb_sensors

            // reads rb_sensors + robot_wrapper (for foot FK), runs the KF, and
            // writes the estimated base pose/velocity back into
            // robot_wrapper.q/dq via updateRobotState()
            state_estimator.update(rb_sensors, robot_wrapper);

            rampFrac = std::min(simTime / rampDuration, 1.0);
            rampedJointPos = rampFrac * qIniDes;

            pvtCtr.getFeedbackMotorState(robot_wrapper); // read back the estimated joint pos/vel
            pvtCtr.calMotorsPVT(rampedJointPos, VectorXd::Zero(robot_wrapper.model_na_),
                                VectorXd::Zero(robot_wrapper.model_na_)); // PD impedance stand control

            mj_interface.setMotorsTorque(pvtCtr.motor_tor_out_motor); // set joint torque to mujoco
        }

        uiController.updateScene();
    }

    // free visualization storage
    uiController.Close();
    return 0;
}

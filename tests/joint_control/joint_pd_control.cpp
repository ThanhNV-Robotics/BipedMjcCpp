// Use pinocchio to calculate rigid body dynamics
#include <mujoco/mujoco.h>
#include <iostream>
#include <string>
#include "GLFW_callbacks.h"

//pinocchio
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>


const std::string MODEL_DIR = "models/mjcf"; // path to mujoco xml model

int main (int argc, char** argv)
{
    std::cout<< "Program Starts, Loading Mujoco xml model\n";

    //-------------------------------------------------------------------
    // Compile mujoco xml model
    //-------------------------------------------------------------------

    const std::string model_path = MODEL_DIR + "/right_leg_scene.xml";
    //const std::string model_path = (argc > 1) ? argv[1] : model_path; // if input model path in the arg then use that path
    std::cout<< "Input model path: " + model_path + "\n";
    char loadError[1024] = ""; // character array, size 1024

    // load/compile xml model
    // model_path.c_str() return a read-only pointer to const std::string model_path
    mjModel* mj_model = mj_loadXML(model_path.c_str(), nullptr, loadError, sizeof(loadError));

    // check if loading xml is ok or not
    if (!mj_model){ // if mj_model is a nullptr
        std::fprintf(stderr, "failed to load %s: %s\n", model_path.c_str(), loadError);
    return 1; // stop program
    }

    std::printf("Load xml successful \n");

    mjData* mj_data = mj_makeData(mj_model); // pointer to mjData struct

    //-------------------------------------------------------------------
    // Compile pinocchio urdf model
    //-------------------------------------------------------------------

    const std::string model_urdf = "models/urdf/right_leg_6dof.urdf";
    pinocchio::Model pin_model;
    pinocchio::urdf::buildModel(model_urdf, pin_model);
    pinocchio::Data pin_data(pin_model);

    std::printf("Compile pinocchio from urdf ok\n");

    // Init mujoco UI from GLFW_callbacks
    UIctr ui(mj_model, mj_data);
    ui.iniGLFW();
    ui.enableTracking();
    ui.createWindow(model_path.c_str(), /*saveVideo=*/false);
    
    double simStart = mj_data->time;

    while (!glfwWindowShouldClose(ui.window))
    {
        simStart = mj_data->time;
        while (mj_data->time - simStart < 1.0 / 60.0 && ui.runSim) // ensure the rendering in real-time
        {
            ui.applyPerturbation();
            mj_step(mj_model, mj_data);
        }
        ui.updateScene(); //
    }

    return 0;
}
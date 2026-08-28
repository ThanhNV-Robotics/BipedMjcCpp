#include "my_gait_scheduler.h"
#include "foot_placement.h"
#include "Eigen/Dense"
#include "data_bus.h"
#include "matplotlibcpp.h"
#include <vector>
#include <iostream>

namespace plt = matplotlibcpp;

using namespace std;

int main ()
{
    cout << "Test foot placement" << endl;

    const double dt = 0.001; // sampling time
    const double tSwingIn = 1;

    // init classes
    MyGaitScheduler gaitScheduler_(tSwingIn, dt);
    FootPlacement footPlacement_;
    cout << "Created a gait scheduler" << endl;
    const int model_nv = 12;
    DataBus RobotState(model_nv); // 12 = model, init its own motion state to Stand

    // Nominal standing pose. This test has no state estimator/dynamics feeding
    // DataBus, so these stay fixed for the whole run -- just enough for the
    // scheduler + foot placement's own computation to be exercised with
    // sane numbers instead of uninitialized garbage. Values match the
    // stand_legLength/width_hips used in float_control.cpp and walk_wbc.cpp.
    const double standLegLength = 0.75;
    RobotState.width_hips = 0.334;
    RobotState.rpy[0] = RobotState.rpy[1] = RobotState.rpy[2] = 0;
    RobotState.base_omega_W.setZero();
    RobotState.base_pos << 0, 0, standLegLength;
    RobotState.hip_r_pos_W << 0, -RobotState.width_hips / 2, standLegLength;
    RobotState.hip_l_pos_W << 0,  RobotState.width_hips / 2, standLegLength;
    RobotState.fe_r_pos_W  << 0, -RobotState.width_hips / 2, 0;
    RobotState.fe_l_pos_W  << 0,  RobotState.width_hips / 2, 0;
    footPlacement_.legLength = standLegLength;

    double t{0}; // time
    double sim_duration = 6;
    double startWalkingTime = 3;

    std::vector<double> timePlot, phiPlot;
    std::vector<Eigen::Vector3d> feRPlot, feLPlot, swingStartPlot, posHipPlot, posSTPlot, hipRPlot, hipLPlot;
    std::vector<Eigen::Vector3d> swingDesPlot;
    std::vector<double> footX_plot, footY_plot, footZ_plot;

    while (t < sim_duration)
    {
        if (t >= startWalkingTime)
        {
            // gaitScheduler_.start(); // start the gait scheduler
            // RobotState.motionState = DataBus::Walk;
            // gaitScheduler_.dataBusRead(RobotState);
            // gaitScheduler_.step();
            // gaitScheduler_.dataBusWrite(RobotState);

            // footPlacement_.dataBusRead(RobotState);
            // footPlacement_.getSwingPos();
            // footPlacement_.dataBusWrite(RobotState);
        }

        t += dt;
    }

    plt::figure();
    plt::plot(timePlot, phiPlot);
    plt::xlabel("time [s]");
    plt::ylabel("phi");

    plt::show();

    return 0;
}
#include "my_gait_scheduler.h"
#include "Eigen/Dense"
#include "data_bus.h"
#include <Eigen/src/Core/Matrix.h>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

using namespace std;

int main ()

{
    cout<<"Test gait scheduler"<<endl;

    const double dt = 0.001; // sampling time
    const double tSwingIn = 1;

    // init classes
    MyGaitScheduler gaitScheduler_(tSwingIn, dt);
    cout<<"Created a gait scheduler"<<endl;
    const int model_nv = 12; 
    DataBus RobotState(model_nv); //12 = model, init its own motion state to Stand

    double t{0}; //time
    double sim_duration = 6; 
    double startWalkingTime = 3;

    std::vector<double> timePlot, phiPlot;
    std::vector<Eigen::Vector3d> feRPlot, feLPlot, swingStartPlot, posHipPlot, posSTPlot, hipRPlot, hipLPlot;

    while (t < sim_duration )
    {

        if (t >= startWalkingTime)
        {
            gaitScheduler_.start(); // start the gait scheduler
            RobotState.motionState = DataBus::Walk;
            // cout<<"Robot motion state changed to Walk"<<endl;
            gaitScheduler_.dataBusRead(RobotState);
            gaitScheduler_.step();
            gaitScheduler_.dataBusWrite(RobotState);
        }

        timePlot.push_back(t);
        phiPlot.push_back(RobotState.phi);
        feRPlot.push_back(RobotState.fe_r_pos_W);
        feLPlot.push_back(RobotState.fe_l_pos_W);
        swingStartPlot.push_back(RobotState.swingStartPos_W);
        posHipPlot.push_back(RobotState.posHip_W);
        posSTPlot.push_back(RobotState.posST_W);
        hipRPlot.push_back(RobotState.hip_r_pos_W);
        hipLPlot.push_back(RobotState.hip_l_pos_W);

        t += dt;
    }

    plt::figure();
    plt::plot(timePlot, phiPlot);
    plt::xlabel("time [s]");
    plt::ylabel("phi");

    auto component = [](const std::vector<Eigen::Vector3d> &v, int idx) {
        std::vector<double> out(v.size());
        for (size_t i = 0; i < v.size(); i++)
            out[i] = v[i](idx);
        return out;
    };
    auto plotVec3 = [&](const std::string &title, const std::vector<Eigen::Vector3d> &v) {
        plt::figure();
        plt::named_plot("x", timePlot, component(v, 0));
        plt::named_plot("y", timePlot, component(v, 1));
        plt::named_plot("z", timePlot, component(v, 2));
        plt::xlabel("time [s]");
        plt::ylabel(title + " [m]");
        plt::legend();
    };

    plotVec3("fe_r_pos_W", feRPlot);
    plotVec3("fe_l_pos_W", feLPlot);
    plotVec3("swingStartPos_W", swingStartPlot);
    plotVec3("posHip_W", posHipPlot);
    plotVec3("posST_W", posSTPlot);
    plotVec3("hip_r_pos_W", hipRPlot);
    plotVec3("hip_l_pos_W", hipLPlot);

    plt::show();

    return 0;
}
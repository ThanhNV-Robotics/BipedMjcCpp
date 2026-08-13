#include "foot_placement.h"
#include "my_gait_scheduler.h"
#include "joystick_interpreter.h"
#include "data_bus.h"
#include "matplotlibcpp.h"
#include <vector>

#include "bezier_1D.h"

namespace plt = matplotlibcpp;

using namespace std;

int main ()
{
    cout<<"Test foot_placement"<<endl;

    // // Init classes
    const int model_nv = 12;
    const double dt = 0.001;
    // const double tSwingIn = 1;

    // MyGaitScheduler gaitScheduler_(tSwingIn, dt);
    // DataBus RobotState(model_nv);

    // double stand_legLength = 0.75; // desired baselink height
    // double foot_height = 0.07; // distance between the foot ankel joint and the bottom
    // double  xv_des = 0.7;  // desired velocity in x direction

    // RobotState.width_hips = 0.334;
    
    // test bezier interpolation


    const double T = 0.8; // swing time
    const double simeTime = 2; //simulation time
    const double height = 0.7;
    const double len = 0;

    double z{0};
    double t{0};
    double t0 = 0;
    double s; //phase variable
    double trajOut {0};

    std::vector<double> timePlt, phiPlot, zPlot, trajPlot;

    const std::vector<double> Z = {0, height, height, height, 0};
    Bezier_1D Bswpid;
    FootPlacement footPlanner;
    Bswpid.P = Z;

    while ( t < simeTime){
        
        s = (t - t0)/T;

        if (s >= 1)
        {
            s = 0;
            t0 = t;
        }
        
        z = Bswpid.getOut(s);

        footPlanner.phi = s;
        trajOut = footPlanner.Trajectory(0.2, height, len);

        timePlt.push_back(t);
        phiPlot.push_back(s);
        zPlot.push_back(z);
        trajPlot.push_back(trajOut);
        
        t += dt;
    }

    // plotting stuff
    plt::figure();
    plt::plot(timePlt, phiPlot);
    plt::xlabel("time (s)");
    plt::ylabel("phi");

    plt::figure();
    plt::plot(timePlt, zPlot);
    plt::xlabel("time (s)");
    plt::ylabel("Z");

    plt::figure();
    plt::plot(timePlt, trajPlot);
    plt::xlabel("time (s)");
    plt::ylabel("trajectory");
    
    plt::show();

    return 0;
}
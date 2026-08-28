#include "CP_Planning.h"
#include <iostream>
#include <vector>
#include "matplotlibcpp.h"

using namespace std;
namespace plt = matplotlibcpp;
int main ()
{
    cout<<"Test Captured Point Planning"<<endl;
    const double dt = 0.001;
    const double zc = 0.75;
    CP_Planning cp_planning(dt, zc);
    cout<<"Init a CP Planning"<<endl;

    double cxi_x = 0;
    double cxi_y = 0.34; //reference captured point

    std::vector<double> timePlot, com_x, com_y;

    double t{0};
    double sim_duration = 6;

    while (t<sim_duration)
    {
        cp_planning.computeCoM(cxi_x, cxi_y);
        com_x.push_back(cp_planning.xc_);
        com_y.push_back(cp_planning.yc_);
        timePlot.push_back(t);

        t+= dt;
    }

    plt::figure();
    plt::plot(timePlot, com_y);
    plt::xlabel("time [s]");
    plt::ylabel("CoM_y");
    plt::show();

}
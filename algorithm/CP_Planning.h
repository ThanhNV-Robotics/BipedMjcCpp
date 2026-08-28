#pragma once
#include "Eigen/Dense"


class CP_Planning
{
    public:
        const double g = 9.81; //gravity constant
        double t_swing;
        double w;
        CP_Planning (const double dtIn, const double zIn);
        
        double CoM_dynamics (double cxi, double xc); // dx = f(x,u)
        void computeCoM (double cxi_x, double cxi_y);

        
    // private:
        double dt_; // sampling time        
        double xc_, yc_, zc_; // CoM position
        double d_xc_, d_yc_; // CoM velocity

        double cxi_x_, cxi_y_; // capture point

};
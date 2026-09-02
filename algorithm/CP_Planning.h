#pragma once
#include "Eigen/Dense"
#include "data_type.h"
#include "my_gait_scheduler.h"

class CP_Planning
{
    public:
        const double g = 9.81; //gravity constant
        const double wd_hip = 0.15; // hip width
        double t_swing;
        double w;

        double crossoverFraction{0.7};
        CP_Planning (const double dtIn, const double zIn);
        
        double CoM_dynamics (double cxi, double xc); // dx = f(x,u)
        double CP_dynamics (double p, double cxi);
        void computeCoM (double cxi_x, double cxi_y);
        void planWarmingUp (MyGaitScheduler &gait_scheduler);
        void computeCP (double zmp_x, double zmp_y);

        
    // private:
        double dt_; // sampling time        
        double xc_, yc_, zc_; // CoM position
        double d_xc_, d_yc_; // CoM velocity

        double cxi_x_, cxi_y_, cxi_xd_, cxi_yd_; // capture point
        double cxi_x0_, cxi_y0_;

        double px_d_, py_d_; // desired zmp 
        LegState leg_state_;

};
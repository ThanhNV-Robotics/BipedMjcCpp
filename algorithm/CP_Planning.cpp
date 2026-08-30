#include "CP_Planning.h"
#include "data_type.h"
#include <cmath>

CP_Planning::CP_Planning(const double dtIn, const double zIn)
{
    this->dt_ = dtIn;
    this->zc_ = zIn;
    this->w = std::sqrt(this->g/this->zc_);
    xc_ = 0; yc_= 0;
    d_xc_ = d_yc_ = 0;
    leg_state = LegState::DSt; // init at double stand
}

double CP_Planning::CoM_dynamics(double cxi, double xc)
{
    // Input: cxi: capture point input, xc: CoM position
    double d_xc;
    d_xc = this->w* (cxi - xc);
    return d_xc;
}

double CP_Planning::CP_dynamics (double p, double cxi)
{
    double d_cxi;
    d_cxi = this->w*(cxi - p);
    return  d_cxi;
}

void CP_Planning::computeCoM(double cxi_x, double cxi_y)
{
    d_xc_ = CoM_dynamics(cxi_x, this->xc_);
    d_yc_ = CoM_dynamics(cxi_y, this->yc_);
    xc_ += d_xc_*dt_;
    yc_ += d_yc_*dt_;
}

void CP_Planning::planWarmingUp (MyGaitScheduler &gait_scheduler)
{
    // phi is the phase variable
    auto leg_state = gait_scheduler.legState;
    auto phi = gait_scheduler.phi;

    if (leg_state == LegState::LSt)
    {
        cxi_yd = wd_hip;
    }

    if (leg_state == LegState::RSt)
    {
         cxi_yd = -wd_hip;
    }

    if (leg_state == LegState::DSt)
        cxi_yd = 0;

    return;
}


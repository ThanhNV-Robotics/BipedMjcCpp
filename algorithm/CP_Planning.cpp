#include "CP_Planning.h"
#include <cmath>

CP_Planning::CP_Planning(const double dtIn, const double zIn)
{
    this->dt_ = dtIn;
    this->zc_ = zIn;
    this->w = std::sqrt(this->g/this->zc_);
    xc_ = 0; yc_= 0;
    d_xc_ = d_yc_ = 0;
}

double CP_Planning::CoM_dynamics(double cxi, double xc)
{
    // Input: cxi: capture point input, xc: CoM position
    double d_xc;
    d_xc = this->w* (cxi - xc);
    return d_xc;
}

void CP_Planning::computeCoM(double cxi_x, double cxi_y)
{
    d_xc_ = CoM_dynamics(cxi_x, this->xc_);
    d_yc_ = CoM_dynamics(cxi_y, this->yc_);
    xc_ += d_xc_*dt_;
    yc_ += d_yc_*dt_;
}


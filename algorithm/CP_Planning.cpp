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
    cxi_x_ = cxi_y_ = 0;
    leg_state_ = LegState::DSt; // init at double stand
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

void CP_Planning::computeCP (double zmp_x, double zmp_y)
{
    double d_cxi_x{0}, d_cxi_y{0};
    d_cxi_x = CP_dynamics(zmp_x, this->cxi_x_);
    d_cxi_y = CP_dynamics(zmp_y, this->cxi_y_);

    this->cxi_x_ += d_cxi_x*dt_;
    this->cxi_y_ += d_cxi_y*dt_;
}

void CP_Planning::planWarmingUp (MyGaitScheduler &gait_scheduler)
{
    // phi is the phase variable
    auto phi = gait_scheduler.phi;
    t_swing = gait_scheduler.tSwing;

    if (leg_state_ == LegState::DSt && gait_scheduler.legState == LegState::LSt)
    {
        cxi_x0_ = this->cxi_x_;
        cxi_y0_ = this->cxi_y_;

        cxi_xd_ = 0;
        cxi_yd_ = wd_hip;
    }

    if (leg_state_ == LegState::DSt && gait_scheduler.legState == LegState::RSt)
    {
        cxi_x0_ = this->cxi_x_;
        cxi_y0_ = this->cxi_y_;

        cxi_xd_ = 0;
        cxi_yd_ = -wd_hip;
    }

    if (leg_state_ == LegState::LSt && gait_scheduler.legState == LegState::RSt)
    {
        cxi_x0_ = this->cxi_x_;
        cxi_y0_ = this->cxi_y_;

        cxi_xd_ = 0;
        cxi_yd_ = -wd_hip;
    }


    if (leg_state_ == LegState::RSt && gait_scheduler.legState == LegState::LSt)
    {
        cxi_x0_ = this->cxi_x_;
        cxi_y0_ = this->cxi_y_;

        cxi_xd_ = 0;
        cxi_yd_ = wd_hip;
    }

    // calculate ZMP
    const double e = 2.718281828459;
    const double b = std::pow(e, this->w* this->t_swing);
    px_d_ = (cxi_xd_ - b*cxi_x0_)/(1-b);
    py_d_ = (cxi_yd_ - b*cxi_y0_)/(1-b);

    // compute Capture Point
    this->computeCP(px_d_, py_d_);

    // calculate CoM
    this->computeCoM(cxi_x_, cxi_y_);

    this->leg_state_ = gait_scheduler.legState;

    return;
}


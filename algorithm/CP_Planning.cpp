#include "CP_Planning.h"
#include "data_type.h"
#include <cmath>

CP_Planning::CP_Planning(const double dtIn, const double zIn, double wd_hipIn)
{
    this->dt_ = dtIn;
    this->zc_ = zIn;
    this->wd_hip = wd_hipIn;
    this->w = std::sqrt(this->g/this->zc_);
    xc_ = 0; yc_= 0;
    d_xc_ = d_yc_ = 0;
    cxi_x_ = cxi_y_ = 0;
    // cxi_xd_/cxi_yd_/cxi_x0_/cxi_y0_ are only ever assigned inside
    // planWarmingUp()'s DSt->LSt/RSt transition blocks, but its ZMP formula
    // (px_d_ = (cxi_xd_ - b*cxi_x0_)/(1-b)) reads them unconditionally every
    // tick regardless of whether that transition has happened yet -- zero
    // them here so a tick before the first transition (or one that never
    // transitions, e.g. motionState stuck at STAND) computes a harmless
    // px_d_=0 instead of reading uninitialized garbage.
    cxi_xd_ = cxi_yd_ = 0;
    cxi_x0_ = cxi_y0_ = 0;
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

void CP_Planning::planWalking (MyGaitScheduler &gait_scheduler, JoyStickInterpreter &joyStick)
{
    // gait_scheduler: provide the gait phase variable
    // joyStick: provide the walking velocity

    double vx = joyStick.vx_W; // forward walking velocity
    t_swing = gait_scheduler.tSwing;
    // step_length must use the same formula as the Raibert foot placement heuristic
    // (foot_placement.cpp: posDes_W = hipPos_W + 0.5*T*v_des + ...) so that the
    // CP CoM reference advances by the same amount per step as the foot lands forward.
    // Using vx/T here instead caused a 0.1 m/step CoM-vs-foot mismatch that accumulated.
    this->step_length = 0.5 * t_swing * vx;
    auto phi = gait_scheduler.phi; // phase variable

    if (this->step_length >= 0.1) // saturation: max step length 0.2 m
    {
        this->step_length = 0.1;
    }


    // Planning desired Capture Point
    if (leg_state_ == LegState::DSt && gait_scheduler.legState == LegState::LSt)
    {
        cxi_x0_ = this->cxi_x_;
        cxi_y0_ = this->cxi_y_;

        cxi_xd_ += step_length; // first step: advance CoM reference same as steady-state steps
        cxi_yd_ = -0.5*wd_hip;
    }

    if (leg_state_ == LegState::DSt && gait_scheduler.legState == LegState::RSt)
    {
        cxi_x0_ = this->cxi_x_;
        cxi_y0_ = this->cxi_y_;

        cxi_xd_ += step_length; // first step: advance CoM reference same as steady-state steps
        cxi_yd_ = 0.5*wd_hip;
    }

    if (leg_state_ == LegState::LSt && gait_scheduler.legState == LegState::RSt)
    {
        cxi_x0_ = this->cxi_x_;
        cxi_y0_ = this->cxi_y_;

        cxi_xd_ += step_length;
        cxi_yd_ = 0.5*wd_hip;
    }


    if (leg_state_ == LegState::RSt && gait_scheduler.legState == LegState::LSt)
    {
        cxi_x0_ = this->cxi_x_;
        cxi_y0_ = this->cxi_y_;

        cxi_xd_ += step_length;
        cxi_yd_ = -0.5*wd_hip;
    }

    // Compute desired ZMP
    const double e = 2.718281828459;
    if (phi < crossoverFraction)
    {
        const double b = std::pow(e, this->w * crossoverFraction * this->t_swing);
        px_d_ = (cxi_xd_ - b*cxi_x0_)/(1-b);
        py_d_ = (cxi_yd_ - b*cxi_y0_)/(1-b);
    }
    else
    {
        px_d_ = cxi_xd_;
        py_d_ = cxi_yd_;
    }

    // compute Capture Point
    this->computeCP(px_d_, py_d_);

    // calculate CoM
    this->computeCoM(cxi_x_, cxi_y_);

    this->leg_state_ = gait_scheduler.legState;

    return;
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
        cxi_yd_ = -0.5*wd_hip;
    }

    if (leg_state_ == LegState::DSt && gait_scheduler.legState == LegState::RSt)
    {
        cxi_x0_ = this->cxi_x_;
        cxi_y0_ = this->cxi_y_;

        cxi_xd_ = 0;
        cxi_yd_ = 0.5*wd_hip;
    }

    if (leg_state_ == LegState::LSt && gait_scheduler.legState == LegState::RSt)
    {
        cxi_x0_ = this->cxi_x_;
        cxi_y0_ = this->cxi_y_;

        cxi_xd_ = 0;
        cxi_yd_ = 0.5*wd_hip;
    }


    if (leg_state_ == LegState::RSt && gait_scheduler.legState == LegState::LSt)
    {
        cxi_x0_ = this->cxi_x_;
        cxi_y0_ = this->cxi_y_;

        cxi_xd_ = 0;
        cxi_yd_ = -0.5*wd_hip;
    }

    // calculate ZMP -- boundary-value target arrival is aimed at
    // crossoverFraction*t_swing rather than the full phase (see the header
    // comment on crossoverFraction for why), so the CoM finishes its
    // weight-shift roughly in step with FootPlacement's swing foot landing,
    // not only right at phi=1. Once phi passes that point, cxi_x_/cxi_y_
    // have (by construction) already arrived at cxi_xd_/cxi_yd_, so holding
    // the ZMP AT that same target for the remainder of the phase keeps them
    // there (p==cxi at rest satisfies d(cxi)/dt=w*(cxi-p)=0 exactly).
    const double e = 2.718281828459;
    if (phi < crossoverFraction)
    {
        const double b = std::pow(e, this->w * crossoverFraction * this->t_swing);
        px_d_ = (cxi_xd_ - b*cxi_x0_)/(1-b);
        py_d_ = (cxi_yd_ - b*cxi_y0_)/(1-b);
    }
    else
    {
        px_d_ = cxi_xd_;
        py_d_ = cxi_yd_;
    }

    // compute Capture Point
    this->computeCP(px_d_, py_d_);

    // calculate CoM
    this->computeCoM(cxi_x_, cxi_y_);

    this->leg_state_ = gait_scheduler.legState;

    return;
}


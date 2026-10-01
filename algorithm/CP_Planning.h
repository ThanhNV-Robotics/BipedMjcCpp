#pragma once
#include "Eigen/Dense"
#include "data_type.h"
#include "joystick_interpreter.h"
#include "my_gait_scheduler.h"

class CP_Planning
{
    public:
        const double g = 9.81; //gravity constant
        double wd_hip = 0.1; // hip width
        double t_swing;
        double w;
        double step_length;
        // Swing-foot phase variable: mirrors phi (gait_scheduler.phi) tick
        // for tick, but is paired with leg_state_swing_ (not leg_state_) --
        // see leg_state_swing_'s comment below for what the pairing means.
        // Set this way by both planWarmingUp() and planWalking().
        double phi_swing{0.0};

        // The leg_state_ that was active ONE FULL CYCLE AGO, i.e. leg_state_
        // delayed by exactly one leg-state transition. Consumers that select
        // which foot to swing (FootPlacement::updateFromRobot's cp_planner
        // overload) should key off THIS instead of leg_state_: leg_state_
        // always reflects the CoM/ZMP's CURRENT weight-shift target, which
        // hasn't been reached yet while phi is still ramping toward it, so a
        // foot swinging off leg_state_ directly lifts before the ZMP has
        // actually arrived over the other foot (unsupported -> fall). Using
        // the one-cycle-old leg_state_ instead means: while the CoM spends
        // cycle N shifting onto its cycle-N target, the foot lifts for
        // cycle (N-1)'s target, which by now (cycle N) has actually been
        // fully weighted. Starts at DSt (no swing target yet); FootPlacement
        // should treat DSt as "no swing" for the very first cycle.
        LegState leg_state_swing_{LegState::DSt};

        // Scales planWarmingUp()'s sway target (normally +-0.5*wd_hip, a
        // full weight-shift onto one foot, sized for eventually lifting the
        // other) down for callers that want to just sway the CoM while both
        // feet stay planted -- 1.0 (default) preserves the original
        // full-amplitude behavior.
        double swayAmplitudeScale{1.0};

        // World-frame offset added to planWarmingUp()/planWalking()'s
        // cxi_xd_/cxi_yd_ targets -- callers should set this to the robot's
        // ACTUAL CoM position right after initial standing (same value
        // xc_/yc_ get seeded to). Without it, the targets are 0/+-0.5*wd_hip
        // in absolute world-frame terms, so even though xc_/yc_ start at the
        // real stance position, the very first sway target still pulls the
        // CoM toward world (0,0) instead of around the robot's own actual
        // stance -- a real, if usually small, state-estimator-bias-driven
        // "impulse" toward (0,0) at the first target-setting edge.
        double xBias{0.0}, yBias{0.0};
        CP_Planning (const double dtIn, const double zIn, double wd_hipIn);
        
        double CoM_dynamics (double cxi, double xc); // dx = f(x,u)
        double CP_dynamics (double p, double cxi);
        void computeCoM (double cxi_x, double cxi_y);
        void planWarmingUp (MyGaitScheduler &gait_scheduler);
        void planWalking (MyGaitScheduler &gait_scheduler, JoyStickInterpreter &joyStick);
        void computeCP (double zmp_x, double zmp_y);

        // Simple CoM sway: yc_/d_yc_ trace a pure sinusoid around centerY,
        // with NO dependency on the capture-point ODE (computeCP/
        // computeCoM) or leg_state_ -- unlike planWarmingUp()/planWalking()
        // there's no leg-state-transition target-setting at all. Still
        // driven by gait_scheduler.phi/tSwing for its time base (so the
        // sway stays in lockstep with whatever's advancing phi elsewhere),
        // not an internally-accumulated clock. For isolating whether the
        // null-space IK / DynWBC QP can track a smooth, bounded CoM
        // position command at all, with none of the capture-point/foot-
        // swing machinery in the loop.
        void planSwaySin (MyGaitScheduler &gait_scheduler, double centerY);
        double swayAmplitude{0.05};     // meters, sine amplitude around centerY
        double swayCyclesPerPhase{1.0}; // number of full left-right-left sine cycles per gait phase (phi: 0->1)


    // private:
        double dt_; // sampling time
        double xc_, yc_, zc_; // CoM position
        double d_xc_, d_yc_; // CoM velocity

        double cxi_x_, cxi_y_, cxi_xd_, cxi_yd_; // capture point
        double cxi_x0_, cxi_y0_;

        double px_d_, py_d_; // desired zmp 
        LegState leg_state_;

};
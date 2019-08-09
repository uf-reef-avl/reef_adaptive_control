#include "simple_pid.h"

namespace reef_control
{

//
// Basic initialization
//
SimplePID::SimplePID()
{
  kp_ = 0.0;
  ki_ = 0.0;
  kd_ = 0.0;
  integrator_ = 0.0;
  differentiator_ = 0.0;
  last_error_ = 0.0;
  last_state_ = 0.0;
  tau_ = 0.0;
}

//
// Initialize the controller
//
SimplePID::SimplePID(double p, double i, double d, double max, double min, double tau):
    kp_(p),ki_(i),kd_(d),tau_(tau), max_(max), min_(min)
{
  integrator_ = 0.0;
  differentiator_ = 0.0;
  last_error_ = 0.0;
  last_state_ = 0.0;
}

//
// Compute the control;
//
//x_c is desired state, x is current state
double SimplePID::computePID(double x_c, double x, double dt)
{
  // calculate derivative
  double xdot;
  if(dt > 0.0)
  {
    // Noise reduction (See "Small Unmanned Aircraft". Chapter 6. Slide 31/33)
      // d/dx w.r.t. error:: differentiator_ = (2*tau_ - dt)/(2*tau_ + dt)*differentiator_ + 2/(2*tau_ + dt)*(error - last_error_);
      differentiator_ = (2*tau_ - dt)/(2*tau_ + dt)*differentiator_ + 2/(2*tau_ + dt)*(x - last_state_);
      xdot = differentiator_;
  }
  else
  {
      xdot = 0.0;
  }

  last_state_ = x;
  return computePID(x_c, x, xdot, dt);
}
//x_c is desired state, x is current state
double SimplePID::computePID(double x_c, double x, double x_dot, double dt)
{
  double error = x_c -x;

  // Initialize Terms
  double p_term = error * kp_;
  double i_term = 0.0f;
  double d_term = 0.0f;

  if(dt == 0.0 || !std::isfinite(error))
  {
    last_error_ = error;
    last_state_ = x;
    return 0.0;
  }

  // If there is a derivative term
  if (kd_ > 0.0f)
  {
    d_term = kd_ * x_dot;
  }

  //If there is an integrator term and we are updating integrators
  if (ki_ > 0.0f)
  {
    // integrate
    integrator_ += error * dt;
    // calculate I term
    i_term = ki_ * integrator_;
  }

  last_error_ = error;
  last_state_ = x;

  // sum three terms
  double u = p_term + d_term + i_term;

  // Integrator anti-windup
  //// Include reference to Dr. Beard's notes here
  double u_sat = (u > max_) ? max_ : (u < min_) ? min_ : u;
  if (u != u_sat && std::abs(i_term) > std::abs(u - p_term + d_term) && ki_ > 0.0f)
    integrator_ = (u_sat - p_term + d_term)/ki_;

  // Set output
  return u_sat;
}

//
// Late initialization or redo
//
void SimplePID::setGains(double p, double i, double d, double tau)
{
  //! \todo Should we really be zeroing this while we are gain tuning?
  kp_ = p;
  ki_ = i;
  kd_ = d;
  tau_ = tau;
}

void SimplePID::setMinMax(double min, double max)
{
  min_ = min;
  max_ = max;
}

// double SimplePID::saturate(double value, double low, double high)
// {
// 	return std::min(std::max(value,low),high);
// }

} // namespace relative_nav

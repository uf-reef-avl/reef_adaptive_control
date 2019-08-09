#include <ros/ros.h>
#include "controller.h"
#include "PID.h"

namespace reef_control
{
  PIDController::PIDController() : Controller()
  {
    func_ = boost::bind(&PIDController::gainsCallback,this,_1,_2);
    server_.setCallback(func_);
    desired_state_pub_ = nh_.advertise<reef_msgs::DesiredState>("controller_state", 1);
    nh_.param<bool>("face_target",face_target_, false);
    nh_.param<bool>("fly_fixed_wing",fly_fixed_wing_, false);
  }

  void PIDController::gainsCallback(reef_adaptive_control::GainsConfig &config, uint32_t level)
  {
    double xIntegrator = config.xIntegrator?1.0:0.0;
    double uIntegrator = config.uIntegrator?1.0:0.0;

    ROS_INFO("New u(PID):   %0.4f,%0.4f,%0.4f", config.uP,uIntegrator*config.uI,config.uD);
    ROS_INFO("New v(PID):   %0.4f,%0.4f,%0.4f", config.vP,uIntegrator*config.vI,config.vD);
    ROS_INFO("New w(PID):   %0.4f,%0.4f,%0.4f", config.wP,uIntegrator*config.wI,config.wD);
    ROS_INFO("New YAW(PID): %0.4f,%0.4f,%0.4f", config.yawP,config.yawI,config.yawD);

    kp = config.kp;
    deadzone = config.deadzone;
    vel_max = config.max_vel;
    x_0 = config.center_point;
    alpha = config.alpha;
    sigma = 0.1;

    d_.setGains( config.dP,   xIntegrator*config.dI,   config.dD,   config.nedtau);
    yaw_.setGains(   config.yawP, config.yawI,             config.yawD, config.yawtau);
    u_.setGains(     config.uP,   uIntegrator*config.uI,   config.uD,   config.uvtau);
    v_.setGains(     config.vP,   uIntegrator*config.vI,   config.vD,   config.uvtau);
    w_.setGains(     config.wP,   uIntegrator*config.wI,   config.wD,   config.uvtau);

    d_.setMinMax(-0.7, config.max_d);
    yaw_.setMinMax(-2.0, 2.0);
    u_.setMinMax(-config.max_u, config.max_u);
    v_.setMinMax(-config.max_v, config.max_v);
    w_.setMinMax(-config.max_w, config.max_w);
  }

  void PIDController::computeCommand(const nav_msgs::Odometry current_state_,
                                     reef_msgs::DesiredState& desired_state,
                                     double dt)  {
    if(!initialized_) {
      d_.clearIntegrator();
      yaw_.clearIntegrator();
      u_.clearIntegrator();
      v_.clearIntegrator();
      w_.clearIntegrator();
    }

    current_yaw = reef_msgs::get_yaw(current_state_.pose.pose.orientation);
    desired_state.velocity.z = d_.computePID(desired_state.pose.z, current_state_.pose.pose.position.z, dt);
    desired_state.acceleration.z = w_.computePID(desired_state.velocity.z, current_state_.twist.twist.linear.z, dt);

    if(desired_state.position_valid)
    {
      if(face_target_)
        desired_state.pose.yaw = theta + current_yaw;
      desired_state.velocity.yaw = yaw_.computePID(desired_state.pose.yaw, current_yaw, dt);
      lookupTable(desired_state,current_state_);
      desired_state.velocity_valid = true;
    }

    if(desired_state.velocity_valid)
    {
      desired_state.acceleration.x = u_.computePID(desired_state.velocity.x, current_state_.twist.twist.linear.x, dt);
      desired_state.acceleration.y = v_.computePID(desired_state.velocity.y, current_state_.twist.twist.linear.y, dt);
    }
    desired_state_pub_.publish(desired_state);
  }

  void PIDController::lookupTable(reef_msgs::DesiredState& desired_state ,const nav_msgs::Odometry& current_state_)
  {
    double velocity_request;
    double euclidian_distance;
    double x_error;
    double y_error;
    double multiplier;

    x_error = desired_state.pose.x - current_state_.pose.pose.position.x;
    y_error = desired_state.pose.y - current_state_.pose.pose.position.y;
    euclidian_distance = sqrt(x_error * x_error + y_error * y_error);
    if (euclidian_distance < deadzone)
      velocity_request = 0;
    else
      velocity_request = vel_max * 1 / ( 1 + kp * exp( - (euclidian_distance - x_0)/alpha ) );

    theta = atan2(y_error,x_error) - current_yaw;

    if(fly_fixed_wing_){
      multiplier = exp( - ( desired_state.velocity.yaw * desired_state.velocity.yaw ) / (2 * sigma * sigma) );
      multiplier = std::min(multiplier + 0.1 , 1.0);
    }
    else
      multiplier = 1;


    desired_state.velocity.x = multiplier  * velocity_request * cos(theta);
    desired_state.velocity.y = multiplier * velocity_request * sin(theta);

  }


}

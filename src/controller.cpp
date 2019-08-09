#include <ros/ros.h>
#include "controller.h"

namespace reef_control
{
  Controller::Controller() :
    nh_(),
    nh_private_("~"),
    armed_(false),
    initialized_(false),
    is_flying_(false)
  {

    // Get Global Parameters
    nh_.param<double>("gravity", gravity_, 9.80665);

    command_publisher_       = nh_.advertise<rosflight_msgs::Command>("command", 1);

    desired_state_subcriber_ = nh_.subscribe("desired_state",1,&Controller::desiredStateCallback,this);
    status_subscriber_       = nh_.subscribe("status",1,&Controller::statusCallback,this);
    is_flying_subcriber_     = nh_.subscribe("is_flying",1, &Controller::isflyingCallback,this);
    current_state_subcriber_ = nh_.subscribe("xyz_estimate", 1, &Controller::currentStateCallback,this);
    rc_in_subcriber_         = nh_.subscribe("rc_raw",1,&Controller::RCInCallback,this);
    pose_subcriber_          = nh_.subscribe("pose_stamped", 1, &Controller::poseCallback,this);

    time_of_previous_control_ = ros::Time(0);

  }

  void Controller::desiredStateCallback(const reef_msgs::DesiredState& msg)
  {
    desired_state_ = msg;
  }

  void Controller::currentStateCallback(const reef_msgs::XYZEstimate& msg)
  {
    current_state_.header = msg.header;
    current_state_.twist.twist.linear.x = msg.xy_plus.x_dot;
    current_state_.twist.twist.linear.y = msg.xy_plus.y_dot;
    current_state_.twist.twist.linear.z = msg.z_plus.z_dot;
    current_state_.pose.pose.position.z = msg.z_plus.z;
    computeCommand();
  }

  void Controller::poseCallback(const geometry_msgs::PoseStamped& msg)
  {
    current_state_.pose.pose.position.x = msg.pose.position.x;
    current_state_.pose.pose.position.y = msg.pose.position.y;
    current_state_.pose.pose.orientation = msg.pose.orientation;

  }

  void Controller::statusCallback(const rosflight_msgs::Status &msg)
  {
    armed_ = msg.armed;
    initialized_ = armed_;
  }

  void Controller::isflyingCallback(const std_msgs::Bool &msg)
  {
    is_flying_ = msg.data;
    initialized_ = is_flying_ && armed_;
  }

  void Controller::RCInCallback(const rosflight_msgs::RCRaw &msg)
  {

  }

  void Controller::computeCommand()
  {
    // Time calculation
    dt = (current_state_.header.stamp - time_of_previous_control_).toSec();
    time_of_previous_control_ = current_state_.header.stamp;
    if(dt <= 0.0000001)
    {
      // Don't do anything if dt is really close (or equal to) zero
      return;
    }

    computeCommand(current_state_ ,desired_state_,dt);

    phi_desired = desired_state_.acceleration.y;
    theta_desired = -desired_state_.acceleration.x;
    thrust = -desired_state_.acceleration.z;

    /*
    accel_out = Eigen::Vector3d(desired_state_.acceleration.x, desired_state_.acceleration.y, desired_state_.acceleration.z );
    total_accel = sqrt( pow(accel_out.x(),2) + pow(accel_out.y(),2) + pow((1 - accel_out.z()),2) );
    thrust = total_accel * hover_throttle_ ;

    if(thrust > 0.001)
    {
      phi_desired = asin(accel_out.y() / total_accel);
      theta_desired = -1.0 * asin(accel_out.x() / total_accel);
    }
    else
    {
        phi_desired = 0;
        theta_desired = 0;
    }
    */

    command.mode = rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE;
    command.F = std::min(std::max(thrust, 0.0), 1.0);
    if(!desired_state_.attitude_valid && !desired_state_.altitude_only) {
      command.ignore = 0x00;
      command.x = std::min(std::max(phi_desired, -1.0 * max_roll_), max_roll_);
      command.y = std::min(std::max(theta_desired, -1.0 * max_pitch_), max_pitch_);
      command.z = std::min(std::max(desired_state_.velocity.yaw, -1.0 * max_yaw_rate_), max_yaw_rate_);
    }else if(desired_state_.altitude_only)
      command.ignore = 0x07;
    else
    {
      command.ignore = 0x00;
      command.x = desired_state_.attitude.x;
      command.y = desired_state_.attitude.y;
      command.z = desired_state_.attitude.yaw;
    }

    command_publisher_.publish(command);
  }

} //namespace

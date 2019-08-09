#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <ros/ros.h>

#include <rosflight_msgs/Command.h>
#include <rosflight_msgs/Status.h>
#include <rosflight_msgs/RCRaw.h>

#include <math.h>
#include <eigen3/Eigen/Core>

#include <reef_msgs/XYZEstimate.h>
#include <reef_msgs/DesiredState.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>

#include <simple_pid.h>


namespace reef_control
{
  class Controller
  {
  public:
    Controller();
    ~Controller(){}

    bool initialized_;
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

   private:
    bool is_flying_;                       // Set by is_flying callback
    bool armed_;
    bool xy_control_flag;

    nav_msgs::Odometry current_state_;
    reef_msgs::DesiredState desired_state_;

    ros::Publisher command_publisher_;

    ros::Subscriber status_subscriber_;
    ros::Subscriber current_state_subcriber_;
    ros::Subscriber desired_state_subcriber_;
    ros::Subscriber is_flying_subcriber_;
    ros::Subscriber pose_subcriber_;
    ros::Subscriber rc_in_subcriber_;

    ros::Time time_of_previous_control_;
    rosflight_msgs::Command command;

    double mass_;
    double gravity_;
    double max_roll_, max_pitch_, max_yaw_rate_;
    double max_thrust_, min_thrust_;
    double hover_throttle_;
    double max_u_, max_v_, max_w_;
    double dt;
    double total_accel;
    double thrust;
    double phi_desired;
    double theta_desired;

    Eigen::Vector3d accel_out;

    void currentStateCallback(const reef_msgs::XYZEstimate& msg);
    void desiredStateCallback(const reef_msgs::DesiredState& msg);
    void poseCallback(const geometry_msgs::PoseStamped& msg);
    void isflyingCallback(const std_msgs::Bool& msg);
    void statusCallback(const rosflight_msgs::Status &msg);
    void RCInCallback(const rosflight_msgs::RCRaw &msg);
    void computeCommand();  // Computes and sends command message

    // Virtual Function
    virtual void computeCommand(const nav_msgs::Odometry current_state,
                  reef_msgs::DesiredState& desired_state,
                  double dt) = 0;

  };
}
#endif

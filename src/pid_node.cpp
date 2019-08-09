#include "ros/ros.h"
#include "controller.h"
#include "PID.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "position_controller_pid");
  reef_control::PIDController pid_object;

  ros::spin();
  return 0;
}

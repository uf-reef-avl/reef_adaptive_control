//
// Created by jonah on 7/15/19.
//

#include <ros/ros.h>
#include <reef_msgs/DesiredState.h>
#include <reef_msgs/XYZEstimate.h>

#include "adaptive_control.h"
#include "state_space.h"
#include "adaptive_io.h"

int main(int argc, char **argv)
{
    // Initialize the node
    ros::init(argc, argv, "adaptive_node");
    ROS_INFO("ADAPTIVE_NODE: initialized adaptive node");

    // Set adaptive controller model
    // TODO: change to parameters obtained from rosparam
    Eigen::VectorXd numerator;
    Eigen::VectorXd denominator;
    ROS_INFO("ADAPTIVE_NODE: instantiated numerator and denominator");

    numerator.setZero(3);
    denominator.setZero(4);

    numerator << 45, 51, 12;
    denominator << 45, 54, 14.4, 1;
    ROS_INFO("ADAPTIVE_NODE: set values of numerator and denominator");

    TransferFunction ModelTF(numerator, denominator);
    ROS_INFO("ADAPTIVE_NODE: set model transfer function");

    AdaptiveIO AdaptiveController(ModelTF);
    ROS_INFO("ADAPTIVE_NODE: set adaptive controller with TF");

    // Set loop rate and begin pub/sub in ROS
    ros::Rate loop_rate(40);
    while (ros::ok())
    {
        AdaptiveController.Publish();
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
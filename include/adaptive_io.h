//
// Created by jonah on 7/15/19.
//

#ifndef REEF_CONTROL_ADAPTIVE_IO_H
#define REEF_CONTROL_ADAPTIVE_IO_H

#include <ros/ros.h>
#include <reef_msgs/DesiredState.h>
#include <reef_msgs/XYZEstimate.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include <dynamic_reconfigure/server.h>
#include <reef_adaptive_control/AdapativeConfig.h>

#include "adaptive_control.h"
#include "state_space.h"

class AdaptiveIO
{
public:
    // Constructor creates publishers and subscribers
    AdaptiveIO() : n(""), n_private("~")
    {
        DesiredStateSub = n.subscribe("desired_state", 100, &AdaptiveIO::desiredStateCallback, this);
        CurrentStateSub = n.subscribe("xyz_estimate", 100, &AdaptiveIO::currentStateCallback, this);
        PlantTorqueSub = n.subscribe("total_torque", 100, &AdaptiveIO::plantTorqueCallback, this);
        StatusSub = n.subscribe("adapt_on", 100, &AdaptiveIO::statusCallback, this);
        GainSub = n.subscribe("adaptive_gain", 1000, &AdaptiveIO::gainCallback, this);

        MRAC_Publisher = n.advertise<geometry_msgs::Vector3Stamped>("added_torque", 40);
        ROS_INFO("ADAPTIVE_NODE: set publishers and subscribers");

        func_ = boost::bind(&AdaptiveIO::gainsCallback, this, _1, _2);
        server_.setCallback(func_);

    }

    // Single-argument constructor creates pubs and subs, and adaptive controllers as well
    AdaptiveIO(TransferFunction ModelTF) : n(""), n_private("~")
    {
        DesiredStateSub = n.subscribe("desired_state", 100, &AdaptiveIO::desiredStateCallback, this);
        CurrentStateSub = n.subscribe("xyz_estimate", 100, &AdaptiveIO::currentStateCallback, this);
        PlantTorqueSub = n.subscribe("total_torque", 100, &AdaptiveIO::plantTorqueCallback, this);
        StatusSub = n.subscribe("enable_adaptation", 100, &AdaptiveIO::statusCallback, this);
        GainSub = n.subscribe("adaptive_gain", 1000, &AdaptiveIO::gainCallback, this);

        MRAC_Publisher = n.advertise<geometry_msgs::Vector3Stamped>("added_torque", 40);
        ROS_INFO("ADAPTIVE_NODE: set publishers and subscribers");
        ROS_WARN_STREAM("IM IN THIS CONSTRUCTOR");
        func_ = boost::bind(&AdaptiveIO::gainsCallback, this, _1, _2);
        server_.setCallback(func_);

        SetControlTransFcn(ModelTF);
    }

    // Method for publishing adaptive control output
    void Publish()
    {
        // Get current time and set as this update time
        adapt_time = ros::Time::now().nsec;

        // At start, discard the delta time because it has no prior measurement
        if (last_adapt_time == 0)
            last_adapt_time = adapt_time;

        double delta_t = (adapt_time - last_adapt_time) * 1e-9;

        // Adapt and update if delta_t is a nonzero value
        if (delta_t > 0.0)
        {
            x_ctrl.AdaptParameters(delta_t);
            y_ctrl.AdaptParameters(delta_t);

            // PITCH IS INVERTED
            double adaptive_pitch_torque = (-1) * x_ctrl.GetAdaptiveTorque();
            double adaptive_roll_torque = y_ctrl.GetAdaptiveTorque();

            //ROS_INFO("ADAPTIVE_NODE: adaptive_pitch_torque = %f, adaptive_roll_torque = %f",
                    //adaptive_pitch_torque, adaptive_roll_torque);

            msg.header.stamp = ros::Time::now();
            msg.vector.x = adaptive_roll_torque;
            msg.vector.y = adaptive_pitch_torque;
            msg.vector.z = 0.0;

            MRAC_Publisher.publish(msg);
        }
    }

    // Controllers can be set from a transfer function
    void SetControlTransFcn(TransferFunction TF)
    {
        x_ctrl.InitializeControl(TF);
        y_ctrl.InitializeControl(TF);
        ROS_INFO("ADAPTIVE_NODE: set control transfer function");
    }

    // Enables adaptation in adaptive controllers
    bool EnableAdaptation()
    {
        bool state;

        state = x_ctrl.EnableAdaptation();
        state = state && y_ctrl.EnableAdaptation();

        if (state)
            ROS_INFO("ADAPTIVE_NODE: adaptation enabled");
        else
            ROS_WARN("ADAPTIVE_NODE: could not enable adaptation");
        return state;
    }

    // Disables adaptation in adaptive controllers
    void DisableAndResetAdaptation()
    {
        x_ctrl.DisableAndResetAdaptation();
        y_ctrl.DisableAndResetAdaptation();
        ROS_INFO("ADAPTIVE_NODE: adaptation set to off");
    }

    // Sets adaptive gain for adaptive controllers
    void SetAdaptiveGain(double gamma)
    {
        x_ctrl.SetAdaptiveGain(gamma);
        y_ctrl.SetAdaptiveGain(gamma);
        ROS_INFO("ADAPTIVE_NODE: adaptation gain set to %f", gamma);
    }

private:
    ros::Publisher MRAC_Publisher;

    ros::Subscriber DesiredStateSub;
    ros::Subscriber CurrentStateSub;
    ros::Subscriber PlantTorqueSub;
    ros::Subscriber StatusSub;
    ros::Subscriber GainSub;

    dynamic_reconfigure::Server<reef_adaptive_control::AdapativeConfig> server_;
    dynamic_reconfigure::Server<reef_adaptive_control::AdapativeConfig>::CallbackType func_;

    ros::NodeHandle n;
    ros::NodeHandle n_private;

    geometry_msgs::Vector3Stamped msg;

    AdaptiveController x_ctrl;
    AdaptiveController y_ctrl;

    double last_input_time_up = 0;
    double input_time_up = 0;

    double last_input_time_yp = 0;
    double input_time_yp = 0;

    double last_input_time_r = 0;
    double input_time_r = 0;

    double last_adapt_time = 0;
    double adapt_time = 0;

    // Callback function for desired state message
    void desiredStateCallback(const reef_msgs::DesiredState& msg)
    {
        // Update time
        input_time_r = double(msg.header.stamp.nsec);

        // At start, discard the delta time because it has no prior measurement
        if (last_input_time_r == 0)
        {
            (last_input_time_r = input_time_r);
        }

        // Assign delta t as the difference in times (value given in seconds)
        double delta_t = (input_time_r - last_input_time_r) * 1e-9;

        // Test if delta_t is positive and nonzero
        if (delta_t > 0.0)
        {
            double x_desired_vel = msg.velocity.x;
            double y_desired_vel = msg.velocity.y;

            //ROS_INFO("ADAPTIVE_NODE: x_desired_vel = %f, y_desired_vel = %f", x_desired_vel, y_desired_vel);

            // Update adaptive controller parameters from input data
            x_ctrl.UpdateReferenceInput(x_desired_vel, delta_t);
            y_ctrl.UpdateReferenceInput(y_desired_vel, delta_t);
        }
        // Update input time for next iteration
        last_input_time_r = input_time_r;
    }

    // callback function for current state estimate
    void currentStateCallback(const reef_msgs::XYZEstimate& msg)
    {
        // Update time
        input_time_yp = double(msg.header.stamp.nsec);

        // At start, discard the delta time because it has no prior measurement
        if (last_input_time_yp == 0)
        {
            (last_input_time_yp = input_time_yp);
        }

        // Assign delta t as the difference in times (value given in seconds)
        double delta_t = (input_time_yp - last_input_time_yp) * 1e-9;

        // Test if delta_t is positive and nonzero
        if (delta_t > 0.0)
        {
            double x_vel = msg.xy_plus.x_dot;
            double y_vel = msg.xy_plus.y_dot;

            //ROS_INFO("ADAPTIVE_NODE: x_vel = %f, y_vel = %f", x_vel, y_vel);

            // Update adaptive controller parameters from input data
            x_ctrl.UpdatePlantOutput(x_vel, delta_t);
            y_ctrl.UpdatePlantOutput(y_vel, delta_t);
        }
        // Update input time for next iteration
        last_input_time_yp = input_time_yp;
    }

    // callback function for control input update
    void plantTorqueCallback(const geometry_msgs::Vector3Stamped& msg)
    {
        // Update time
        input_time_up = double(msg.header.stamp.nsec);

        // At start, discard the delta time because it has no prior measurement
        if (last_input_time_up == 0)
        {
            (last_input_time_up = input_time_up);
        }

        // Assign delta t as the difference in times (value given in seconds)
        double delta_t = (input_time_up - last_input_time_up) * 1e-9;

        // Test if delta_t is positive and nonzero
        if (delta_t > 0.0)
        {
            // NOTE: PITCH ANGLE IS INVERTED
            double roll_torque = double(msg.vector.x);
            double pitch_torque = -1 * double(msg.vector.y);

            //ROS_INFO("ADAPTIVE_NODE: pitch_torque = %f, roll_torque = %f", pitch_torque, roll_torque);

            // Update adaptive controller parameters from input data
            x_ctrl.UpdateControlInput(pitch_torque, delta_t);
            y_ctrl.UpdateControlInput(roll_torque, delta_t);
        }
        // Update input time for next iteration
        last_input_time_up = input_time_up;
    }

    // Callback for interpreting on/off boolean message
    void statusCallback(const std_msgs::Bool& msg)
    {
        if (msg.data)
            EnableAdaptation();
        else
            DisableAndResetAdaptation();
    }

    void gainCallback(const std_msgs::Float64 gain_msg)
    {
        if (gain_msg.data > 0)
        {
            ROS_WARN("ADAPTIVE_NODE: Increasing adaptive gain can cause instability.");
            SetAdaptiveGain(gain_msg.data);
        }
        else
        {
            ROS_WARN("ADAPTIVE_NODE: adaptive gain must be a positive value.");
        }
    }

    void gainsCallback(reef_adaptive_control::AdapativeConfig &config, uint32_t level){

        x_ctrl.SetAdaptiveGain(config.x_gamma);
        y_ctrl.SetAdaptiveGain(config.y_gamma);
        ROS_INFO_STREAM("ADAPTIVE_NODE: X adaptation gain set to " << config.x_gamma);
        ROS_INFO_STREAM("ADAPTIVE_NODE: Y adaptation gain set to " << config.y_gamma);

    }
};

#endif //REEF_CONTROL_ADAPTIVE_IO_H

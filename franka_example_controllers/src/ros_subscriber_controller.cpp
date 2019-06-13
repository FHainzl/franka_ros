// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/ros_subscriber_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_example_controllers {

    bool RosSubscriberController::init(hardware_interface::RobotHW* robot_hardware,
                                       ros::NodeHandle& node_handle) {

        std::vector<std::string> joint_names;
        if (!node_handle.getParam("joint_names", joint_names)) {
            ROS_ERROR("RosSubscriberController: Could not parse joint names");
        }

        if (joint_names.size() != 7) {
            ROS_ERROR_STREAM("RosSubscriberController: Wrong number of joint names, got "
                                     << joint_names.size() << " instead of 7 names!");
            return false;
        }

        velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
        if (velocity_joint_interface_ == nullptr) {
          ROS_ERROR(
              "RosSubscriberController: Error getting velocity joint interface from hardware!");
          return false;
        }

        velocity_joint_handles_.resize(7);
        for (size_t i = 0; i < 7; ++i) {
            try {
                velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
            } catch (const hardware_interface::HardwareInterfaceException &ex) {
                ROS_ERROR_STREAM(
                        "RosSubscriberController: Exception getting joint velocity handles: " << ex.what());
                return false;
            }
        }

        joint_command_subscriber_ = node_handle.subscribe ("controller_command/joint_command",
                                                           10, &RosSubscriberController::joint_state_callback, this);

        return true;
    }

    void RosSubscriberController::starting(const ros::Time& /* time */) {
        joint_command_.velocity.resize(7);
        joint_command_.position.resize(7);
        joint_command_.effort.resize(7);
        joint_command_.name.resize(7);

        for (size_t i = 0; i < 7; ++i) {
            joint_command_.velocity.at(i) = 0.0f;
            joint_command_.position.at(i) = 0.0f;
            joint_command_.effort.at(i) = 0.0f;
            joint_command_.name.at(i) = "velocity";
        }
    }

    void RosSubscriberController::update(const ros::Time& /* time */,
                                         const ros::Duration& period) {

        if (joint_command_.name.size() != 7) {
            ROS_ERROR_ONCE(
                    "RosSubscriberController: Expected an array size of 7 for the name!");
            for (int joint=0; joint<7; ++joint) {
                velocity_joint_handles_.at(joint).setCommand(0.0f);
            }
            return;
        }

        if(std::find(joint_command_.name.begin(), joint_command_.name.end(), "position") != joint_command_.name.end()) {
            if (joint_command_.position.size() != 7) {
                ROS_ERROR_ONCE(
                        "RosSubscriberController: Expected an array size of 7 for the position!");
            }
        }
        if(std::find(joint_command_.name.begin(), joint_command_.name.end(), "velocity") != joint_command_.name.end()) {
            if (joint_command_.velocity.size() != 7) {
                ROS_ERROR_ONCE(
                        "RosSubscriberController: Expected an array size of 7 for the velocity!");
            }
        }
        if(std::find(joint_command_.name.begin(), joint_command_.name.end(), "effort") != joint_command_.name.end()) {
            if (joint_command_.effort.size() != 7) {
                ROS_ERROR_ONCE(
                        "RosSubscriberController: Expected an array size of 7 for the effort!");
            }
        }

        for (int joint=0; joint<7; ++joint) {

            if (joint_command_.name.at(joint) == "position") {
                float current_pos = velocity_joint_handles_.at(joint).getPosition();
                float target_pos = joint_command_.position.at(joint);
                float velocity = PoseToVelocityController (current_pos, target_pos);
                velocity_joint_handles_.at(joint).setCommand(velocity);
            } else if (joint_command_.name.at(joint) == "velocity") {
                velocity_joint_handles_.at(joint).setCommand(joint_command_.velocity.at(joint));
            } else if (joint_command_.name.at(joint) == "effort") {
                float current_vel = velocity_joint_handles_.at(joint).getVelocity();
                float target_accel = joint_command_.effort.at(joint);
                float velocity = EffortToAngularAccelerationController (current_vel, target_accel);
                velocity_joint_handles_.at(joint).setCommand(velocity);
            } else {
                ROS_ERROR_ONCE(
                        "RosSubscriberController: invalid joint motion type. Needs to be either position, velocity or effort.");
                velocity_joint_handles_.at(joint).setCommand(0.0f);
            }
        }

    }


    float RosSubscriberController::PoseToVelocityController (float current_pos, float target_pos) {
        float current_position = current_pos;
        float target_position = target_pos;
        float position_deviation = -(current_position-target_position);
        const float k_epsilon = 0.01; //The deviation which is allowed from a position
        const float k_scaling = 5.0; //scaling factor for the controller
        float v = k_scaling * position_deviation;
        const float k_max_velocity = 2.0;
        const float k_min_velocity = 0.2;

        // If ouside of tolerance
        if (v<-k_epsilon || k_epsilon < v){

            // If too negative, clip
            if (v < -k_max_velocity) {
                v = -k_max_velocity;
            }

            // If too positive, clip
            if (v > k_max_velocity) {
                v = k_max_velocity;
            }

            // Lower magnitude limit for negative v
            if (-k_min_velocity < v && v < -k_epsilon) {
                v = -k_min_velocity;
            }

            // Lower magnitude limit for positive v
            if (k_min_velocity > v && v > k_epsilon) {
                v = k_min_velocity;
            }

           return (v);
        } else{
            v = 0.0f;
            return (v);
        }
    }


    float RosSubscriberController::EffortToAngularAccelerationController (float current_vel, float target_accel) {
        const float k_update_frequency = 1000.0;
        return current_vel + (target_accel / k_update_frequency);
    }


    void RosSubscriberController::joint_state_callback (const sensor_msgs::JointState joint_command) {
        joint_command_ = joint_command;
    }

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::RosSubscriberController,
        controller_interface::ControllerBase)

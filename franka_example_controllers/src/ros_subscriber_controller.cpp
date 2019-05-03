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

  velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();

  if (velocity_joint_interface_ == nullptr) {
    ROS_ERROR(
        "RosSubscriberController: Error getting velocity joint interface from hardware!");
    return false;
  }

  position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface_ == nullptr) {
    ROS_ERROR(
            "JointPositionExampleController: Error getting position joint interface from hardware!");
    return false;
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("RosSubscriberController: Could not parse joint names");
  }

  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("RosSubscriberController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }

  velocity_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException &ex) {
      ROS_ERROR_STREAM(
              "RosSubscriberController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

    position_joint_handles_.resize(7);
    for (size_t i = 0; i < 7; ++i) {
      try {
        position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
      } catch (const hardware_interface::HardwareInterfaceException &e) {
        ROS_ERROR_STREAM(
                "JointPositionExampleController: Exception getting joint handles: " << e.what());
        return false;
      }
    }

      joint_command_.velocity.resize(7);
      for (size_t i = 0; i < 7; ++i) {
        joint_command_.velocity.at(i) = 0.0f;
        joint_command_.position.at(i) = position_joint_handles_[i].getPosition();
      }

      joint_command_subscriber_ = node_handle.subscribe ("controller_command/joint_command",
            10, &RosSubscriberController::joint_state_callback, this);

  return true;
}

void RosSubscriberController::starting(const ros::Time& /* time */) {
}

void RosSubscriberController::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {
  float velocity_speed_sum = 0.0f;
  for (size_t i = 0; i < 7; ++i) {
    velocity_speed_sum += joint_command_.velocity.at(i);
  }

  if (velocity_speed_sum != 0.0f) {
    if (joint_command_.velocity.size() != 7) {
      ROS_ERROR_STREAM(
              "RosSubscriberController: Was expecting 7 values for the joint velocity. ");
    } else {
      for (size_t i = 0; i < 7; ++i) {
        velocity_joint_handles_.at(i).setCommand(joint_command_.velocity.at(i));
      }
    }
  } else {
    if (joint_command_.position.size() != 7) {
      ROS_ERROR_STREAM(
              "RosSubscriberController: Was expecting 7 values for the joint position. ");
    } else {
      for (size_t i = 0; i < 7; ++i) {
        position_joint_handles_.at(i).setCommand(joint_command_.position.at(i));
      }
    }
  }

}

void RosSubscriberController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

void RosSubscriberController::joint_state_callback (const sensor_msgs::JointState joint_command) {
  joint_command_ = joint_command;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::RosSubscriberController,
                       controller_interface::ControllerBase)

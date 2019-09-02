# ROS integration for Franka Emika research robots

[![Build Status][travis-status]][travis]

See the [Franka Control Interface (FCI) documentation][fci-docs] for more information.
## About this fork

This fork adds a ROS node which can be launched with
```
roslaunch franka_example_controllers ros_subscriber_controller.launch
```
The node will subscribe to  **controller_command/joint_command** which is sensor message of type JointState.

To use you need to set the name of each joint to the mode you want to give the control in. The options are **position, velocity or effort**.

**Note that the "effort" does not actually set force/torque targets, but angular acceleration.** 

## Usage
A Python interface to control the robot using the ROS Subscriber Controller can be found in https://gitlab.com/iaai/appliedAI/research/panda/panda_utilitites. 

## License

All packages of `franka_ros` are licensed under the [Apache 2.0 license][apache-2.0].

[apache-2.0]: https://www.apache.org/licenses/LICENSE-2.0.html
[fci-docs]: https://frankaemika.github.io/docs
[travis-status]: https://travis-ci.org/frankaemika/franka_ros.svg?branch=kinetic-devel
[travis]: https://travis-ci.org/frankaemika/franka_ros

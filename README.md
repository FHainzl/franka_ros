# ROS integration for Franka Emika research robots

[![Build Status][travis-status]][travis]

See the [Franka Control Interface (FCI) documentation][fci-docs] for more information.
## About this fork

This fork adds a ROS node which can be launched with
```
roslaunch ros_subscriber_controller.launch
```
The node will subscribe to  **controller_command/joint_command** which is sensor message of type JointState.

In this message either the joint velocities or joint positions can be specified. The joint position is prioritized higher, in order to provide a joint velocity **set all joint positions to zero**.

## License

All packages of `franka_ros` are licensed under the [Apache 2.0 license][apache-2.0].

[apache-2.0]: https://www.apache.org/licenses/LICENSE-2.0.html
[fci-docs]: https://frankaemika.github.io/docs
[travis-status]: https://travis-ci.org/frankaemika/franka_ros.svg?branch=kinetic-devel
[travis]: https://travis-ci.org/frankaemika/franka_ros

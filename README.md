# Coordination Controllers

[![license - apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)

**A ROS package providing coordinated motion controllers for the
[ros_control](https://github.com/ros-controls/ros2_control) framework.** In
general, these controllers support coordinating the task-space motion of a
manipulator with external axes or positioner units.

This package implements a decentralized architecture for coordinated motion
control. The feedback communication between a manipulator and its coordinated
unit is done via ROS messaging. This allows each mechanical unit (i.e.
manipulators and positioners) to be controlled from separate hardware devices.

## Installation

This package depends on
[`taskspace_control`](https://github.com/alexarbogast/taskspace_control).
Create a ROS 2 workspace and populate a src directory with this package and its
dependencies.

```bash
sudo apt update
rosdep update
cd src
vcs import < coordination_controllers/coordination_controllers.repos
rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
```

Build the packages:

```bash
cd <colcon_workspace>
colcon build
```

## Coordinated Controller Configuration

An example configuration can be found in the `coordinated_motion_examples`
[configuration](./coordinated_motion_examples/config). Or refer to the `yaml`
files that lay out each controllers parameters for the [parameter
generator](https://github.com/PickNikRobotics/generate_parameter_library). The
`rr_objective_type` and `pos_objective_type` parameters decide the type of
objectives used for redundancy resolution and positioner control. The available
controller types can be found in the [plugin
description](./coordinated_motion_controllers/coordinated_controller_plugins.xml).

## Running the Demos

Launch the demo multi-robot system with the desired robot.

```bash
ros2 launch coordinated_motion_examples two_robot_bringup.launch.py
```
```
# robot_type (default "robot6R"): One of 'robot6R', 'robot7R
```

In another terminal, launch the control demo with the desired controller.

```bash
ros2 launch coordinated_motion_examples coordinated_motion_demo.launch.py robot_type:=robot6R controller:=pose_controller
```
```
# 'robot_type':
#     Select which robot configuration to use. Valid choices are: ['robot6R', 'robot7R']
#     (default: 'robot6R')
#
# 'controller':
#     Which controller should be started?. Valid choices are: ['coordinated_pose_controller']
#     (default: 'coordinated_pose_controller')
```

Modify the positioner and redundancy resolution objectives in the respective
`coordinated_motion_examples/config/<robot_type>_controllers.yaml`.

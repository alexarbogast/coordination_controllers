# Coordination Controllers

**A ROS package providing coordinated motion controllers for the
[ros_control](https://github.com/ros-controls/ros_control) framework.** In
general, these controllers support coordinating the task space motion of a
manipulator with external axes or positioner units.

This package implements a decentralized architecture for coordinated motion
control. The feedback communication between a manipulator and its coordinated
unit is done via ROS messaging. This allows each mechanical unit (i.e.
manipulators and positioners) to be controlled from separate hardware devices.

## Installation

This package depends on
[`taskspace_control`](https://github.com/alexarbogast/taskspace_control). Create
a ROS workspace and build the package and dependencies.

```bash
# create workspace
mkdir -p catkin_ws/src && cd catkin_ws/src

# clone project and dependencies
git clone git@github.com:alexarbogast/taskspace_control.git
git clone git@github.com:alexarbogast/coordination_controllers.git

# build packages
cd ../
catkin build
```

## Coordinated Controller Configuration

An example controller configuration is provided below. Each coordinated robot
provides a coordinate controller configuration. The `rr_objective_type` and
`pos_objective_type` decide the type of objectives used for redundancy
resolution and positioner control. The available controller types can be found
in the [plugin
description](./coordinated_motion_controllers/coordinated_controller_plugins.xml).
See the [configuration](./coordinated_motion_examples/config) in
`coordinated_motion_examples` for further reference.

```yaml
coordinated_pose_controller:
  type: coordinated_motion_controllers/PoseController
  joints:
    - rob1_joint1
    - rob1_joint2
    - rob1_joint3
    - rob1_joint4
    - rob1_joint5
    - rob1_joint6

  positioner_link: positioner # the reference frame for coordinated motion
  base_link: rob1_base_link
  eef_link: rob1_tool

  k_position: 10.0
  k_orient: 10.0

  rr_objective_type: minimize_velocity
  # rr_objective_type: maximize_manipulability
  # rr_objective:
  #   k_manip: 10.0
  # rr_objective_type: avoid_joint_limits
  # rr_objective:
  #   K_limits: 10.0

  pos_objective_type: minimize_velocity
  # pos_objective_type: match_configuration
  # pos_objective:
  #   match_config: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

  positioner_topic: /positioner/joint_states
  setpoint_topic: setpoint
```

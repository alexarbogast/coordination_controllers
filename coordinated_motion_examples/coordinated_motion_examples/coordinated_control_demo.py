#!/usr/bin/env python3

import numpy as np
import quaternion
import abc

from taskspace_control_examples import ControlDemo

LINEAR_VELOCITY = 0.300

robot_params = {
    "robot6R": {
        "orient": np.quaternion(1.0, 0.0, 0.0, 0.0),
        "home": [0.0, -1.125, 2.275, -1.15, 1.571, 0.0],
    },
    "robot7R": {
        "orient": np.quaternion(0.0, 1.0, 0.0, 0.0),
        "home": [0.0, 0.0, 0.0, -np.pi / 2, 0.0, np.pi / 2, 0.0],
    },
}


class CoordinatedControlDemo(ControlDemo):
    def __init__(self, node_name: str, setpoint_hz=250):
        super().__init__(node_name, setpoint_hz)

        # Declare and get ROS2 parameters
        self.declare_parameter("arm_id", "")
        self.arm_id = self.get_parameter("arm_id").value
        if self.arm_id == "":
            raise RuntimeError("Missing required parameter: arm_id")

        self.declare_parameter("robot_type", "robot6R")
        robot_type = self.get_parameter("robot_type").value

        self.static_orient = robot_params[robot_type]["orient"]
        self.home = robot_params[robot_type]["home"]

    def run(self):
        self.small_circle()
        # self.small_hypotrochoid()
        # self.hypotrochoid()
        # self.circle()

    # Specific paths are implemented in base classes
    @abc.abstractmethod
    def small_circle(self):
        pass

    @abc.abstractmethod
    def small_hypotrochoid(self):
        pass

    @abc.abstractmethod
    def circle(self):
        pass

    @abc.abstractmethod
    def hypotrochoid(self):
        pass

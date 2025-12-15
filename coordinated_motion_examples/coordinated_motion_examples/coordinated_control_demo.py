#!/usr/bin/env python3

import numpy as np
import quaternion
import abc

from std_msgs.msg import ColorRGBA

from taskspace_control_examples import ControlDemo, PathVisualization

robot_params = {
    "robot6R": {
        "home": [0.0, -1.125, 2.275, -1.15, 1.571, 0.0],
    },
    "robot7R": {
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
        self.robot_type = self.get_parameter("robot_type").value

        self.home = robot_params[self.robot_type]["home"]

        # add path visualization in namespace
        self.path_viz = PathVisualization(
            self, 0.007, ColorRGBA(r=0.96, g=0.38, b=0.21, a=1.0), ns=self.arm_id
        )

    def run(self):
        self.small_circle()
        self.small_hypotrochoid()
        self.hypotrochoid()
        self.circle()

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

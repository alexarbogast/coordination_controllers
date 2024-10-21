#!/usr/bin/env python3

import numpy as np
import rospy

from coordinated_motion_examples import ControlDemo
from coordinated_motion_examples.trajectory import *


class BaseframeControlDemo(ControlDemo):
    def __init__(self, setpoint_hz=1000):
        super(BaseframeControlDemo, self).__init__(setpoint_hz)
        self.home = [-0.368, -0.845, 1.858, 0.558, 1.571, 0.189]

    def run(self):
        self.start_joint_control()
        self.joint_controller_client.move_joint(self.home, 1.0)

        self.start_taskspace_control()

        self.test_cube()
        self.base_frame_circle()
        self.base_frame_hypotrochoid()

        self.start_joint_control()
        self.joint_controller_client.move_joint(self.home, 1.0)

    def test_cube(self):
        center = np.array([0.5, 0.0, 0.065])
        points = [
            center + np.array([-0.25, -0.5, 0.1]),
            center + np.array([-0.25, 0.5, 0.1]),
            center + np.array([-1.0, 0.5, 0.1]),
            center + np.array([-1.0, 0.5, 0.6]),
            center + np.array([-0.25, 0.5, 0.6]),
            center + np.array([-0.25, -0.5, 0.6]),
            center + np.array([-1.0, -0.5, 0.6]),
            center + np.array([-1.0, -0.5, 0.1]),
            center + np.array([-0.25, -0.5, 0.1]),
        ]

        self.path_viz.visualize_path(points, f"{self.arm_id}_base_link")
        points.insert(0, center)

        for i in range(len(points) - 1):
            current = points[i]
            next = points[i + 1]
            self.execute_linear_path(current, next, 2.0)

        self.path_viz.reset()

    def base_frame_circle(self):
        tf = 5
        tt = np.linspace(0, tf, int(self.hz * tf))
        f, f_dot = circular_traj(1 / 7, tf)

        offset = np.array([0.5, 0.0, 0.1])
        ft, f_dott = f(tt) + offset, f_dot(tt)

        self.path_viz.visualize_path(
            [f(t) + offset for t in np.linspace(0, tf, 500)],
            f"{self.arm_id}_base_link",
        )

        self.movel(ft[0], 2)
        self.execute_path(ft, f_dott)
        self.path_viz.reset()

    def base_frame_hypotrochoid(self):
        scale = 1 / 28
        tf = 10
        tt = np.linspace(0, tf, int(self.hz * tf))
        f, f_dot = hypotrochoid_traj(3, 5, 4.5, tf, scaling=Order.THIRD)

        offset = np.array([0.5, 0.0, 0.1])
        ft, f_dott = scale * f(tt) + offset, scale * f_dot(tt)

        self.path_viz.visualize_path(
            [scale * f(t) + offset for t in np.linspace(0, tf, 500)],
            f"{self.arm_id}_base_link",
        )

        self.movel(ft[0], 1)
        self.execute_path(ft, f_dott)
        self.path_viz.reset()


if __name__ == "__main__":
    rospy.init_node("base_frame_control_client")

    try:
        demo = BaseframeControlDemo()
        demo.run()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python3

import numpy as np
import rospy

from geometry_msgs.msg import Quaternion

from taskspace_control_examples import ControlDemo
from taskspace_control_examples.trajectory import *

LINEAR_VELOCITY = 0.300


class CoordinatedControlDemo(ControlDemo):
    def __init__(self, setpoint_hz=1000):
        super(CoordinatedControlDemo, self).__init__(setpoint_hz)
        self.arm_id = rospy.get_param("~arm_id")
        self.static_orient = Quaternion(0.0, 1.0, 0.0, 0.0)
        self.home = [-0.368, -0.845, 1.858, 0.558, 1.571, 0.189]

    def run(self):
        self.start_joint_control()
        self.joint_controller_client.move_joint(self.home, 1.0)

        self.start_taskspace_control()

        self.small_circle()
        self.small_hypotrochoid()
        self.hypotrochoid()
        self.circle()

        self.start_joint_control()
        self.joint_controller_client.move_joint(self.home, 1.0)

    def small_circle(self):
        tf = 7
        tt = np.linspace(0, tf, int(self.hz * tf))
        f, f_dot = circular_traj(1 / 7, tf, phase=np.pi)

        offset = np.array([-0.2, -0.2, 0.005])
        ft, f_dott = f(tt) + offset, f_dot(tt)

        self.path_viz.visualize_path(
            [f(t) + offset for t in np.linspace(0, tf, 500)],
            "positioner",
        )

        self.movel(ft[0], 2)
        self.execute_path(ft, f_dott)
        self.path_viz.reset()

    def small_hypotrochoid(self):
        tf = 7
        tt = np.linspace(0, tf, int(self.hz * tf))
        f, f_dot = hypotrochoid_traj(3, 5, 4.5, tf, scaling=Order.THIRD)

        scaling = 1 / 43
        offset = np.array([-0.175, -0.175, 0.005])
        ft, f_dott = scaling * f(tt) + offset, scaling * f_dot(tt)

        self.path_viz.visualize_path(
            [scaling * f(t) + offset for t in np.linspace(0, tf, 500)],
            "positioner",
        )

        self.movel(ft[0], 2)
        self.execute_path(ft, f_dott)
        self.path_viz.reset()

    def circle(self):
        tf = 10.0
        tt = np.linspace(0, tf, int(self.hz * tf))
        f, f_dot = circular_traj(1 / 4, tf, phase=3 / 2 * np.pi)

        offset = np.array([0.0, 0.0, 0.005])
        ft, f_dott = f(tt) + offset, f_dot(tt)

        self.path_viz.visualize_path(
            [f(t) + offset for t in np.linspace(0, tf, 500)],
            "positioner",
        )

        self.movel(ft[0], 2)
        self.execute_path(ft, f_dott)
        self.path_viz.reset()

    def hypotrochoid(self):
        tf = 12
        tt = np.linspace(0, tf, int(self.hz * tf))
        f, f_dot = hypotrochoid_traj(
            3, 5, 4.5, tf, phase=5 / 3 * np.pi, scaling=Order.THIRD
        )

        scaling = 1 / 25
        offset = np.array([0.0, 0.0, 0.005])
        ft, f_dott = scaling * f(tt) + offset, scaling * f_dot(tt)

        self.path_viz.visualize_path(
            [scaling * f(t) + offset for t in np.linspace(0, tf, 500)],
            "positioner",
        )

        self.movel(ft[0], 2)
        self.execute_path(ft, f_dott)
        self.path_viz.reset()


if __name__ == "__main__":
    rospy.init_node("coordinated_control_client")

    try:
        demo = CoordinatedControlDemo()
        demo.run()
    except rospy.ROSInterruptException:
        pass

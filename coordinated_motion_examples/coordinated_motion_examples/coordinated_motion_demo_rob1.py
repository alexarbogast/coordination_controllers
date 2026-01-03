#!/usr/bin/env python3

import numpy as np
import quaternion
import rclpy
import threading

from taskspace_control_examples.trajectory import *
from coordinated_motion_examples import CoordinatedControlDemo

robot_params = {
    "robot6R": {"orient": np.quaternion(1.0, 0.0, 0.0, 0.0)},
    "robot7R": {"orient": np.quaternion(0.0, 1.0, 0.0, 0.0)},
}


class CoordinatedControlDemoRob1(CoordinatedControlDemo):
    """
    Routines are selected for all robots in `coordinated_control_demo.py`
    """

    def __init__(self, node_name: str, setpoint_hz=250):
        super().__init__(node_name, setpoint_hz)
        self.static_orient = robot_params[self.robot_type]["orient"]

    def small_circle(self):
        tf = 15
        tt = np.linspace(0, tf, int(self.hz * tf))
        f, f_dot = circular_traj(1 / 7, tf, phase=np.pi)

        offset = np.array([-0.1, -0.2, 0.005])
        ft, f_dott = f(tt) + offset, f_dot(tt)

        self.path_viz.visualize_path(
            [f(t) + offset for t in np.linspace(0, tf, 500)],
            "positioner",
        )

        self.movel(ft[0], self.static_orient, 2)
        self.execute_path(ft, f_dott, self.static_orient)
        self.path_viz.reset()

    def small_hypotrochoid(self):
        tf = 15
        tt = np.linspace(0, tf, int(self.hz * tf))
        f, f_dot = hypotrochoid_traj(3, 5, 4.5, tf, scaling=Order.FIRST)

        scaling = 1 / 43
        offset = np.array([-0.1, -0.175, 0.005])
        ft, f_dott = scaling * f(tt) + offset, scaling * f_dot(tt)

        self.path_viz.visualize_path(
            [scaling * f(t) + offset for t in np.linspace(0, tf, 500)],
            "positioner",
        )

        self.movel(ft[0], self.static_orient, 2)
        self.execute_path(ft, f_dott, self.static_orient)
        self.path_viz.reset()

    def circle(self):
        tf = 15.0
        tt = np.linspace(0, tf, int(self.hz * tf))
        f, f_dot = circular_traj(1 / 4, tf, phase=3 / 2 * np.pi)

        offset = np.array([0.0, 0.0, 0.005])
        ft, f_dott = f(tt) + offset, f_dot(tt)

        self.path_viz.visualize_path(
            [f(t) + offset for t in np.linspace(0, tf, 500)],
            "positioner",
        )

        self.movel(ft[0], self.static_orient, 2)
        self.execute_path(ft, f_dott, self.static_orient)
        self.path_viz.reset()

    def hypotrochoid(self):
        tf = 30
        tt = np.linspace(0, tf, int(self.hz * tf))
        f, f_dot = hypotrochoid_traj(
            3, 5, 4.5, tf, phase=5 / 3 * np.pi, scaling=Order.FIRST
        )

        scaling = 1 / 25
        offset = np.array([0.0, 0.0, 0.005])
        ft, f_dott = scaling * f(tt) + offset, scaling * f_dot(tt)

        self.path_viz.visualize_path(
            [scaling * f(t) + offset for t in np.linspace(0, tf, 500)],
            "positioner",
        )

        self.movel(ft[0], self.static_orient, 2)
        self.execute_path(ft, f_dott, self.static_orient)
        self.path_viz.reset()


def main(args=None):
    rclpy.init(args=args)
    node = CoordinatedControlDemoRob1("rob1_coordinated_control_demo")

    try:
        threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
        node.run()
    except Exception as e:
        node.get_logger().error(f"Exception in demo: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

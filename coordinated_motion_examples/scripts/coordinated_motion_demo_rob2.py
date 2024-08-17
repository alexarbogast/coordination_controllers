import numpy as np
import rospy

from coordinated_motion_examples import ControlDemo
from coordinated_motion_examples.path import *

LINEAR_VELOCITY = 0.300


class CoordinatedControlDemo(ControlDemo):
    def __init__(self):
        super(CoordinatedControlDemo, self).__init__()
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
        scaling = 1 / 7
        offset = np.array([0.2, 0.2, 0.005])
        tt = np.linspace(0, 6 * np.pi, 15000)
        f, f_dot = circle(scaling)

        self.path_viz.visualize_path(
            [f(t) + offset for t in np.linspace(0, 6 * np.pi, 500)],
            "positioner",
        )

        self.execute_path(f, f_dot, offset, tt)
        self.path_viz.reset()

    def small_hypotrochoid(self):
        scaling = 1 / 43
        offset = np.array([0.175, 0.175, 0.005])
        tt = np.linspace(0, 6 * np.pi, 15000)
        f, f_dot = hypotrochoid(scaling)

        self.path_viz.visualize_path(
            [f(t) + offset for t in np.linspace(0, 6 * np.pi, 500)],
            "positioner",
        )

        self.execute_path(f, f_dot, offset, tt)
        self.path_viz.reset()

    def circle(self):
        scaling = 1 / 4
        offset = np.array([0.0, 0.0, 0.005])
        tt = np.linspace(np.pi / 2, 5 * np.pi / 2, 10000)
        f, f_dot = circle(scaling)

        self.path_viz.visualize_path(
            [f(t) + offset for t in np.linspace(0, 6 * np.pi, 500)],
            "positioner",
        )

        self.execute_path(f, f_dot, offset, tt)
        self.path_viz.reset()

    def hypotrochoid(self):
        scaling = 1 / 25
        offset = np.array([0.0, 0.0, 0.005])
        tt = np.linspace(0, 6 * np.pi, 20000)
        f, f_dot = hypotrochoid(scaling)

        self.path_viz.visualize_path(
            [f(t) + offset for t in np.linspace(0, 6 * np.pi, 500)],
            "positioner",
        )

        self.execute_path(f, f_dot, offset, tt)
        self.path_viz.reset()


if __name__ == "__main__":
    rospy.init_node("coordinated_control_client")

    try:
        demo = CoordinatedControlDemo()
        demo.run()
    except rospy.ROSInterruptException:
        pass

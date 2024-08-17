import numpy as np
import rospy

from coordinated_motion_examples import ControlDemo
from coordinated_motion_examples.path import *

LINEAR_VELOCITY = 0.300


class BaseframeControlDemo(ControlDemo):
    def __init__(self):
        super(BaseframeControlDemo, self).__init__()
        self.home = [-0.368, -0.845, 1.858, 0.558, 1.571, 0.189]

    def run(self):
        self.start_joint_control()
        self.joint_controller_client.move_joint(self.home, 1.0)

        self.start_taskspace_control()

        # self.test_cube()
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
            self.execute_linear_path(current, next, LINEAR_VELOCITY)

        self.path_viz.reset()

    def base_frame_circle(self):
        scaling = 1 / 7
        offset = np.array([0.5, 0.0, 0.1])
        tt = np.linspace(0, 6 * np.pi, 10000)
        f, f_dot = circle(scaling)

        self.path_viz.visualize_path(
            [f(t) + offset for t in np.linspace(0, 6 * np.pi, 500)],
            f"{self.arm_id}_base_link",
        )

        self.execute_path(f, f_dot, offset, tt)
        self.path_viz.reset()

    def base_frame_hypotrochoid(self):
        scaling = 1 / 28
        offset = np.array([0.5, 0.0, 0.1])
        tt = np.linspace(0, 6 * np.pi, 10000)
        f, f_dot = hypotrochoid(scaling)

        self.path_viz.visualize_path(
            [f(t) + offset for t in np.linspace(0, 6 * np.pi, 500)],
            f"{self.arm_id}_base_link",
        )

        self.execute_path(f, f_dot, offset, tt)
        self.path_viz.reset()


if __name__ == "__main__":
    rospy.init_node("base_frame_control_client")

    try:
        demo = BaseframeControlDemo()
        demo.run()
    except rospy.ROSInterruptException:
        pass

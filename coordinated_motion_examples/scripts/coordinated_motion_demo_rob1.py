import numpy as np
import rospy

from geometry_msgs.msg import Point, Vector3, Quaternion
from std_msgs.msg import ColorRGBA
from coordinated_control_msgs.msg import AxiallySymmetricSetpoint

from coordinated_motion_examples import (
    ControllerClient,
    JointControllerClient,
    ControllerManagerClient,
    PathVisualization,
)
from coordinated_motion_examples.path import linear_path, hypotrochoid

NS = "rob1"
LINEAR_VELOCITY = 0.300


class CoordinatedMotionDemo:
    def __init__(self):
        self.controller_client = ControllerClient("coordinated_motion_controller")
        self.joint_controller_client = JointControllerClient(
            "joint_trajectory_controller"
        )

        self.controller_manager_client = ControllerManagerClient()
        self.path_viz = PathVisualization(0.007, ColorRGBA(0.96, 0.38, 0.21, 1.0), NS)

        self.home = [-0.368, -0.845, 1.858, 0.558, 1.571, 0.189]

    def run(self):
        self.start_joint_control()
        self.joint_controller_client.move_joint(self.home, 1.0)

        self.start_coordinated_control()
        self.coordinated_hypotrochoid()

        self.start_joint_control()
        self.joint_controller_client.move_joint(self.home, 1.0)

    def coordinated_hypotrochoid(self):
        scaling = 1 / 25
        offset = np.array([0.0, 0.0, 0.01])
        tt = np.linspace(np.pi, 7 * np.pi, 1500)
        f, f_dot = hypotrochoid(scaling)

        self.path_viz.visualize_path(
            [f(t) + offset for t in np.linspace(np.pi, 7 * np.pi, 500)],
            "positioner",
        )

        rate = rospy.Rate(100)
        setpoint = AxiallySymmetricSetpoint()
        setpoint.pose.orientation = Quaternion(0.0, 1.0, 0.0, 0.0)

        # travel to start
        pose = self.controller_client.get_pose()
        if pose is None:
            return
        current_position = np.array([pose.position.x, pose.position.y, pose.position.z])
        init_path_p = f(tt[0]) + offset

        g, g_dot, dur = linear_path(current_position, init_path_p, LINEAR_VELOCITY)
        for t in np.linspace(0, 1, int(dur * 100)):
            gt = g(t)
            g_dott = g_dot(t)
            setpoint.pose.position = Point(gt[0], gt[1], gt[2])
            setpoint.velocity = Vector3(g_dott[0], g_dott[1], g_dott[2])
            self.controller_client.publish_setpoint(setpoint)
            rate.sleep()

        # follow path
        for t in tt:
            ft = f(t) + offset
            f_dott = f_dot(t)

            setpoint.pose.position = Point(ft[0], ft[1], ft[2])
            setpoint.velocity = Vector3(f_dott[0], f_dott[1], f_dott[2])
            self.controller_client.publish_setpoint(setpoint)
            rate.sleep()

        self.path_viz.reset()

    def start_joint_control(self):
        self.controller_manager_client.switch_controller(
            [self.joint_controller_client.name], [self.controller_client.name]
        )

    def start_coordinated_control(self):
        self.controller_manager_client.switch_controller(
            [self.controller_client.name], [self.joint_controller_client.name]
        )


if __name__ == "__main__":
    rospy.init_node("coordinated_motion_client")

    try:
        demo = CoordinatedMotionDemo()
        demo.run()
    except rospy.ROSInterruptException:
        pass

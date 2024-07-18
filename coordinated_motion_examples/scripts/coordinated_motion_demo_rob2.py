import numpy as np
import rospy

from geometry_msgs.msg import Vector3
from coordinated_control_msgs.msg import RobotSetpoint
from std_msgs.msg import ColorRGBA

from coordinated_motion_examples import PathVisualization, CoordinatedMotionClient

LINEAR_VELOCITY = 0.300


def linear_path(start, end, vel):
    diff = end - start
    path_len = np.linalg.norm(diff)
    u = diff / path_len
    dur = path_len / vel

    f = lambda t: start + t * diff
    f_dot = lambda t: vel * u
    return f, f_dot, dur


def hypotrochoid(scaling):
    # https://www.desmos.com/calculator/r3ltep03hx
    f = lambda t: scaling * np.array(
        [
            2 * np.cos(t) + 4.5 * np.cos(2 / 3 * t),
            2 * np.sin(t) - 4.5 * np.sin(2 / 3 * t),
            0,
        ]
    )
    f_dot = lambda t: scaling * np.array(
        [
            -2 * np.sin(t) - 3 * np.sin(2 / 3 * t),
            2 * np.cos(t) - 3 * np.cos(2 / 3 * t),
            0,
        ]
    )
    return f, f_dot


class CoordinatedMotionDemo:
    def __init__(self):
        self.motion_client = CoordinatedMotionClient()
        self.path_viz = PathVisualization(
            0.007, ColorRGBA(244 / 255, 96 / 255, 54 / 255, 0.8), "rob2"
        )

        self.home = [
            -0.3682676987777704,
            -0.845456854726745,
            1.8581698861744447,
            0.5580733655537403,
            1.5707974618414442,
            0.18855996186926843,
        ]

    def run(self):
        self.motion_client.start_joint_control()
        self.motion_client.move_joint(self.home, 1.0)

        self.motion_client.start_base_frame_control()
        self.base_frame_hypotrochoid()

        self.motion_client.start_coordinated_control()
        self.coordinated_hypotrochoid()

        self.motion_client.start_joint_control()
        self.motion_client.move_joint(self.home, 1.0)

    def base_frame_hypotrochoid(self):
        scaling = 1 / 25
        offset = np.array([0.5, 0.0, 0.1])
        tt = np.linspace(0, 6 * np.pi, 1000)
        f, f_dot = hypotrochoid(scaling)

        self.path_viz.visualize_path(
            [f(t) + offset for t in np.linspace(0, 6 * np.pi, 500)],
            "rob2_base_link",
        )

        rate = rospy.Rate(100)
        setpoint = RobotSetpoint()
        setpoint.pose.aiming = Vector3(0.0, 0.0, 1.0)

        # travel to start
        pose = self.motion_client.get_pose()
        current_position = np.array([pose.position.x, pose.position.y, pose.position.z])
        init_path_p = f(tt[0]) + offset

        g, g_dot, dur = linear_path(current_position, init_path_p, LINEAR_VELOCITY)
        for t in np.linspace(0, 1, int(dur * 100)):
            gt = g(t)
            g_dott = g_dot(t)
            setpoint.pose.position = Vector3(gt[0], gt[1], gt[2])
            setpoint.velocity = Vector3(g_dott[0], g_dott[1], g_dott[2])
            self.motion_client.publish_setpoint(setpoint)
            rate.sleep()

        # follow path
        for t in tt:
            ft = f(t) + offset
            f_dott = f_dot(t)

            setpoint.pose.position = Vector3(ft[0], ft[1], ft[2])
            setpoint.velocity = Vector3(f_dott[0], f_dott[1], f_dott[2])
            self.motion_client.publish_setpoint(setpoint)
            rate.sleep()

        self.path_viz.reset()

    def coordinated_hypotrochoid(self):
        scaling = 1 / 25
        offset = np.array([0.0, 0.0, 0.01])
        tt = np.linspace(0, 6 * np.pi, 1500)
        f, f_dot = hypotrochoid(scaling)

        self.path_viz.visualize_path(
            [f(t) + offset for t in np.linspace(np.pi, 7 * np.pi, 500)],
            "positioner",
        )

        rate = rospy.Rate(100)
        setpoint = RobotSetpoint()
        setpoint.pose.aiming = Vector3(0.0, 0.0, 1.0)

        # travel to start
        pose = self.motion_client.get_pose()
        current_position = np.array([pose.position.x, pose.position.y, pose.position.z])
        init_path_p = f(tt[0]) + offset

        g, g_dot, dur = linear_path(current_position, init_path_p, LINEAR_VELOCITY)
        for t in np.linspace(0, 1, int(dur * 100)):
            gt = g(t)
            g_dott = g_dot(t)
            setpoint.pose.position = Vector3(gt[0], gt[1], gt[2])
            setpoint.velocity = Vector3(g_dott[0], g_dott[1], g_dott[2])
            self.motion_client.publish_setpoint(setpoint)
            rate.sleep()

        # follow path
        for t in tt:
            ft = f(t) + offset
            f_dott = f_dot(t)

            setpoint.pose.position = Vector3(ft[0], ft[1], ft[2])
            setpoint.velocity = Vector3(f_dott[0], f_dott[1], f_dott[2])
            self.motion_client.publish_setpoint(setpoint)
            rate.sleep()

        self.path_viz.reset()


if __name__ == "__main__":
    rospy.init_node("coordinated_motion_client")

    try:
        demo = CoordinatedMotionDemo()
        demo.run()
    except rospy.ROSInterruptException:
        pass

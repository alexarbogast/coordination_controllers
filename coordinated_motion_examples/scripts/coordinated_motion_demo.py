from typing import List
from numpy.typing import NDArray
import numpy as np
import quaternion

import rospy
import actionlib

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Quaternion as QuaternionMsg

from coordinated_control_msgs.msg import Setpoint


class MarkerVisualization(object):
    def __init__(self, radius: float, color: ColorRGBA):
        self.r = radius
        self.color = color
        self.vis_pub = rospy.Publisher(
            "visualization_marker_array", MarkerArray, queue_size=1, latch=True
        )

    def visualize_path(self, path: List[NDArray], frame="world"):
        marker_id = 0
        marker_array = MarkerArray()

        for i in range(len(path) - 1):
            p1, p2 = path[i], path[i + 1]
            marker = self.make_cylinder(p1, p2)
            marker.id = marker_id
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = frame
            marker.frame_locked = True
            marker_array.markers.append(marker)
            marker_id += 1

        self.vis_pub.publish(marker_array)

    def make_cylinder(self, p1: NDArray, p2: NDArray):
        v = p2 - p1
        h = np.linalg.norm(v)
        center = np.average([p1, p2], axis=0)

        u = v / h
        z_axis = np.array([0, 0, 1])
        axis = np.cross(z_axis, u)
        norm = np.linalg.norm(axis)
        if norm > 0:
            axis /= norm
        angle = np.arccos(np.dot(z_axis, u))
        q = quaternion.from_rotation_vector(angle * axis)

        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.pose.orientation = QuaternionMsg(q.x, q.y, q.z, q.w)
        marker.pose.position = Point(center[0], center[1], center[2])
        marker.scale = Vector3(self.r, self.r, h)
        marker.color = self.color
        return marker

    def delete_all(self):
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.action = marker.DELETEALL
        marker_array.markers.append(marker)
        self.vis_pub.publish(marker_array)


class ControllerManagerClient(object):
    def __init__(self, ns):
        self.ns = ns
        rospy.wait_for_service(f"/{ns}/controller_manager/switch_controller")
        self.switch_controller_client = rospy.ServiceProxy(
            f"/{ns}/controller_manager/switch_controller", SwitchController
        )

    def switch_controller(self, start_controllers, stop_controllers):
        try:
            req = SwitchControllerRequest()
            req.start_controllers = start_controllers
            req.stop_controllers = stop_controllers
            req.strictness = 1
            self.switch_controller_client.call(req)
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")


class CoordinatedMotionDemo(object):
    def __init__(self):
        self.task_space_setpoint_pub = rospy.Publisher(
            "/rob1/task_space_controller/setpoint", Setpoint, latch=True, queue_size=1
        )

        self.coordinated_setpoint_pub = rospy.Publisher(
            "/rob1/coordinated_motion_controller/setpoint",
            Setpoint,
            latch=True,
            queue_size=1,
        )

        self.joint_traj_client = actionlib.SimpleActionClient(
            "/rob1/joint_trajectory_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self.joint_traj_client.wait_for_server()
        self.controller_client = ControllerManagerClient("rob1")
        self.marker_viz = MarkerVisualization(
            0.007,
            ColorRGBA(244 / 255, 96 / 255, 54 / 255, 0.8),
        )

    def run(self):
        self.controller_client.switch_controller(
            ["joint_trajectory_controller"], ["task_space_controller"]
        )
        self.move_home()

        self.controller_client.switch_controller(
            ["task_space_controller"], ["joint_trajectory_controller"]
        )
        self.task_space_hypotrochoid()
        self.controller_client.switch_controller(
            ["coordinated_motion_controller"], ["task_space_controller"]
        )

        self.coordinated_hypotrochoid()
        self.controller_client.switch_controller(
            ["joint_trajectory_controller"], ["coordinated_motion_controller"]
        )
        self.move_home()

    def task_space_hypotrochoid(self):
        # https://www.desmos.com/calculator/r3ltep03hx
        scaling = 1 / 25

        tt = np.linspace(0, 2 * np.pi * 5, 1000)
        f = lambda t, z: scaling * np.array(
            [
                2 * np.cos(t) + 4.5 * np.cos(2 / 3 * t),
                2 * np.sin(t) - 4.5 * np.sin(2 / 3 * t),
                z / scaling,
            ]
        )
        f_dot = lambda t: scaling * np.array(
            [
                -2 * np.sin(t) - 3 * np.sin(2 / 3 * t),
                2 * np.cos(t) - 3 * np.cos(2 / 3 * t),
                0,
            ]
        )

        self.marker_viz.visualize_path(
            [f(t, 0.1) + np.array([0.5, 0.0, 0.0]) for t in tt],
            "rob1_base_link",
        )

        setpoint = Setpoint()
        setpoint.pose.aiming = Vector3(0.0, 0.0, 1.0)

        rate = rospy.Rate(100)
        for t in tt:
            ft = f(t, 0.1)
            f_dott = f_dot(t)

            setpoint.pose.position = Vector3(ft[0] + 0.5, ft[1], ft[2])
            setpoint.velocity = Vector3(f_dott[0], f_dott[1], f_dott[2])
            self.task_space_setpoint_pub.publish(setpoint)
            rate.sleep()

        self.marker_viz.delete_all()

    def coordinated_hypotrochoid(self):
        scaling = 1 / 25

        tt = np.linspace(0, 2 * np.pi * 5, 1000)
        f = lambda t, z: scaling * np.array(
            [
                2 * np.cos(t) + 4.5 * np.cos(2 / 3 * t),
                2 * np.sin(t) - 4.5 * np.sin(2 / 3 * t),
                z / scaling,
            ]
        )
        f_dot = lambda t: scaling * np.array(
            [
                -2 * np.sin(t) - 3 * np.sin(2 / 3 * t),
                2 * np.cos(t) - 3 * np.cos(2 / 3 * t),
                0,
            ]
        )

        self.marker_viz.visualize_path(
            [f(t, 0.01) for t in tt],
            "positioner",
        )

        setpoint = Setpoint()
        setpoint.pose.aiming = Vector3(0.0, 0.0, 1.0)

        rate = rospy.Rate(100)
        for t in tt:
            ft = f(t, 0.01)
            f_dott = f_dot(t)

            setpoint.pose.position = Vector3(ft[0], ft[1], ft[2])
            setpoint.velocity = Vector3(f_dott[0], f_dott[1], f_dott[2])
            self.coordinated_setpoint_pub.publish(setpoint)
            rate.sleep()

    def move_home(self):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = [
            "rob1_joint1",
            "rob1_joint2",
            "rob1_joint3",
            "rob1_joint4",
            "rob1_joint5",
            "rob1_joint6",
        ]

        home = JointTrajectoryPoint()
        home.positions = [
            -0.3682676987777704,
            -0.845456854726745,
            1.8581698861744447,
            0.5580733655537403,
            1.5707974618414442,
            0.18855996186926843,
        ]
        home.time_from_start = rospy.Duration(1)

        goal.trajectory.points = [home]

        self.joint_traj_client.send_goal(goal)
        self.joint_traj_client.wait_for_result()


if __name__ == "__main__":
    rospy.init_node("coordinated_motion_demo")

    try:
        demo = CoordinatedMotionDemo()
        demo.run()
    except rospy.ROSInterruptException:
        pass

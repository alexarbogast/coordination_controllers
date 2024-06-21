import rospy
import actionlib

import numpy as np
import math

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest

from coordinated_control_msgs.msg import Setpoint
from geometry_msgs.msg import Vector3


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

    def run(self):
        self.controller_client.switch_controller(
            ["joint_trajectory_controller"], ["task_space_controller"]
        )
        self.move_home()

        self.controller_client.switch_controller(
            ["task_space_controller"], ["joint_trajectory_controller"]
        )
        self.task_space_demo()

        self.controller_client.switch_controller(
            ["coordinated_motion_controller"], ["task_space_controller"]
        )
        self.coordinated_demo()

        self.controller_client.switch_controller(
            ["joint_trajectory_controller"], ["coordinated_motion_controller"]
        )
        self.move_home()

    def task_space_demo(self):
        path = [
            np.array([0.7, 0.4, 0.2]),
            np.array([0.3, 0.4, 0.2]),
            np.array([0.3, -0.4, 0.2]),
            np.array([0.7, -0.4, 0.2]),
            np.array([0.7, 0.4, 0.2]),
        ]

        setpoint = Setpoint()
        setpoint.pose.aiming = Vector3(0.0, 0.0, 1.0)

        vel = 0.2
        freq = 1000
        rate = rospy.Rate(freq)
        for i in range(len(path) - 1):
            start = path[i]
            end = path[i + 1]

            vec = end - start
            path_len = np.linalg.norm(vec)
            dir = vec / path_len
            vel_vec = dir * vel

            final_t = path_len / vel
            tt = np.linspace(0, final_t, math.ceil(final_t * freq))
            for t in tt:
                pos = start + (dir * vel * t)
                setpoint.pose.position = Vector3(pos[0], pos[1], pos[2])
                setpoint.velocity = Vector3(vel_vec[0], vel_vec[1], vel_vec[2])
                self.task_space_setpoint_pub.publish(setpoint)
                rate.sleep()

    def coordinated_demo(self):
        path = [
            np.array([0.3, 0.3, 0.0]),
            np.array([-0.3, 0.3, 0.0]),
            np.array([-0.3, -0.3, 0.0]),
            np.array([0.3, -0.3, 0.0]),
            np.array([0.3, 0.3, 0.0]),
        ]

        setpoint = Setpoint()
        setpoint.pose.aiming = Vector3(0.0, 0.0, 1.0)

        vel = 0.2
        freq = 1000
        rate = rospy.Rate(freq)
        for i in range(len(path) - 1):
            start = path[i]
            end = path[i + 1]

            vec = end - start
            path_len = np.linalg.norm(vec)
            dir = vec / path_len
            vel_vec = dir * vel

            final_t = path_len / vel
            tt = np.linspace(0, final_t, math.ceil(final_t * freq))
            for t in tt:
                pos = start + (dir * vel * t)
                setpoint.pose.position = Vector3(pos[0], pos[1], pos[2])
                setpoint.velocity = Vector3(vel_vec[0], vel_vec[1], vel_vec[2])
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
            -0.33520197589459866,
            -1.166151476125835,
            2.3323065987852196,
            -1.1661642688951233,
            1.7660348728271527,
            2.5210987150084945e-06,
        ]
        home.time_from_start = rospy.Duration(1)

        goal.trajectory.points = [home]

        self.joint_traj_client.send_goal(goal)
        self.joint_traj_client.wait_for_result()


if __name__ == "__main__":
    rospy.init_node("coordinated_motion_demo")

    demo = CoordinatedMotionDemo()
    demo.run()

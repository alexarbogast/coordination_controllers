import numpy as np
import rospy
import actionlib

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest

from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA
from coordinated_control_msgs.msg import RobotSetpoint

from coordinated_motion_examples import PathVisualization


def hypotrochoid(scaling):
    # https://www.desmos.com/calculator/r3ltep03hx
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
    return f, f_dot


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
            "/rob2/task_space_controller/setpoint",
            RobotSetpoint,
            latch=True,
            queue_size=1,
        )

        self.coordinated_setpoint_pub = rospy.Publisher(
            "/rob2/coordinated_motion_controller/setpoint",
            RobotSetpoint,
            latch=True,
            queue_size=1,
        )

        self.joint_traj_client = actionlib.SimpleActionClient(
            "/rob2/joint_trajectory_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self.joint_traj_client.wait_for_server()
        self.controller_client = ControllerManagerClient("rob2")
        self.path_viz = PathVisualization(
            0.007, ColorRGBA(244 / 255, 96 / 255, 54 / 255, 0.8), "rob2"
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
        scaling = 1 / 25
        tt = np.linspace(0, 6 * np.pi, 1000)
        f, f_dot = hypotrochoid(scaling)

        self.path_viz.visualize_path(
            [
                f(t, 0.1) + np.array([0.5, 0.0, 0.0])
                for t in np.linspace(0, 6 * np.pi, 500)
            ],
            "rob2_base_link",
        )

        setpoint = RobotSetpoint()
        setpoint.pose.aiming = Vector3(0.0, 0.0, 1.0)

        rate = rospy.Rate(100)
        for t in tt:
            ft = f(t, 0.1)
            f_dott = f_dot(t)

            setpoint.pose.position = Vector3(ft[0] + 0.5, ft[1], ft[2])
            setpoint.velocity = Vector3(f_dott[0], f_dott[1], f_dott[2])
            self.task_space_setpoint_pub.publish(setpoint)
            rate.sleep()

        self.path_viz.reset()

    def coordinated_hypotrochoid(self):
        scaling = 1 / 25
        tt = np.linspace(np.pi, 7 * np.pi, 1500)
        f, f_dot = hypotrochoid(scaling)

        self.path_viz.visualize_path(
            [f(t, 0.01) for t in np.linspace(np.pi, 7 * np.pi, 500)],
            "positioner",
        )

        setpoint = RobotSetpoint()
        setpoint.pose.aiming = Vector3(0.0, 0.0, 1.0)

        rate = rospy.Rate(100)
        for t in tt:
            ft = f(t, 0.01)
            f_dott = f_dot(t)

            setpoint.pose.position = Vector3(ft[0], ft[1], ft[2])
            setpoint.velocity = Vector3(f_dott[0], f_dott[1], f_dott[2])
            self.coordinated_setpoint_pub.publish(setpoint)
            rate.sleep()

        self.path_viz.reset()

    def move_home(self):
        home = [
            -0.3682676987777704,
            -0.845456854726745,
            1.8581698861744447,
            0.5580733655537403,
            1.5707974618414442,
            0.18855996186926843,
        ]
        self.move_joint(home, 1.0)

    def move_joint(self, joint_goal, duration):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = [
            "rob2_joint1",
            "rob2_joint2",
            "rob2_joint3",
            "rob2_joint4",
            "rob2_joint5",
            "rob2_joint6",
        ]
        point = JointTrajectoryPoint()
        point.positions = joint_goal
        point.time_from_start = rospy.Duration(duration)
        goal.trajectory.points = [point]

        self.joint_traj_client.send_goal(goal)
        self.joint_traj_client.wait_for_result()


if __name__ == "__main__":
    rospy.init_node("coordinated_motion_demo_rob2")

    try:
        demo = CoordinatedMotionDemo()
        demo.run()
    except rospy.ROSInterruptException:
        pass

import numpy as np
import rospy
import actionlib

from geometry_msgs.msg import Point, Vector3, Quaternion
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest

from coordinated_control_msgs.msg import TwistDecompositionSetpoint
from coordinated_control_msgs.srv import QueryPose

NS = "rob1"
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


class ControllerManagerClient(object):
    def __init__(self):
        rospy.wait_for_service("controller_manager/switch_controller")
        self.switch_controller_client = rospy.ServiceProxy(
            "controller_manager/switch_controller", SwitchController
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


class TwistDecompositionDemo:
    def __init__(self):
        self.joint_controller_name = "joint_trajectory_controller"
        self.base_frame_controller_name = "twist_decomposition_controller"

        self.home = [
            -0.368,
            -0.845,
            1.858,
            0.558,
            1.571,
            0.189,
        ]
        self.joint_names = [
            f"{NS}_joint1",
            f"{NS}_joint2",
            f"{NS}_joint3",
            f"{NS}_joint4",
            f"{NS}_joint5",
            f"{NS}_joint6",
        ]

        self.setpoint_pub = rospy.Publisher(
            f"{self.base_frame_controller_name}/setpoint",
            TwistDecompositionSetpoint,
            latch=True,
            queue_size=1,
        )
        self._pose_client = rospy.ServiceProxy(
            self.base_frame_controller_name + "/query_pose", QueryPose
        )

        print("waiting for follow_joint_trajectory")
        self._joint_traj_client = actionlib.SimpleActionClient(
            f"{self.joint_controller_name}/follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._joint_traj_client.wait_for_server()
        self._controller_client = ControllerManagerClient()

    def run(self):
        self.start_joint_control()
        self.move_joint(self.home, 1.0)

        self.start_base_frame_control()
        self.base_frame_hypotrochoid()

        self.start_joint_control()
        self.move_joint(self.home, 1.0)

    def base_frame_hypotrochoid(self):
        scaling = 1 / 25
        offset = np.array([0.5, 0.0, 0.1])
        tt = np.linspace(0, 6 * np.pi, 1000)
        f, f_dot = hypotrochoid(scaling)

        rate = rospy.Rate(100)
        setpoint = TwistDecompositionSetpoint()
        setpoint.pose.orientation = Quaternion(0.0, 1.0, 0.0, 0.0)

        # travel to start
        pose = self.get_pose()
        current_position = np.array([pose.position.x, pose.position.y, pose.position.z])
        init_path_p = f(tt[0]) + offset

        g, g_dot, dur = linear_path(current_position, init_path_p, LINEAR_VELOCITY)
        for t in np.linspace(0, 1, int(dur * 100)):
            gt = g(t)
            g_dott = g_dot(t)
            setpoint.pose.position = Point(gt[0], gt[1], gt[2])
            setpoint.velocity = Vector3(g_dott[0], g_dott[1], g_dott[2])
            self.setpoint_pub.publish(setpoint)
            rate.sleep()

        # follow path
        for t in tt:
            ft = f(t) + offset
            f_dott = f_dot(t)

            setpoint.pose.position = Point(ft[0], ft[1], ft[2])
            setpoint.velocity = Vector3(f_dott[0], f_dott[1], f_dott[2])
            self.setpoint_pub.publish(setpoint)
            rate.sleep()

    def move_joint(self, joint_goal, duration):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = joint_goal
        point.time_from_start = rospy.Duration(duration)
        goal.trajectory.points = [point]

        self._joint_traj_client.send_goal(goal)
        self._joint_traj_client.wait_for_result()

    def start_joint_control(self):
        self._controller_client.switch_controller(
            [self.joint_controller_name], [self.base_frame_controller_name]
        )

    def start_base_frame_control(self):
        self._controller_client.switch_controller(
            [self.base_frame_controller_name],
            [self.joint_controller_name],
        )

    def get_pose(self):
        try:
            resp = self._pose_client.call()
            return resp.pose
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")


if __name__ == "__main__":
    rospy.init_node("twist_decomposition_client")

    try:
        demo = TwistDecompositionDemo()
        demo.run()
    except rospy.ROSInterruptException:
        pass

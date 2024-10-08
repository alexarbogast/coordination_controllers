import numpy as np
import rospy

from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, Vector3, Quaternion
from coordinated_control_msgs.msg import AxiallySymmetricSetpoint

from .controller_client import ControllerClient, JointControllerClient
from .controller_manager_client import ControllerManagerClient
from .path_visualization import PathVisualization
from .path import linear_path

LINEAR_VELOCITY = 0.300


class ControlDemo(object):
    def __init__(self, setpoint_hz=1000):
        controller_name = rospy.get_param("~controller")
        joint_controller_name = rospy.get_param("~joint_controller")
        self.arm_id = rospy.get_param("~arm_id")

        self.controller_client = ControllerClient(controller_name)
        self.joint_controller_client = JointControllerClient(joint_controller_name)

        self.controller_manager_client = ControllerManagerClient()
        self.path_viz = PathVisualization(
            0.007, ColorRGBA(0.96, 0.38, 0.21, 1.0), self.arm_id  # type: ignore
        )

        self.hz = setpoint_hz

    def execute_linear_path(self, start, end, vel):
        rate = rospy.Rate(self.hz)
        setpoint = AxiallySymmetricSetpoint()
        setpoint.pose.orientation = Quaternion(0.0, 1.0, 0.0, 0.0)

        f, f_dot, dur = linear_path(start, end, vel)
        for t in np.linspace(0, 1, int(dur * self.hz)):
            ft = f(t)
            f_dott = f_dot(t)
            setpoint.pose.position = Point(ft[0], ft[1], ft[2])
            setpoint.velocity = Vector3(f_dott[0], f_dott[1], f_dott[2])
            self.controller_client.publish_setpoint(setpoint)
            rate.sleep()

    def execute_path(self, f, f_dot, offset, tt):
        rate = rospy.Rate(self.hz)
        setpoint = AxiallySymmetricSetpoint()
        setpoint.pose.orientation = Quaternion(0.0, 1.0, 0.0, 0.0)

        # travel to start
        current_position = self.get_position()
        if current_position is None:
            return
        self.execute_linear_path(current_position, f(tt[0]) + offset, LINEAR_VELOCITY)

        # follow path
        for t in tt:
            ft = f(t) + offset
            f_dott = f_dot(t)

            setpoint.pose.position = Point(ft[0], ft[1], ft[2])
            setpoint.velocity = Vector3(f_dott[0], f_dott[1], f_dott[2])
            self.controller_client.publish_setpoint(setpoint)
            rate.sleep()

    def get_pose(self):
        return self.controller_client.get_pose()

    def get_position(self):
        pose = self.get_pose()
        if pose is None:
            return None
        return np.array([pose.position.x, pose.position.y, pose.position.z])

    def start_joint_control(self):
        self.controller_manager_client.switch_controller(
            [self.joint_controller_client.name], [self.controller_client.name]
        )

    def start_taskspace_control(self):
        self.controller_manager_client.switch_controller(
            [self.controller_client.name], [self.joint_controller_client.name]
        )

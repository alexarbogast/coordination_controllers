import actionlib
import rospy

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from coordinated_control_msgs.msg import RobotSetpoint


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


class CoordinatedMotionClient(object):
    def __init__(self):
        self.joint_controller_name = rospy.get_param("~joint_space_controller")
        self.task_space_controller_name = rospy.get_param("~task_space_controller")
        self.coordination_controller_name = rospy.get_param("~coordination_controller")

        self.joint_names = rospy.get_param(self.joint_controller_name + "/joints")

        # setpoint publishers
        task_space_setpoint_topic = rospy.get_param(
            self.task_space_controller_name + "/setpoint_topic"
        )
        self._task_space_setpoint_pub = rospy.Publisher(
            self.task_space_controller_name + "/" + task_space_setpoint_topic,
            RobotSetpoint,
            latch=True,
            queue_size=1,
        )

        coord_setpoint_topic = rospy.get_param(
            self.coordination_controller_name + "/setpoint_topic"
        )
        self._coordinated_setpoint_pub = rospy.Publisher(
            self.coordination_controller_name + "/" + coord_setpoint_topic,
            RobotSetpoint,
            latch=True,
            queue_size=1,
        )

        self._joint_traj_client = actionlib.SimpleActionClient(
            self.joint_controller_name + "/follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._joint_traj_client.wait_for_server()

        # current controllers
        self._controller_manager_client = ControllerManagerClient()
        self._active_setpoint_pub = self._task_space_setpoint_pub

    def move_joint(self, joint_goal, duration):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = joint_goal
        point.time_from_start = rospy.Duration(duration)
        goal.trajectory.points = [point]

        self._joint_traj_client.send_goal(goal)
        self._joint_traj_client.wait_for_result()

    def publish_setpoint(self, setpoint: RobotSetpoint):
        self._active_setpoint_pub.publish(setpoint)

    def start_joint_control(self):
        self._controller_manager_client.switch_controller(
            [self.joint_controller_name],
            [self.task_space_controller_name, self.coordination_controller_name],
        )

    def start_task_space_control(self):
        self._controller_manager_client.switch_controller(
            [self.task_space_controller_name],
            [self.joint_controller_name, self.coordination_controller_name],
        )
        self._active_setpoint_pub = self._task_space_setpoint_pub

    def start_coordinated_control(self):
        self._controller_manager_client.switch_controller(
            [self.coordination_controller_name],
            [self.joint_controller_name, self.task_space_controller_name],
        )
        self._active_setpoint_pub = self._coordinated_setpoint_pub

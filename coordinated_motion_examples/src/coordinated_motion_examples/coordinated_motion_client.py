import actionlib
import rospy

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from coordinated_control_msgs.msg import RobotSetpoint
from coordinated_control_msgs.srv import QueryPose


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
        self.base_frame_controller_name = rospy.get_param("~base_frame_controller")
        self.coordinated_controller_name = rospy.get_param("~coordinated_controller")

        self.joint_names = rospy.get_param(self.joint_controller_name + "/joints")

        # base frame controller
        base_frame_setpoint_topic = rospy.get_param(
            self.base_frame_controller_name + "/setpoint_topic"
        )
        self._base_frame_setpoint_pub = rospy.Publisher(
            self.base_frame_controller_name + "/" + base_frame_setpoint_topic,
            RobotSetpoint,
            latch=True,
            queue_size=1,
        )
        self._base_frame_pose_client = rospy.ServiceProxy(
            self.base_frame_controller_name + "/query_pose", QueryPose
        )

        # coordinated controller
        coord_setpoint_topic = rospy.get_param(
            self.coordinated_controller_name + "/setpoint_topic"
        )
        self._coordinated_setpoint_pub = rospy.Publisher(
            self.coordinated_controller_name + "/" + coord_setpoint_topic,
            RobotSetpoint,
            latch=True,
            queue_size=1,
        )
        self._coordinated_pose_client = rospy.ServiceProxy(
            self.coordinated_controller_name + "/query_pose", QueryPose
        )

        # joint space controller
        self._joint_traj_client = actionlib.SimpleActionClient(
            self.joint_controller_name + "/follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._joint_traj_client.wait_for_server()

        # active controllers
        self._controller_manager_client = ControllerManagerClient()
        self._active_setpoint_pub = self._base_frame_setpoint_pub
        self._active_pose_client = self._base_frame_pose_client

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
            [self.base_frame_controller_name, self.coordinated_controller_name],
        )
        self._active_setpoint_pub = None
        self._active_pose_client = None

    def start_base_frame_control(self):
        self._controller_manager_client.switch_controller(
            [self.base_frame_controller_name],
            [self.joint_controller_name, self.coordinated_controller_name],
        )
        self._active_setpoint_pub = self._base_frame_setpoint_pub
        self._active_pose_client = self._base_frame_pose_client

    def start_coordinated_control(self):
        self._controller_manager_client.switch_controller(
            [self.coordinated_controller_name],
            [self.joint_controller_name, self.base_frame_controller_name],
        )
        self._active_setpoint_pub = self._coordinated_setpoint_pub
        self._active_pose_client = self._coordinated_pose_client

    def get_pose(self):
        try:
            resp = self._active_pose_client.call()
            return resp.pose
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

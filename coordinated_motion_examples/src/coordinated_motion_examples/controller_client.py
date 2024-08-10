import rospy
import actionlib

from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectoryPoint

from coordinated_control_msgs.msg import RobotSetpoint
from coordinated_control_msgs.srv import QueryPose


class ControllerClient:
    def __init__(self, name):
        self.name = name

        setpoint_topic = rospy.get_param(self.name + "/setpoint_topic")
        self.setpoint_pub = rospy.Publisher(
            f"{self.name}/{setpoint_topic}",
            RobotSetpoint,
            latch=True,
            queue_size=1,
        )
        self.pose_client = rospy.ServiceProxy(f"{self.name}/query_pose", QueryPose)
        rospy.loginfo(f"Connected to controller: {self.name}")

    def publish_setpoint(self, setpoint: RobotSetpoint):
        self.setpoint_pub.publish(setpoint)

    def get_pose(self):
        try:
            resp = self.pose_client.call()
            return resp.pose
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")


class JointControllerClient:
    def __init__(self, name):
        self.name = name

        self.joint_names = rospy.get_param(self.name + "/joints")
        self.joint_traj_client = actionlib.SimpleActionClient(
            f"{self.name}/follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        rospy.loginfo("Waiting for follow_joint_trajectory action")
        self.joint_traj_client.wait_for_server()

    def move_joint(self, joint_goal, duration):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = joint_goal
        point.time_from_start = rospy.Duration(duration)
        goal.trajectory.points = [point]

        self.joint_traj_client.send_goal(goal)
        self.joint_traj_client.wait_for_result()

import numpy as np
import rospy

# from std_msgs.msg import ColorRGBA
# from geometry_msgs.msg import Point, Vector3, Quaternion
# from coordinated_control_msgs.msg import AxiallySymmetricSetpoint
#
# from .controller_client import ControllerClient, JointControllerClient
# from .controller_manager_client import ControllerManagerClient
# from .path_visualization import PathVisualization

from geometry_msgs.msg import Quaternion

from taskspace_control_examples import ControlDemo
from taskspace_control_examples.trajectory import *


class CoordinatedControlDemo(ControlDemo):
    def __init__(self, setpoint_hz=1000):
        super(CoordinatedControlDemo, self).__init__(setpoint_hz)
        self.arm_id = rospy.get_param("~arm_id")
        self.static_orient = Quaternion(0.0, 0.0, 0.0, 1.0)

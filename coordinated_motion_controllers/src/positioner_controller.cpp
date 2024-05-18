#include <coordinated_motion_controllers/positioner_controller.h>

#include <pluginlib/class_list_macros.h>

namespace coordinated_motion_controllers
{

const std::string JOINT_STATES_TOPIC = "/joint_states";

CoordinatedRobotData::CoordinatedRobotData(
    const std::string& controller_ns, const std::vector<std::string>& joints)
  : controller_ns(controller_ns), joints(joints), n_joints(joints.size())
{
  positions.resize(n_joints);
  velocities.resize(n_joints);
}

bool CoordinatedRobotData::init(const sensor_msgs::JointState& joint_state)
{
  size_t n_joints = joints.size();
  joint_state_indices.resize(n_joints);

  for (size_t i = 0; i < n_joints; ++i)
  {
    std::string j = joint_state.name[i];
    auto it = find(joint_state.name.begin(), joint_state.name.end(), j);
    if (it == joint_state.name.end())
    {
      return false;
    }

    size_t index = it - joint_state.name.begin();
    joint_state_indices[i] = index;
  }
  return true;
}

bool PositionerController::init(hardware_interface::PositionJointInterface* hw,
                                ros::NodeHandle& nh)
{
  std::string ns = nh.getNamespace();
  std::vector<std::string> joint_names;
  if (!nh.getParam("joints", joint_names))
  {
    ROS_ERROR_STREAM("Failed to load " << ns << "/joints"
                                       << " from parameter server");
    return false;
  }

  // store namespaces of controllers coordinated with the positioner
  std::vector<std::string> coordinated_controllers_ns;
  if (!nh.getParam("coordinated_controllers", coordinated_controllers_ns))
  {
    ROS_ERROR_STREAM("Failed to load" << ns << "/coordinated_controllers"
                                      << " from the parameter server");
    return false;
  }

  // wait for a joints states topic and find the indices of values for each
  // coordinated robot
  sensor_msgs::JointStateConstPtr joint_state =
      ros::topic::waitForMessage<sensor_msgs::JointState>(JOINT_STATES_TOPIC,
                                                          ros::Duration(10));
  if (joint_state == NULL)
  {
    ROS_ERROR_STREAM("Timed out waiting for topic:" << JOINT_STATES_TOPIC);
    return false;
  }

  for (const std::string& ns : coordinated_controllers_ns)
  {
    std::string param = ns + "/joints";
    std::vector<std::string> controller_joints;
    if (!nh.getParam(param, controller_joints))
    {
      ROS_ERROR_STREAM("Failed to load" << param
                                        << " from the parameter server");
      return false;
    }
    robot_data_[ns] =
        std::make_shared<CoordinatedRobotData>(ns, controller_joints);
    robot_data_[ns]->init(*joint_state);
  }

  // Get joint handles from hardware interface
  joint_handles_.resize(joint_names.size());
  for (size_t i = 0; i < joint_names.size(); ++i)
  {
    try
    {
      joint_handles_[i] = hw->getHandle(joint_names[i]);
    }
    catch (const hardware_interface::HardwareInterfaceException& e)
    {
      ROS_ERROR_STREAM("Exception getting joint handles from hw interface"
                       << e.what());
      return false;
    }
  }

  // Setup ROS components
  sub_joint_states_ = nh.subscribe(
      "/joint_states", 1, &PositionerController::jointStateCallback, this);

  return true;
}

void PositionerController::update(const ros::Time& time, const ros::Duration& period)
{
  for (auto& handle : joint_handles_)
  {
    double old = handle.getPosition();
    //handle.setCommand(old + 0.0005);
    handle.setCommand(sin(time.toSec()));
  }
}

void PositionerController::starting(const ros::Time&) {}
void PositionerController::stopping(const ros::Time&) {}

void PositionerController::jointStateCallback(
    const sensor_msgs::JointStateConstPtr& msg)
{
  for (auto robot : robot_data_)
  {
    for (size_t i = 0; i < robot.second->n_joints; ++i)
    {
      robot.second->positions(i) =
          msg->position[robot.second->joint_state_indices[i]];
    }
  }

  // TODO: update robot jacobians etc.
}

}  // namespace coordinated_motion_controllers

PLUGINLIB_EXPORT_CLASS(coordinated_motion_controllers::PositionerController,
                       controller_interface::ControllerBase)

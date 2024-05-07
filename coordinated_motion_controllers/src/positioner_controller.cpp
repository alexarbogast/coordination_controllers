#include <coordinated_motion_controllers/positioner_controller.h>
#include <pluginlib/class_list_macros.h>

namespace coordinated_motion_controllers
{

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

  return true;
}

void PositionerController::update(const ros::Time&, const ros::Duration& period)
{
  for (auto& handle : joint_handles_)
  {
    double old = handle.getPosition();
    std::cout << old << std::endl;
    handle.setCommand(old + 0.001);
  }
}

void PositionerController::starting(const ros::Time&) {}
void PositionerController::stopping(const ros::Time&) {}

}  // namespace coordinated_motion_controllers

PLUGINLIB_EXPORT_CLASS(coordinated_motion_controllers::PositionerController,
                       controller_interface::ControllerBase)

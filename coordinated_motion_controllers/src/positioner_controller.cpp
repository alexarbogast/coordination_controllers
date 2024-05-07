#include <coordinated_motion_controllers/positioner_controller.h>
#include <pluginlib/class_list_macros.h>

namespace coordinated_motion_controllers
{

bool PositionerController::init(hardware_interface::RobotHW* robot_hardware,
                                ros::NodeHandle& nh)
{
  return true;
}

void PositionerController::update(const ros::Time&, const ros::Duration& period)
{
}
void PositionerController::starting(const ros::Time&) {}
void PositionerController::stopping(const ros::Time&) {}

}  // namespace coordinated_motion_controllers

PLUGINLIB_EXPORT_CLASS(coordinated_motion_controllers::PositionerController,
                       controller_interface::ControllerBase)

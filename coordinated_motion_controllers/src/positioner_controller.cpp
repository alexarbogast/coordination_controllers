#include <coordinated_motion_controllers/positioner_controller.h>

namespace coordinated_motion_controllers
{

bool PositionerController::init(hardware_interface::RobotHW* robot_hardware,
                                ros::NodeHandle& nh)
{
  return true;
}

void PositionerController::starting(const ros::Time&) {}
void PositionerController::stopping(const ros::Time&) {}

}  // namespace coordinated_motion_controllers


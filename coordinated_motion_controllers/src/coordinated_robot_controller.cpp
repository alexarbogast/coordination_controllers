#include <coordinated_motion_controllers/coordinated_robot_controller.h>

namespace coordinated_motion_controllers
{

bool CoordinatedRobotController::init(
    hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& nh)
{
  return true;
}

void CoordinatedRobotController::starting(const ros::Time&) {}
void CoordinatedRobotController::stopping(const ros::Time&) {}

}  // namespace coordinated_motion_controllers


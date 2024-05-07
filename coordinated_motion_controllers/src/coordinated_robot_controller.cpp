#include <coordinated_motion_controllers/coordinated_robot_controller.h>
#include <pluginlib/class_list_macros.h>

namespace coordinated_motion_controllers
{

bool CoordinatedRobotController::init(
    hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& nh)
{
  return true;
}

void CoordinatedRobotController::update(const ros::Time&,
                                        const ros::Duration& period)
{
}

void CoordinatedRobotController::starting(const ros::Time&) {}
void CoordinatedRobotController::stopping(const ros::Time&) {}

}  // namespace coordinated_motion_controllers
//

PLUGINLIB_EXPORT_CLASS(
    coordinated_motion_controllers::CoordinatedRobotController,
    controller_interface::ControllerBase)

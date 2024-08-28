// Copyright 2024 Alex Arbogast
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <coordinated_motion_controllers/positioner_controller.h>

#include <pluginlib/class_list_macros.h>

namespace coordinated_motion_controllers
{

static const std::string POS_SETPOINT_NS = "pos_setpoint";

CoordinatedRobotData::CoordinatedRobotData(const std::string& controller_ns,
                                           unsigned int n_pos_joints)
  : controller_ns_(controller_ns), n_pos_joints_(n_pos_joints), coord(false)
{
  KDL::JntArray init_setpoint(n_pos_joints);
  rec_setpoint.writeFromNonRT(init_setpoint);

  pnh_ = ros::NodeHandle(controller_ns_);
  sub_rec_setpoint_ = pnh_.subscribe(
      POS_SETPOINT_NS, 1, &CoordinatedRobotData::setpointCallback, this);
}

void CoordinatedRobotData::setpointCallback(
    const coordinated_control_msgs::PositionerSetpointConstPtr& msg)
{
  coord.store(msg->coordinated);
  KDL::JntArray setpoint(n_pos_joints_);
  for (size_t i = 0; i < n_pos_joints_; ++i)
  {
    setpoint(i) = msg->velocity[i];
  }
  rec_setpoint.writeFromNonRT(setpoint);
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
  n_joints_ = joint_names.size();

  // store namespaces of controllers coordinated with the positioner
  std::vector<std::string> coordinated_controllers_ns;
  if (!nh.getParam("coordinated_controllers", coordinated_controllers_ns))
  {
    ROS_ERROR_STREAM("Failed to load" << ns << "/coordinated_controllers"
                                      << " from the parameter server");
    return false;
  }

  KDL::JntArray init_setpoint(n_joints_);
  for (const std::string& ns : coordinated_controllers_ns)
  {
    robot_data_[ns] = std::make_shared<CoordinatedRobotData>(ns, n_joints_);
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

  return true;
}

void PositionerController::update(const ros::Time& time,
                                  const ros::Duration& period)
{
  Eigen::VectorXd cmd = Eigen::VectorXd::Zero(n_joints_);
  int n_coord = 0;
  for (auto& robot : robot_data_)
  {
    if (!robot.second->coord)
       continue;

    auto robot_rec_cmd = robot.second->rec_setpoint.readFromRT();
    cmd += robot_rec_cmd->data;
    n_coord++;
  }

  if (n_coord)
  {
    cmd /= n_coord;
  }

  //auto new_position = joint_h
  double dt = period.toSec();
  for (unsigned int i = 0; i < n_joints_; ++i)
  {
    auto new_position = joint_handles_[i].getPosition() + (cmd[i] * dt);
    joint_handles_[i].setCommand(new_position);
  }
}

void PositionerController::starting(const ros::Time&) {}
void PositionerController::stopping(const ros::Time&) {}

}  // namespace coordinated_motion_controllers

PLUGINLIB_EXPORT_CLASS(coordinated_motion_controllers::PositionerController,
                       controller_interface::ControllerBase)

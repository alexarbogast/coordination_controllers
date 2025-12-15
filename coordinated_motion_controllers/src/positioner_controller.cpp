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

#include <coordinated_motion_controllers/positioner_controller.hpp>
#include <axially_symmetric_controllers/utility.hpp>

#include <controller_interface/helpers.hpp>

namespace coordinated_motion_controllers
{

static const std::string POS_SETPOINT_NS = "pos_setpoint";

// =============================================================================
// CoordinatedRobotData
// =============================================================================

CoordinatedRobotData::CoordinatedRobotData(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
    const std::string& controller_ns, unsigned int n_pos_joints)
  : node_(node)
  , controller_ns_(controller_ns)
  , n_pos_joints_(n_pos_joints)
  , coord(false)
{
  KDL::JntArray init_setpoint(n_pos_joints);
  rec_setpoint.writeFromNonRT(init_setpoint);

  // Create subscription with full topic path
  std::string topic = "/" + controller_ns_ + "/" + POS_SETPOINT_NS;
  sub_rec_setpoint_ = node_->create_subscription<
      coordinated_control_msgs::msg::PositionerSetpoint>(
      topic, rclcpp::SystemDefaultsQoS(),
      std::bind(&CoordinatedRobotData::setpointCallback, this,
                std::placeholders::_1));
}

void CoordinatedRobotData::setpointCallback(
    const coordinated_control_msgs::msg::PositionerSetpoint::SharedPtr msg)
{
  coord.store(msg->coordinated);
  KDL::JntArray setpoint(n_pos_joints_);
  for (size_t i = 0; i < n_pos_joints_; ++i)
  {
    setpoint(i) = msg->velocity[i];
  }
  rec_setpoint.writeFromNonRT(setpoint);
}

// =============================================================================
// PositionerController
// =============================================================================

controller_interface::InterfaceConfiguration
PositionerController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names.reserve(params_.joints.size() * params_.command_interfaces.size());
  for (const auto& type : params_.command_interfaces)
  {
    for (const auto& joint : params_.joints)
    {
      cfg.names.push_back(joint + std::string("/").append(type));
    }
  }
  return cfg;
}

controller_interface::InterfaceConfiguration
PositionerController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // use only position feedback for now
  const std::string interface = "position";
  for (const auto& joint : params_.joints)
  {
    cfg.names.push_back(joint + std::string("/").append(interface));
  }
  return cfg;
}

controller_interface::CallbackReturn PositionerController::on_init()
{
  try
  {
    param_listener_ =
        std::make_shared<positioner_controller::ParamListener>(get_node());
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during on_init: %s",
                 e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PositionerController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  auto node = get_node();
  RCLCPP_INFO(node->get_logger(), "Configuring PositionerController...");

  params_ = param_listener_->get_params();

  n_joints_ = params_.joints.size();

  // allocate dynamic memory
  last_reference_.resize(n_joints_);
  last_commanded_ = last_reference_;
  joint_state_ = last_reference_;

  if (params_.command_interfaces.empty())
  {
    RCLCPP_ERROR(node->get_logger(),
                 "'command_interfaces' parameter was "
                 "empty");
    return controller_interface::CallbackReturn::FAILURE;
  }

  has_position_command_interface_ = ctrl::contains_interface_type(
      params_.command_interfaces, hardware_interface::HW_IF_POSITION);
  has_velocity_command_interface_ = ctrl::contains_interface_type(
      params_.command_interfaces, hardware_interface::HW_IF_VELOCITY);

  // Create robot data objects for each coordinated controller
  for (const std::string& ns : params_.coordinated_controllers)
  {
    robot_data_[ns] =
        std::make_shared<CoordinatedRobotData>(get_node(), ns, n_joints_);
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PositionerController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  auto logger = get_node()->get_logger();
  RCLCPP_INFO(logger, "Activating PositionerController...");

  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();

  RCLCPP_INFO(logger, "Activated PositionerController");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PositionerController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type PositionerController::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
{
  read_state_from_hardware(joint_state_);

  ctrl::VectorND cmd_vel = ctrl::VectorND::Zero(n_joints_);
  int n_coord = 0;

  for (auto& robot : robot_data_)
  {
    if (!robot.second->coord)
      continue;

    auto robot_rec_cmd = robot.second->rec_setpoint.readFromRT();
    cmd_vel += robot_rec_cmd->data;
    n_coord++;
  }

  if (n_coord)
  {
    cmd_vel /= n_coord;
  }

  ctrl::VectorND new_position =
      joint_state_.q.data + (cmd_vel * period.seconds());

  auto cmd = ctrl::transformEigenToKDL(new_position, cmd_vel);
  write_command(cmd);

  return controller_interface::return_type::OK;
}

void PositionerController::read_state_from_hardware(KDL::JntArrayVel& state)
{
  bool nan_position = false;
  size_t pos_ind = 0;
  for (size_t joint_ind = 0; joint_ind < n_joints_; ++joint_ind)
  {
    state.q(joint_ind) =
        state_interfaces_[pos_ind * n_joints_ + joint_ind].get_value();
    nan_position |= std::isnan(state.q(joint_ind));
  }

  if (nan_position)
  {
    state.q = last_commanded_.q;
  }
}

void PositionerController::write_command(const KDL::JntArrayVel& cmd)
{
  size_t pos_ind = 0;
  size_t vel_ind = (has_position_command_interface_) ?
                       pos_ind + has_velocity_command_interface_ :
                       pos_ind;
  for (size_t joint_ind = 0; joint_ind < n_joints_; ++joint_ind)
  {
    if (has_position_command_interface_)
    {
      command_interfaces_[pos_ind * n_joints_ + joint_ind].set_value(
          cmd.q(joint_ind));
    }
    if (has_velocity_command_interface_)
    {
      command_interfaces_[vel_ind * n_joints_ + joint_ind].set_value(
          cmd.qdot(joint_ind));
    }
  }
  last_commanded_ = cmd;
}

}  // namespace coordinated_motion_controllers

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(coordinated_motion_controllers::PositionerController,
                       controller_interface::ControllerInterface)

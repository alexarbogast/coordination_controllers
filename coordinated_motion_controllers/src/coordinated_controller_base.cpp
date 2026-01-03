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

#include <controller_interface/controller_interface_base.hpp>
#include <coordinated_motion_controllers/coordinated_controller_base.hpp>
#include <controller_interface/helpers.hpp>
#include <axially_symmetric_controllers/utility.hpp>

#include <urdf/model.h>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <memory>

// positioner state interfaces
#include <coordinated_motion_controllers/positioner_state_interface/topic_state_interface.hpp>
#include <coordinated_motion_controllers/positioner_state_interface/loaned_state_interface.hpp>

namespace coordinated_motion_controllers
{

static const std::string POS_SETPOINT_NS = "~/pos_setpoint";
using namespace std::chrono_literals;

controller_interface::InterfaceConfiguration
CoordinatedControllerBase::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names.reserve(n_robot_joints_ * params_.command_interfaces.size());
  for (const auto& type : params_.command_interfaces)
  {
    for (const auto& joint : robot_joint_names_)
    {
      cfg.names.push_back(joint + std::string("/").append(type));
    }
  }
  return cfg;
}

controller_interface::InterfaceConfiguration
CoordinatedControllerBase::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // use only position feedback for now
  for (const auto& joint : robot_joint_names_)
  {
    cfg.names.push_back(
        joint + std::string("/").append(hardware_interface::HW_IF_POSITION));
  }

  // handle the positioner_state_interface == loaned case
  if (params_.state_interface == "loaned")
  {
    for (const auto& joint : positioner_joint_names_)
    {
      cfg.names.push_back(
          joint + std::string("/").append(hardware_interface::HW_IF_POSITION));
      cfg.names.push_back(
          joint + std::string("/").append(hardware_interface::HW_IF_VELOCITY));
    }
  }
  return cfg;
}

controller_interface::CallbackReturn CoordinatedControllerBase::on_init()
{
  try
  {
    param_listener_ =
        std::make_shared<coordinated_controller_base::ParamListener>(
            get_node());
  }
  catch (const std::exception& e)
  {
    fprintf(stderr,
            "Exception thrown during controller's init with message: %s \n",
            e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  auto_declare<std::string>("robot_description", "");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CoordinatedControllerBase::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  auto logger = get_node()->get_logger();
  params_ = param_listener_->get_params();

  std::string urdf_xml;
#ifdef COORDINATED_CONTROLLERS_JAZZY
  urdf_xml = this->get_robot_description();
#else
  urdf_xml = get_node()->get_parameter("robot_description").as_string();
#endif

  if (urdf_xml.empty())
  {
    RCLCPP_ERROR(logger, "robot_description is empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  positioner_link_ = params_.positioner_link;
  base_link_ = params_.base_link;
  eef_link_ = params_.eef_link;

  // parse URDF -> KDL
  urdf::Model urdf_model;
  KDL::Tree kdl_tree;
  if (!urdf_model.initString(urdf_xml))
  {
    RCLCPP_ERROR(logger,
                 "Failed to parse URDF from 'robot_description' parameter.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree))
  {
    RCLCPP_FATAL(logger, "Failed to convert URDF to KDL tree.");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!kdl_tree.getChain(base_link_, eef_link_, robot_chain_))
  {
    RCLCPP_FATAL(logger, "Failed to build KDL chain from '%s' to '%s'.",
                 base_link_.c_str(), eef_link_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!kdl_tree.getChain(positioner_link_, eef_link_, coordinated_chain_))
  {
    RCLCPP_FATAL(logger, "Failed to build KDL chain from '%s' to '%s'.",
                 positioner_link_.c_str(), eef_link_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  robot_joint_names_ = ctrl::joints_along_chain(robot_chain_);
  n_robot_joints_ = robot_joint_names_.size();

  // temporary chain to find the positioner joints
  KDL::Chain pos_chain;
  if (!kdl_tree.getChain(base_link_, positioner_link_, pos_chain))
  {
    RCLCPP_FATAL(logger, "Failed to build KDL chain from '%s' to '%s'.",
                 positioner_link_.c_str(), base_link_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  std::vector<std::string> coordinated_joint_names =
      ctrl::joints_along_chain(coordinated_chain_);
  auto n_coordinated_joints = coordinated_joint_names.size();

  positioner_joint_names_ = ctrl::joints_along_chain(pos_chain);
  n_pos_joints_ = positioner_joint_names_.size();

  // allocate dynamic memory
  last_reference_.resize(n_robot_joints_);
  last_commanded_ = last_reference_;
  joint_state_ = last_reference_;

  if (params_.command_interfaces.empty())
  {
    RCLCPP_ERROR(logger, "'command_interfaces' parameter was empty");
    return controller_interface::CallbackReturn::FAILURE;
  }

  has_position_command_interface_ = ctrl::contains_interface_type(
      params_.command_interfaces, hardware_interface::HW_IF_POSITION);
  has_velocity_command_interface_ = ctrl::contains_interface_type(
      params_.command_interfaces, hardware_interface::HW_IF_VELOCITY);

  coordinated_fk_solver_ =
      std::make_unique<KDL::ChainFkSolverPos_recursive>(coordinated_chain_);

  upper_pos_limits_.resize(n_robot_joints_);
  lower_pos_limits_.resize(n_robot_joints_);
  for (size_t i = 0; i < n_robot_joints_; ++i)
  {
    const auto& jn = robot_joint_names_[i];
    auto j = urdf_model.getJoint(jn);
    if (!j)
    {
      RCLCPP_ERROR(logger, "Joint '%s' not found in URDF.", jn.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    if (j->type == urdf::Joint::CONTINUOUS)
    {
      upper_pos_limits_(i) = std::numeric_limits<double>::quiet_NaN();
      lower_pos_limits_(i) = std::numeric_limits<double>::quiet_NaN();
    }
    else if (j->limits)
    {
      upper_pos_limits_(i) = j->limits->upper;
      lower_pos_limits_(i) = j->limits->lower;
    }
    else
    {
      RCLCPP_WARN(logger, "Joint %s has no limits; using NaN.", jn.c_str());
      upper_pos_limits_(i) = std::numeric_limits<double>::quiet_NaN();
      lower_pos_limits_(i) = std::numeric_limits<double>::quiet_NaN();
    }
  }

  // Create positioner state interface
  if (params_.state_interface == "topic")
  {
    pos_state_interface_ = std::make_unique<TopicStateInterface>();
  }
  else if (params_.state_interface == "loaned")
  {
    pos_state_interface_ = std::make_unique<LoanedStateInterface>();
  }
  else
  {
    RCLCPP_FATAL(logger, "Unknown positioner state interface.");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!pos_state_interface_->init(get_node(), positioner_joint_names_, params_))
  {
    RCLCPP_FATAL(logger, "Failed to initialize positioner state iterface");
    return controller_interface::CallbackReturn::ERROR;
  }

  try
  {
    positioner_setpoint_pub_ =
        get_node()
            ->create_publisher<
                coordinated_control_msgs::msg::PositionerSetpoint>(
                POS_SETPOINT_NS, rclcpp::SystemDefaultsQoS());
    realtime_setpoint_pub_ =
        std::make_unique<PosSetpointPublisher>(positioner_setpoint_pub_);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(logger,
                 "Failed to initialize positioner setpoint "
                 "publisher");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  positioner_setpoint_msg_.coordinated = false;
  positioner_setpoint_msg_.velocity.resize(n_pos_joints_);

  // Create service for query_pose
  query_pose_service_ = get_node()->create_service<QueryPose>(
      get_node()->get_name() + std::string("/query_pose"),
      std::bind(&CoordinatedControllerBase::queryPoseServiceCb, this,
                std::placeholders::_1, std::placeholders::_2));

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CoordinatedControllerBase::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  auto logger = get_node()->get_logger();

  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();

  if (params_.state_interface == "loaned")
  {
    auto loaned_iface =
        dynamic_cast<LoanedStateInterface*>(pos_state_interface_.get());
    if (!loaned_iface->bind(state_interfaces_))
    {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Failed to bind positioner state interfaces. "
                   "Is your positioner in the same system interface as you "
                   "robot?");
      return CallbackReturn::ERROR;
    }
  }

  positioner_setpoint_msg_.coordinated = true;

  RCLCPP_INFO(logger, "Activated CoordinatedControllerBase");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CoordinatedControllerBase::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  stop_motion();
  positioner_setpoint_msg_.coordinated = false;
  realtime_setpoint_pub_->tryPublish(positioner_setpoint_msg_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CoordinatedControllerBase::on_shutdown(
    const rclcpp_lifecycle::State& previous_state)
{
  stop_motion();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CoordinatedControllerBase::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  // derived classes will compute command vector and call write_command()
  return controller_interface::return_type::OK;
}

void CoordinatedControllerBase::read_state_from_hardware(
    KDL::JntArrayVel& state)
{
  bool nan_position = false;
  size_t pos_ind = 0;
  for (size_t joint_ind = 0; joint_ind < n_robot_joints_; ++joint_ind)
  {
    state.q(joint_ind) =
        state_interfaces_[pos_ind * n_robot_joints_ + joint_ind].get_value();
    nan_position |= std::isnan(state.q(joint_ind));
  }

  if (nan_position)
  {
    state.q = last_commanded_.q;
  }
}

void CoordinatedControllerBase::get_combined_state(KDL::JntArrayVel& state)
{
  read_state_from_hardware(joint_state_);

  KDL::JntArrayVel pos_state(n_pos_joints_);
  pos_state.resize(n_pos_joints_);
  pos_state_interface_->read(pos_state);

  state.q.data << pos_state.q.data.reverse(), joint_state_.q.data;
  state.qdot.data << pos_state.qdot.data.reverse(), joint_state_.qdot.data;
}

void CoordinatedControllerBase::write_robot_command(const KDL::JntArrayVel& cmd)
{
  size_t pos_ind = 0;
  size_t vel_ind = (has_position_command_interface_) ?
                       pos_ind + has_velocity_command_interface_ :
                       pos_ind;
  for (size_t joint_ind = 0; joint_ind < n_robot_joints_; ++joint_ind)
  {
    if (has_position_command_interface_)
    {
      command_interfaces_[pos_ind * n_robot_joints_ + joint_ind].set_value(
          cmd.q(joint_ind));
    }
    if (has_velocity_command_interface_)
    {
      command_interfaces_[vel_ind * n_robot_joints_ + joint_ind].set_value(
          cmd.qdot(joint_ind));
    }
  }
  last_commanded_ = cmd;
}

void CoordinatedControllerBase::write_positioner_command(
    const ctrl::VectorND& cmd)
{
  if (realtime_setpoint_pub_)
  {
    Eigen::VectorXd::Map(&positioner_setpoint_msg_.velocity[0], cmd.size()) =
        cmd;
    realtime_setpoint_pub_->tryPublish(positioner_setpoint_msg_);
  }
}

void CoordinatedControllerBase::stop_motion()
{
  // Stop motion for velocity control
  if (has_velocity_command_interface_)
  {
    size_t pos_ind = 0;
    size_t vel_ind = (has_position_command_interface_) ?
                         pos_ind + has_velocity_command_interface_ :
                         pos_ind;

    for (size_t joint_ind = 0; joint_ind < n_robot_joints_; ++joint_ind)
    {
      command_interfaces_[vel_ind * n_robot_joints_ + joint_ind].set_value(0.0);
    }
  }
}

bool CoordinatedControllerBase::queryPoseServiceCb(
    const std::shared_ptr<QueryPose::Request> /*req*/,
    std::shared_ptr<QueryPose::Response> resp)
{
  KDL::JntArrayVel combined_state(n_pos_joints_ + n_robot_joints_);
  get_combined_state(combined_state);

  KDL::Frame pose;
  if (!coordinated_fk_solver_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "FK solver not initialized.");
    return false;
  }
  int fk_res = coordinated_fk_solver_->JntToCart(combined_state.q, pose);
  if (fk_res < 0)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "FK solver failed.");
    return false;
  }

  resp->pose.position.x = pose.p.x();
  resp->pose.position.y = pose.p.y();
  resp->pose.position.z = pose.p.z();

  pose.M.GetQuaternion(resp->pose.orientation.x, resp->pose.orientation.y,
                       resp->pose.orientation.z, resp->pose.orientation.w);

  return true;
}

}  // namespace coordinated_motion_controllers

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
#include <coordinated_motion_controllers/pose_controller.hpp>
#include <axially_symmetric_controllers/utility.hpp>

namespace coordinated_motion_controllers
{

controller_interface::CallbackReturn PoseController::on_init()
{
  // Initialize base class
  if (CoordinatedControllerBase::on_init() !=
      controller_interface::CallbackReturn::SUCCESS)
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  // Initialize the PoseController
  try
  {
    pose_param_listener_ =
        std::make_shared<pose_controller::ParamListener>(get_node());
  }
  catch (const std::exception& e)
  {
    fprintf(stderr,
            "Exception thrown during controller's init with message: %s \n",
            e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PoseController::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  auto node = get_node();
  RCLCPP_INFO(node->get_logger(), "Configuring PoseController...");

  pose_params_ = pose_param_listener_->get_params();

  // Initialize base kinematics and joint info
  if (CoordinatedControllerBase::on_configure(previous_state) !=
      controller_interface::CallbackReturn::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize base controller.");
    return CallbackReturn::FAILURE;
  }

  coordinated_jacobian_solver_ =
      std::make_unique<KDL::ChainJntToJacSolver>(coordinated_chain_);

  // --- Setpoint subscription ---
  setpoint_subscriber_ =
      node->create_subscription<taskspace_control_msgs::msg::PoseTwistSetpoint>(
          node->get_name() + std::string("/") + pose_params_.setpoint_topic, 1,
          std::bind(&PoseController::setpointCallback, this,
                    std::placeholders::_1));

  // --- Redundancy Resolution objective ---
  rr_objective_loader_ = std::make_unique<
      pluginlib::ClassLoader<task_priority_controllers::RRObjective>>(
      "task_priority_controllers", "task_priority_controllers::RRObjective");
  try
  {
    rr_objective_ = rr_objective_loader_->createUniqueInstance(
        pose_params_.rr_objective_type);
    RCLCPP_INFO_STREAM(node->get_logger(),
                       "\033[32mLoaded RR Objective: \033[0m"
                       "\033[1;32m"
                           << pose_params_.rr_objective_type << "\033[0m");
  }
  catch (const pluginlib::PluginlibException& e)
  {
    RCLCPP_ERROR(node->get_logger(),
                 "Failed to load redundancy resolution plugin. Execption: %s",
                 e.what());
    return CallbackReturn::FAILURE;
  }

  if (!rr_objective_->init(node, robot_chain_, upper_pos_limits_,
                           lower_pos_limits_))
  {
    RCLCPP_ERROR(node->get_logger(),
                 "Failed to initialize redundancy resolution objective.");
    return CallbackReturn::FAILURE;
  }

  // --- Positioner objective ---
  positioner_objective_loader_ = std::make_unique<pluginlib::ClassLoader<
      coordinated_motion_controllers::PositionerObjective>>(
      "coordinated_motion_controllers",
      "coordinated_motion_controllers::PositionerObjective");
  try
  {
    positioner_objective_ = positioner_objective_loader_->createUniqueInstance(
        pose_params_.pos_objective_type);
    RCLCPP_INFO_STREAM(node->get_logger(),
                       "\033[32mLoaded Positioner Objective: \033[0m"
                       "\033[1;32m"
                           << pose_params_.pos_objective_type << "\033[0m");
  }
  catch (const pluginlib::PluginlibException& e)
  {
    RCLCPP_ERROR(node->get_logger(),
                 "Failed to load positioner objective plugin. Execption: %s",
                 e.what());
    return CallbackReturn::FAILURE;
  }

  if (!positioner_objective_->init(node, robot_chain_))
  {
    RCLCPP_ERROR(node->get_logger(),
                 "Failed to initialize positioner objective.");
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(node->get_logger(),
              "Coordinated PoseController configured for %u robot joints "
              "and %u positioner joints.",
              n_robot_joints_, n_pos_joints_);

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PoseController::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  if (CoordinatedControllerBase::on_activate(previous_state) !=
      controller_interface::CallbackReturn::SUCCESS)
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  // initialize joint state from hardware
  read_state_from_hardware(joint_state_);

  KDL::JntArrayVel pos_state(n_pos_joints_);
  pos_state_interface_->read(pos_state);

  KDL::JntArray combined_positions(n_robot_joints_ + n_pos_joints_);
  combined_positions.data << pos_state.q.data.reverse(), joint_state_.q.data;

  Setpoint init_setpoint;
  coordinated_fk_solver_->JntToCart(combined_positions, init_setpoint.pose);
  setpoint_buffer_.writeFromNonRT(std::move(init_setpoint));

  RCLCPP_INFO(get_node()->get_logger(),
              "Activated CoordinatedPoseController...");
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type PoseController::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
{
  if (pose_param_listener_->is_old(pose_params_))
  {
    pose_params_ = pose_param_listener_->get_params();
  }

  KDL::JntArrayVel combined_state(n_pos_joints_ + n_robot_joints_);
  get_combined_state(combined_state);

  const Setpoint* setpoint = setpoint_buffer_.readFromRT();

  KDL::Jacobian coord_jac(n_robot_joints_ + n_pos_joints_);
  coordinated_jacobian_solver_->JntToJac(combined_state.q, coord_jac);

  KDL::Frame pose_kdl;
  coordinated_fk_solver_->JntToCart(combined_state.q, pose_kdl);

  ctrl::Pose pose;
  ctrl::transformKDLToEigen(pose_kdl, pose);

  ctrl::Pose sp_pose;
  ctrl::transformKDLToEigen(setpoint->pose, sp_pose);

  // --- Error computation ---
  ctrl::AngleAxis aa(sp_pose.rotation() * pose.rotation().inverse());
  ctrl::Vector3D orient_error = aa.axis() * aa.angle();
  ctrl::Vector3D trans_error(sp_pose.translation() - pose.translation());

  ctrl::Vector6D cart_cmd;
  cart_cmd << pose_params_.k_position * trans_error,
      pose_params_.k_orient * orient_error;
  cart_cmd += setpoint->twist;

  // --- Redundancy resolution ---
  ctrl::VectorND h = rr_objective_->getJointControlCmd(joint_state_);

  // --- Control law ---
  static ctrl::MatrixND I =
      ctrl::MatrixND::Identity(n_robot_joints_, n_robot_joints_);

  ctrl::MatrixND Jr =
      coord_jac.data.block(0, n_pos_joints_, 6, n_robot_joints_);
  ctrl::MatrixND Jr_pinv = ctrl::rightPinv(Jr);
  ctrl::MatrixND Jp = coord_jac.data.block(0, 0, 6, n_pos_joints_);

  ctrl::VectorND q_dot_pos = combined_state.q.data.head(n_pos_joints_);

  ctrl::VectorND joint_cmd =
      Jr_pinv * (cart_cmd - Jp * q_dot_pos) + (I - Jr_pinv * Jr) * h;

  ctrl::VectorND new_position =
      joint_state_.q.data + (joint_cmd * period.seconds());

  auto cmd = ctrl::transformEigenToKDL(new_position, joint_cmd);
  write_robot_command(cmd);

  // --- Suggested positioner command ---
  ctrl::VectorND robot_qdot_attempt =
      positioner_objective_->getJointControlCmd(joint_state_);

  ctrl::VectorND pos_setpoint =
      ctrl::dampedPinv(Jp, 0.1) * (cart_cmd - Jr * robot_qdot_attempt);

  pos_setpoint = pos_setpoint.reverse();
  write_positioner_command(pos_setpoint);

  return controller_interface::return_type::OK;
}

void PoseController::setpointCallback(
    const std::shared_ptr<taskspace_control_msgs::msg::PoseTwistSetpoint> msg)
{
  Setpoint setpoint;
  setpoint.pose.p = KDL::Vector(msg->pose.position.x, msg->pose.position.y,
                                msg->pose.position.z);
  setpoint.pose.M = KDL::Rotation::Quaternion(
      msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z,
      msg->pose.orientation.w);
  setpoint.twist << msg->twist.linear.x, msg->twist.linear.y,
      msg->twist.linear.z, msg->twist.angular.x, msg->twist.angular.y,
      msg->twist.angular.z;

  setpoint_buffer_.writeFromNonRT(setpoint);
}

}  // namespace coordinated_motion_controllers

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(coordinated_motion_controllers::PoseController,
                       controller_interface::ControllerInterface)

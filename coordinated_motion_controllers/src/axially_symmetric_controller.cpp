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

#include "coordinated_motion_controllers/axially_symmetric_controller.hpp"
#include "axially_symmetric_controllers/utility.hpp"

namespace coordinated_motion_controllers
{
controller_interface::return_type AxiallySymmetricController::update(
    const rclcpp::Time& time, const rclcpp::Duration& period)
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

  ctrl::AngleAxis aa(sp_pose.rotation() * pose.rotation().inverse());
  ctrl::Vector3D orient_error = aa.axis() * aa.angle();
  ctrl::Vector3D trans_error(sp_pose.translation() - pose.translation());

  ctrl::Vector5D cart_cmd;
  cart_cmd << pose_params_.k_position * trans_error + setpoint->twist.head<3>(),
      pose_params_.k_orient * orient_error;

  // --- Redundancy resolution ---
  ctrl::VectorND h = rr_objective_->getJointControlCmd(joint_state_);

  // --- Control law ---
  ctrl::MatrixND I = ctrl::MatrixND::Identity(n_robot_joints_, n_robot_joints_);
  ctrl::MatrixND Jr =
      coord_jac.data.block(0, n_pos_joints_, 5, n_robot_joints_);
  ctrl::MatrixND Jr_pinv = ctrl::rightPinv(Jr);
  ctrl::MatrixND Jp = coord_jac.data.block(0, 0, 5, n_pos_joints_);

  KDL::JntArrayVel test(n_pos_joints_);
  pos_state_interface_->read(test);

  ctrl::VectorND q_dot_pos = combined_state.qdot.data.head(n_pos_joints_);

  ctrl::VectorND joint_cmd =
      Jr_pinv * (cart_cmd - Jp * q_dot_pos) + (I - Jr_pinv * Jr) * h;

  KDL::JntArray q_cmd = ctrl::transformEigenToKDL(joint_cmd);
  auto cmd = ctrl::create_command(joint_state_.q, q_cmd, joint_limits_,
                                  period.seconds());
  write_robot_command(cmd);

  // --- Suggested positioner command ---
  ctrl::VectorND robot_qdot_attempt =
      positioner_objective_->getJointControlCmd(joint_state_);

  ctrl::VectorND pos_setpoint =
      // ctrl::dampedPinv(Jp, 0.1) * (cart_cmd - Jr * robot_qdot_attempt);
      ctrl::leftPinv(Jp) * (cart_cmd - Jr * robot_qdot_attempt);

  pos_setpoint = pos_setpoint.reverse();
  write_positioner_command(pos_setpoint);
  return controller_interface::return_type::OK;
}

}  // namespace coordinated_motion_controllers

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    coordinated_motion_controllers::AxiallySymmetricController,
    controller_interface::ControllerInterface)

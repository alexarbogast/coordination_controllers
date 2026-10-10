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

namespace coordinated_motion_controllers
{

controller_interface::CallbackReturn AxiallySymmetricController::on_init()
{
  // Initialize base class
  if (PoseController::on_init() !=
      controller_interface::CallbackReturn::SUCCESS)
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  // Initialize the AxiallySymmetricController
  try
  {
    as_param_listener_ =
        std::make_shared<axially_symmetric_controller::ParamListener>(
            get_node());
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

controller_interface::CallbackReturn AxiallySymmetricController::on_configure(
    const rclcpp_lifecycle::State& previous_state)
{
  // Configure the base class
  if (PoseController::on_configure(previous_state) !=
      controller_interface::CallbackReturn::SUCCESS)
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  // Configure the AxiallySymmetricController
  as_params_ = as_param_listener_->get_params();
  auto& tf_axis = as_params_.eef_frame_axis;
  auto& sf_axis = as_params_.setpoint_frame_axis;

  tool_frame_axis_ = ctrl::Vector3D(tf_axis[0], tf_axis[1], tf_axis[2]);
  setpoint_frame_axis_ = ctrl::Vector3D(sf_axis[0], sf_axis[1], sf_axis[2]);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AxiallySymmetricController::on_activate(
    const rclcpp_lifecycle::State& previous_state)
{
  // Activate base class
  if (CoordinatedControllerBase::on_activate(previous_state) !=
      controller_interface::CallbackReturn::SUCCESS)
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  // Initialize joint state from hardware
  read_state_from_hardware(joint_state_);

  KDL::JntArrayVel combined_state(n_pos_joints_ + n_robot_joints_);
  get_combined_state(combined_state);

  Setpoint fk;
  coordinated_fk_solver_->JntToCart(combined_state.q, fk.pose);

  // Initialize a pose with the setpoint_frame_axis_ aiming
  // in the direction of the tool_frame_axis_
  ctrl::Matrix3D R_fk;
  ctrl::transformKDLToEigen(fk.pose.M, R_fk);
  ctrl::Vector3D a_target = R_fk * tool_frame_axis_;
  a_target.normalize();

  ctrl::Vector3D a_set = setpoint_frame_axis_;
  a_set.normalize();

  ctrl::Quaternion q_align = ctrl::Quaternion::FromTwoVectors(a_set, a_target);

  Eigen::AngleAxisd twist(0.0, a_target);
  ctrl::Quaternion q_final = twist * q_align;
  ctrl::Matrix3D R = q_final.toRotationMatrix();

  Setpoint init_setpoint;
  init_setpoint.pose.M = ctrl::transformEigenToKDL(R);
  init_setpoint.pose.p = fk.pose.p;  // keep same position

  setpoint_buffer_.writeFromNonRT(std::move(init_setpoint));
  return CallbackReturn::SUCCESS;
}

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

  // Safety: bail out near singularities
  if (!check_manipulability(coord_jac))
  {
    return controller_interface::return_type::OK;
  }

  KDL::Frame pose_kdl;
  coordinated_fk_solver_->JntToCart(combined_state.q, pose_kdl);

  // --- Error computation ---
  ctrl::Matrix3D R_fk, R_setpoint;
  ctrl::transformKDLToEigen(pose_kdl.M, R_fk);
  ctrl::transformKDLToEigen(setpoint->pose.M, R_setpoint);

  ctrl::Vector3D a_current = R_fk * tool_frame_axis_;
  ctrl::Vector3D a_desired = R_setpoint * setpoint_frame_axis_;

  a_current.normalize();
  a_desired.normalize();

  ctrl::Vector3D pos_error((setpoint->pose.p - pose_kdl.p).data);
  ctrl::Vector3D axis_error = a_current.cross(a_desired).cross(a_current);

  // --- Command generation ---
  ctrl::Vector6D task_cmd;
  task_cmd << pose_params_.k_position * pos_error + setpoint->twist.head<3>(),
      pose_params_.k_orient * axis_error;

  // --- Control law ---
  ctrl::MatrixND I = ctrl::MatrixND::Identity(n_robot_joints_, n_robot_joints_);
  ctrl::MatrixND Jp = coord_jac.data.block(0, 0, 6, n_pos_joints_);
  ctrl::MatrixND Jr =
      coord_jac.data.block(0, n_pos_joints_, 6, n_robot_joints_);

  ctrl::MatrixND Jp_task = Jp;
  ctrl::MatrixND Jr_task = Jr;
  ctrl::Matrix3D a_skew = ctrl::skew(a_current);
  Jp_task.bottomRows(3) = -a_skew * Jp_task.bottomRows(3);
  Jr_task.bottomRows(3) = -a_skew * Jr_task.bottomRows(3);

  ctrl::MatrixND Jp_task_pinv = ctrl::pseudoInverse(Jp_task);
  ctrl::MatrixND Jr_task_pinv = ctrl::pseudoInverse(Jr_task);

  ctrl::VectorND h = rr_objective_->getJointControlCmd(joint_state_);
  ctrl::VectorND q_dot_pos = combined_state.qdot.data.head(n_pos_joints_);

  ctrl::VectorND q_dot_cmd = Jr_task_pinv * (task_cmd - Jp_task * q_dot_pos) +
                             (I - Jr_task_pinv * Jr_task) * h;

  ctrl::integrate_joint_velocity(joint_state_.q.data, q_dot_cmd, joint_limits_,
                                 period.seconds(), joint_command_);

  write_robot_command(joint_command_);

  // --- Suggested positioner command ---
  ctrl::VectorND robot_qdot_attempt =
      positioner_objective_->getJointControlCmd(joint_state_);

  ctrl::VectorND pos_setpoint =
      Jp_task_pinv * (task_cmd - Jr_task * robot_qdot_attempt);

  pos_setpoint = pos_setpoint.reverse();
  write_positioner_command(pos_setpoint);
  return controller_interface::return_type::OK;
}

}  // namespace coordinated_motion_controllers

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    coordinated_motion_controllers::AxiallySymmetricController,
    controller_interface::ControllerInterface)

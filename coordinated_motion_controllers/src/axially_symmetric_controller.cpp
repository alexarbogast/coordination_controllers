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

#include <coordinated_motion_controllers/axially_symmetric_controller.h>
#include <axially_symmetric_controllers/utility.h>

namespace coordinated_motion_controllers
{

void AxiallySymmetricController::update(const ros::Time&,
                                        const ros::Duration& period)
{
  synchronizeJointStates();  // update state

  const DynamicParams* params = dynamic_params_.readFromRT();
  const Setpoint* setpoint = setpoint_.readFromRT();

  KDL::JntArray combined_positions(n_robot_joints_ + n_pos_joints_);
  combined_positions.data << positioner_state_.readFromRT()->q.data.reverse(),
      robot_state_.q.data;

  KDL::Jacobian coord_jac(n_robot_joints_ + n_pos_joints_);
  coordinated_jacobian_solver_->JntToJac(combined_positions, coord_jac);

  KDL::Frame pose_pf;
  coordinated_fk_solver_->JntToCart(combined_positions, pose_pf);

  /* error */
  ctrl::Vector3D aim_current(pose_pf.M.UnitZ().data);
  ctrl::Vector3D aim_desired(setpoint->pose.M.UnitZ().data);

  ctrl::Vector3D rot_axis =
      axially_symmetric_controllers::axisBetween(aim_current, aim_desired);
  double rot_angle =
      axially_symmetric_controllers::angleBetween(aim_current, aim_desired);

  ctrl::Vector2D orient_error(rot_axis.x(), rot_axis.y());
  orient_error *= rot_angle;

  ctrl::Vector3D pos_error((setpoint->pose.p - pose_pf.p).data);

  ctrl::Vector5D cart_cmd;
  cart_cmd << params->k_position * pos_error + setpoint->twist.head<3>(),
      params->k_orient * orient_error;

  /* redundancy resolution */
  ctrl::VectorND h = ctrl::VectorND::Zero(n_robot_joints_);  // TODO

  /* control */
  ctrl::MatrixND I = ctrl::MatrixND::Identity(n_robot_joints_, n_robot_joints_);
  ctrl::MatrixND Jr =
      coord_jac.data.block(0, n_pos_joints_, 5, n_robot_joints_);
  ctrl::MatrixND Jr_pinv = ctrl::rightPinv(Jr);
  ctrl::MatrixND Jp = coord_jac.data.block(0, 0, 5, n_pos_joints_);

  ctrl::VectorND q_dot_pos =
      positioner_state_.readFromRT()->qdot.data.reverse();

  ctrl::VectorND joint_cmd =
      Jr_pinv * (cart_cmd - Jp * q_dot_pos) + (I - Jr_pinv * Jr) * h;

  ctrl::VectorND new_position =
      robot_state_.q.data + (joint_cmd * period.toSec());
  writeRobotCommand(new_position);

  /* desired positioner command */
  ctrl::VectorND robot_qdot_attempt =
      positioner_objective_->getJointControlCmd(robot_state_);

  ctrl::VectorND pos_setpoint =
      ctrl::leftPinv(Jp) * (cart_cmd - Jr * robot_qdot_attempt);

  pos_setpoint = pos_setpoint.reverse();
  writePositionerCommand(pos_setpoint);
}

}  // namespace coordinated_motion_controllers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(
    coordinated_motion_controllers::AxiallySymmetricController,
    controller_interface::ControllerBase)

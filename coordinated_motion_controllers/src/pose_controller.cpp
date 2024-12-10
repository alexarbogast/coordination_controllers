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

#include <coordinated_motion_controllers/pose_controller.h>
#include <axially_symmetric_controllers/utility.h>

namespace coordinated_motion_controllers
{

bool PoseController::init(hardware_interface::PositionJointInterface* hw,
                          ros::NodeHandle& nh)
{
  Base::init(hw, nh);

  coordinated_jacobian_solver_ =
      std::make_unique<KDL::ChainJntToJacSolver>(coordinated_chain_);

  sub_setpoint_ =
      nh.subscribe(setpoint_topic_, 1, &PoseController::setpointCallback, this);

  // Dynamic reconfigure
  dyn_reconf_server_ = std::make_shared<ReconfigureServer>(nh);
  dyn_reconf_server_->setCallback(std::bind(&PoseController::reconfCallback,
                                            this, std::placeholders::_1,
                                            std::placeholders::_2));

  return true;
}

void PoseController::update(const ros::Time&, const ros::Duration& period)
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

  /* error calculation */
  ctrl::Quaternion current_q, setpoint_q;
  pose_pf.M.GetQuaternion(current_q.x(), current_q.y(), current_q.z(),
                          current_q.w());

  setpoint->pose.M.GetQuaternion(setpoint_q.x(), setpoint_q.y(), setpoint_q.z(),
                                 setpoint_q.w());

  ctrl::Vector3D orient_error = (setpoint_q * current_q.inverse()).vec();
  ctrl::Vector3D trans_error((setpoint->pose.p - pose_pf.p).data);

  ctrl::Vector6D cart_cmd;
  cart_cmd << params->k_position * trans_error, params->k_orient * orient_error;
  cart_cmd += setpoint->twist;

  /* redundancy resolution */
  ctrl::VectorND h = ctrl::VectorND::Zero(n_robot_joints_);  // TODO

  /* control */
  ctrl::MatrixND I = ctrl::MatrixND::Identity(n_robot_joints_, n_robot_joints_);
  ctrl::MatrixND Jr =
      coord_jac.data.block(0, n_pos_joints_, 6, n_robot_joints_);
  ctrl::MatrixND Jr_pinv = ctrl::rightPinv(Jr);
  ctrl::MatrixND Jp = coord_jac.data.block(0, 0, 6, n_pos_joints_);

  ctrl::VectorND q_dot_pos =
      positioner_state_.readFromRT()->qdot.data.reverse();

  ctrl::VectorND joint_cmd =
      Jr_pinv * (cart_cmd - Jp * q_dot_pos) + (I - Jr_pinv * Jr) * h;

  ctrl::VectorND new_position =
      robot_state_.q.data + (joint_cmd * period.toSec());
  writeRobotCommand(new_position);

  /* desired positioner command */
  ctrl::VectorND robot_qdot_attempt =
      ctrl::VectorND::Zero(n_robot_joints_);  // TODO

  ctrl::VectorND pos_setpoint =
      ctrl::leftPinv(Jp) * (cart_cmd - Jr * robot_qdot_attempt);

  pos_setpoint = pos_setpoint.reverse();
  writePositionerCommand(pos_setpoint);
}

void PoseController::starting(const ros::Time&)
{
  synchronizeJointStates();
  KDL::JntArray combined_positions(n_robot_joints_ + n_pos_joints_);
  combined_positions.data << positioner_state_.readFromRT()->q.data.reverse(),
      robot_state_.q.data;

  Setpoint init_setpoint;
  coordinated_fk_solver_->JntToCart(combined_positions, init_setpoint.pose);
  setpoint_.initRT(init_setpoint);

  positioner_setpoint_pub_->msg_.coordinated = true;
}

void PoseController::stopping(const ros::Time&)
{
  positioner_setpoint_pub_->msg_.coordinated = false;
  positioner_setpoint_pub_->unlockAndPublish();
}

void PoseController::reconfCallback(ControllerConfig& config,
                                    uint16_t /*level*/)
{
  DynamicParams dynamic_params;
  dynamic_params.k_position = config.k_position;
  dynamic_params.k_orient = config.k_orient;

  dynamic_params_.writeFromNonRT(dynamic_params);
}

void PoseController::setpointCallback(
    const taskspace_control_msgs::PoseTwistSetpointConstPtr& msg)
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

  setpoint_.writeFromNonRT(setpoint);
}

}  // namespace coordinated_motion_controllers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(coordinated_motion_controllers::PoseController,
                       controller_interface::ControllerBase)

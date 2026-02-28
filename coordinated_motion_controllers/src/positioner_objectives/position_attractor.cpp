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

#include "coordinated_motion_controllers/positioner_objectives/position_attractor.hpp"
#include <kdl/jacobian.hpp>

namespace coordinated_motion_controllers
{

bool PositionAttractor::init(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
    const KDL::Chain& chain)
{
  if (!PositionerObjective::init(node, chain))
  {
    return false;
  }

  try
  {
    param_listener_ = std::make_shared<position_attractor::ParamListener>(node);
  }
  catch (const std::exception& e)
  {
    fprintf(stderr,
            "Exception thrown during positioner objective init with message: "
            "%s \n",
            e.what());
    return false;
  }

  params_ = param_listener_->get_params();
  std::copy_n(params_.attractor.data(), 3, tracked_position_.data);

  robot_fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain);
  jacobian_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain);
  return true;
}

ctrl::VectorND
PositionAttractor::getJointControlCmd(const KDL::JntArrayVel& joint_state)
{
  if (param_listener_->is_old(params_))
  {
    params_ = param_listener_->get_params();
  }

  KDL::Frame pose;
  robot_fk_solver_->JntToCart(joint_state.q, pose);

  KDL::Jacobian jac(n_joints_);
  jacobian_solver_->JntToJac(joint_state.q, jac);

  ctrl::Vector3D trans_error((tracked_position_ - pose.p).data);
  return params_.k_attract *
         ctrl::rightPinv(jac.data.block(0, 0, 3, n_joints_)) * trans_error;
}

}  // namespace coordinated_motion_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(coordinated_motion_controllers::PositionAttractor,
                       coordinated_motion_controllers::PositionerObjective)

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

#pragma once

#include "coordinated_motion_controllers/positioner_objectives/positioner_objective_plugin.hpp"
#include "coordinated_motion_controllers/position_attractor_parameters.hpp"

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

namespace coordinated_motion_controllers
{
class PositionAttractor
  : public coordinated_motion_controllers::PositionerObjective
{
public:
  PositionAttractor() = default;

  virtual bool init(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
                    const KDL::Chain& chain) override;
  virtual ctrl::VectorND
  getJointControlCmd(const KDL::JntArrayVel& joint_state) override;

protected:
  std::shared_ptr<position_attractor::ParamListener> param_listener_;
  position_attractor::Params params_;

  KDL::Vector tracked_position_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> robot_fk_solver_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jacobian_solver_;
};

}  // namespace coordinated_motion_controllers

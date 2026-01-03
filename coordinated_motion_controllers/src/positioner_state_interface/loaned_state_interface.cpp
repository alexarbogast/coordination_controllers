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

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <coordinated_motion_controllers/positioner_state_interface/loaned_state_interface.hpp>

namespace coordinated_motion_controllers
{
bool LoanedStateInterface::init(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
    const std::vector<std::string>& joint_names,
    const coordinated_controller_base::Params& params)
{
  if (!PositionerStateInterface::init(node, joint_names, params))
  {
    return false;
  }

  position_handles_.reserve(n_joints_);
  velocity_handles_.reserve(n_joints_);
  return true;
}

bool LoanedStateInterface::read(KDL::JntArrayVel& state)
{
  for (size_t i = 0; i < n_joints_; ++i)
  {
    state.q(i) = position_handles_[i]->get_value();
    state.qdot(i) = velocity_handles_[i]->get_value();
  }
  return true;
}

bool LoanedStateInterface::bind(
    std::vector<hardware_interface::LoanedStateInterface>& state_interfaces)
{
  position_handles_.clear();
  velocity_handles_.clear();

  for (const auto& joint : joint_names_)
  {
    auto pos_it = std::find_if(state_interfaces.begin(), state_interfaces.end(),
                               [&](const auto& iface) {
                                 return iface.get_prefix_name() == joint &&
                                        iface.get_interface_name() ==
                                            hardware_interface::HW_IF_POSITION;
                               });

    auto vel_it = std::find_if(state_interfaces.begin(), state_interfaces.end(),
                               [&](const auto& iface) {
                                 return iface.get_prefix_name() == joint &&
                                        iface.get_interface_name() ==
                                            hardware_interface::HW_IF_VELOCITY;
                               });

    if (pos_it == state_interfaces.end() || vel_it == state_interfaces.end())
    {
      return false;
    }

    position_handles_.push_back(&(*pos_it));
    velocity_handles_.push_back(&(*vel_it));
  }

  return true;
}

}  // namespace coordinated_motion_controllers

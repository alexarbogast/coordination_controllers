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

#include <hardware_interface/loaned_state_interface.hpp>
#include <coordinated_motion_controllers/positioner_state_interface/positioner_state_interface.hpp>

namespace coordinated_motion_controllers
{
/**
 * @brief State interface that reads positioner joint state directly from
 *        ros2_control loaned state interfaces.
 *
 * This implementation provides deterministic, zero-copy access to the
 * positioner joint position and velocity as part of the controller update
 * cycle. It is intended for use when the positioner is managed by the same
 * ros2_control controller manager as the consuming controller, enabling
 * tight synchronization and minimal latency.
 */
class LoanedStateInterface : public PositionerStateInterface
{
public:
  virtual bool init(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
                    const std::vector<std::string>& joint_names,
                    const coordinated_controller_base::Params& params) override;
  virtual bool read(KDL::JntArrayVel& state) override;

  bool
  bind(std::vector<hardware_interface::LoanedStateInterface>& state_interfaces);

private:
  std::vector<hardware_interface::LoanedStateInterface*> position_handles_;
  std::vector<hardware_interface::LoanedStateInterface*> velocity_handles_;
};

}  // namespace coordinated_motion_controllers

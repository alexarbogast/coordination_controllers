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

#include "coordinated_motion_controllers/positioner_state_interface/topic_state_interface.hpp"

namespace coordinated_motion_controllers
{
bool TopicStateInterface::init(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
    const std::vector<std::string>& joint_names,
    const coordinated_controller_base::Params& params)
{
  if (!PositionerStateInterface::init(node, joint_names, params))
  {
    return false;
  }

  // Write initial setpoint
  KDL::JntArrayVel pos_state(n_joints_);
  pos_state.q.data.setZero();
  pos_state.qdot.data.setZero();
  positioner_state_.writeFromNonRT(pos_state);

  positioner_sub_ = node->create_subscription<JointStateMsg>(
      params.positioner_topic, rclcpp::SystemDefaultsQoS(),
      std::bind(&TopicStateInterface::posJointStateCallback, this,
                std::placeholders::_1));
  return true;
}

bool TopicStateInterface::read(KDL::JntArrayVel& state)
{
  state = *positioner_state_.readFromRT();
  return true;
}

void TopicStateInterface::posJointStateCallback(
    const JointStateMsg::SharedPtr msg)
{
  // Create index mapping for filtering
  std::vector<int> indices(n_joints_, -1);

  // Find matching joint names and their indices
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
    if (it != msg->name.end())
    {
      indices[i] = std::distance(msg->name.begin(), it);
    }
  }

  // Create filtered KDL::JntArrayVel
  KDL::JntArrayVel pos_state(n_joints_);

  // Populate with filtered data
  for (size_t i = 0; i < indices.size(); ++i)
  {
    if (indices[i] >= 0)
    {
      // Set position
      if (static_cast<size_t>(indices[i]) < msg->position.size())
      {
        pos_state.q(i) = msg->position[indices[i]];
      }

      // Set velocity
      if (static_cast<size_t>(indices[i]) < msg->velocity.size())
      {
        pos_state.qdot(i) = msg->velocity[indices[i]];
      }
      else
      {
        pos_state.qdot(i) = 0.0;
      }
    }
  }

  // Write to realtime buffer
  positioner_state_.writeFromNonRT(pos_state);
}

}  // namespace coordinated_motion_controllers

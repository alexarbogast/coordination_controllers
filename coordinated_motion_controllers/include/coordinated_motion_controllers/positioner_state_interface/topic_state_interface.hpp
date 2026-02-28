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

#include "sensor_msgs/msg/joint_state.hpp"
#include "realtime_tools/realtime_buffer.hpp"

#include "coordinated_motion_controllers/positioner_state_interface/positioner_state_interface.hpp"

namespace coordinated_motion_controllers
{
/**
 * @brief State interface that acquires positioner joint state from a ROS topic.
 *
 * This implementation subscribes to a positioner state topic and buffers the
 * most recent message in a real-time safe structure for use by a controller.
 * It enables coordination with external or remotely managed positioners where
 * ros2_control loaned state interfaces are not available.
 */
class TopicStateInterface : public PositionerStateInterface
{
public:
  using JointStateMsg = sensor_msgs::msg::JointState;
  using JointStateSub = rclcpp::Subscription<JointStateMsg>::SharedPtr;

  virtual bool init(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
                    const std::vector<std::string>& joint_names,
                    const coordinated_controller_base::Params& params) override;
  virtual bool read(KDL::JntArrayVel& state) override;

private:
  void posJointStateCallback(const JointStateMsg::SharedPtr msg);

  JointStateSub positioner_sub_;
  realtime_tools::RealtimeBuffer<KDL::JntArrayVel> positioner_state_;
};

}  // namespace coordinated_motion_controllers

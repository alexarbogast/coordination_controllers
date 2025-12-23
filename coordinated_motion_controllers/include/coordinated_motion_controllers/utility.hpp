// Copyright 2025 Alex Arbogast
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

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rclcpp/wait_set.hpp>

namespace ctrl
{

/**
 * @brief Wait until a joint states message has been received
 *
 * @param node the node for creating the subscription
 * @param topic the name of the joint states topic
 * @param timeout the duration to wait before returning a nullptr
 *
 * @returns the joint state message if no timeout, else nullptr
 */
template <class Rep, class Period>
sensor_msgs::msg::JointState::SharedPtr
waitForJointState(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                  const std::string& topic,
                  std::chrono::duration<Rep, Period> timeout)
{
  using sensor_msgs::msg::JointState;

  auto sub = node->create_subscription<JointState>(
      topic, rclcpp::QoS(1).best_effort(), [](JointState::SharedPtr) {
        // No-op: using WaitSet + take()
      });

  rclcpp::WaitSet wait_set;
  wait_set.add_subscription(sub);

  auto start = std::chrono::steady_clock::now();

  while (rclcpp::ok())
  {
    auto elapsed = std::chrono::steady_clock::now() - start;
    if (elapsed >= timeout)
    {
      return nullptr;
    }

    auto result = wait_set.wait(timeout - elapsed);
    if (result.kind() == rclcpp::WaitResultKind::Ready)
    {
      JointState msg;
      rclcpp::MessageInfo info;

      if (sub->take(msg, info))
      {
        return std::make_shared<JointState>(std::move(msg));
      }
    }
  }

  return nullptr;
}
}  // namespace ctrl

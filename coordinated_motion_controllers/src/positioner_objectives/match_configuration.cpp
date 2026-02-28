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

#include "coordinated_motion_controllers/positioner_objectives/match_configuration.hpp"

const static std::string CONFIG_PARAM = "match_config";

namespace coordinated_motion_controllers
{

bool MatchConfiguration::init(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
    const KDL::Chain& chain)
{
  if (!PositionerObjective::init(node, chain))
  {
    return false;
  }

  try
  {
    param_listener_ =
        std::make_shared<match_configuration::ParamListener>(node);
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
  config_.data = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      params_.match_config.data(), params_.match_config.size());

  if (params_.match_config.size() != n_joints_)
  {
    const auto msg = std::string("Number of joints in ") +
                     node->get_namespace() + "/" + CONFIG_PARAM +
                     " does not match robot chain";
    RCLCPP_ERROR(node->get_logger(), "%s", msg.c_str());
    return false;
  }
  return true;
}

ctrl::VectorND
MatchConfiguration::getJointControlCmd(const KDL::JntArrayVel& joint_state)
{
  if (param_listener_->is_old(params_))
  {
    params_ = param_listener_->get_params();
  }
  return params_.k_config * (config_.data - joint_state.q.data);
}

}  // namespace coordinated_motion_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(coordinated_motion_controllers::MatchConfiguration,
                       coordinated_motion_controllers::PositionerObjective)

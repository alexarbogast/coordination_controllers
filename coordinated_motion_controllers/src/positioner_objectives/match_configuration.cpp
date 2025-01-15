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

#include <coordinated_motion_controllers/positioner_objectives/match_configuration.h>

const static std::string CONFIG_PARAM = "match_config";

namespace coordinated_motion_controllers
{

bool MatchConfiguration::init(ros::NodeHandle& nh, const KDL::Chain& chain)
{
  if (!PositionerObjective::init(nh, chain))
  {
    return false;
  }

  // Read home configuration from ros parameters
  ros::NodeHandle pnh(nh, "pos_objective");
  std::vector<double> home_config;
  if (!pnh.getParam(CONFIG_PARAM, home_config))
  {
    ROS_ERROR_STREAM("Failed to load " << pnh.getNamespace() << "/"
                                       << CONFIG_PARAM
                                       << " from parameter server");
    return false;
  }
  config_.data = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      home_config.data(), home_config.size());

  if (home_config.size() != n_joints_)
  {
    ROS_ERROR_STREAM("Number of joints in " << pnh.getNamespace() << "/"
                                            << CONFIG_PARAM
                                            << " does not match robot chain");
    return false;
  }
  return true;
}

ctrl::VectorND
MatchConfiguration::getJointControlCmd(const KDL::JntArrayVel& joint_state)
{
  return config_.data - joint_state.q.data;
}

}  // namespace coordinated_motion_controllers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(coordinated_motion_controllers::MatchConfiguration,
                       coordinated_motion_controllers::PositionerObjective)

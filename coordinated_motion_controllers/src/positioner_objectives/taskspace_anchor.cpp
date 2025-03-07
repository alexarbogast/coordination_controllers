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

#include <coordinated_motion_controllers/positioner_objectives/taskspace_anchor.h>
#include <kdl/jacobian.hpp>

const static std::string CONFIG_PARAM = "match_config";

namespace coordinated_motion_controllers
{

bool TaskspaceAnchor::init(ros::NodeHandle& nh, const KDL::Chain& chain)
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
  KDL::JntArray config;
  config.data = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      home_config.data(), home_config.size());

  if (home_config.size() != n_joints_)
  {
    ROS_ERROR_STREAM("Number of joints in " << pnh.getNamespace() << "/"
                                            << CONFIG_PARAM
                                            << " does not match robot chain");
    return false;
  }

  robot_fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain);
  jacobian_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain);

  KDL::Frame pose;
  robot_fk_solver_->JntToCart(config, pose);
  tracked_position_ = pose.p;

  // Dynamic reconfigure
  dyn_reconf_server_ = std::make_shared<ReconfigureServer>(pnh);
  dyn_reconf_server_->setCallback(std::bind(&TaskspaceAnchor::reconfCallback,
                                            this, std::placeholders::_1,
                                            std::placeholders::_2));
  return true;
}

ctrl::VectorND
TaskspaceAnchor::getJointControlCmd(const KDL::JntArrayVel& joint_state)
{
  const DynamicParams* params = dynamic_params_.readFromRT();

  KDL::Frame pose;
  robot_fk_solver_->JntToCart(joint_state.q, pose);

  KDL::Jacobian jac(n_joints_);
  jacobian_solver_->JntToJac(joint_state.q, jac);

  ctrl::Vector3D trans_error((tracked_position_ - pose.p).data);
  return params->k_prop * ctrl::rightPinv(jac.data.block(0, 0, 3, n_joints_)) *
         trans_error;
}

void TaskspaceAnchor::reconfCallback(ObjectiveConfig& config,
                                     uint16_t /*level*/)
{
  DynamicParams dynamic_params;
  dynamic_params.k_prop = config.k_prop;
  dynamic_params_.writeFromNonRT(dynamic_params);
}

}  // namespace coordinated_motion_controllers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(coordinated_motion_controllers::TaskspaceAnchor,
                       coordinated_motion_controllers::PositionerObjective)

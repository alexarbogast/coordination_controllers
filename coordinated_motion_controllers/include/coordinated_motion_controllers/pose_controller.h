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

#include <coordinated_motion_controllers/coordinated_controller_base.h>
#include <coordinated_motion_controllers/positioner_objectives/positioner_objective_plugin.h>

#include <taskspace_controllers/setpoint.h>
#include <task_priority_controllers/objectives/objective_plugin.h>

#include <kdl/chainjnttojacsolver.hpp>

#include <dynamic_reconfigure/server.h>
#include <pluginlib/class_loader.h>

namespace coordinated_motion_controllers
{
class PoseController
  : public coordinated_motion_controllers::CoordinatedControllerBase
{
public:
  virtual bool init(hardware_interface::PositionJointInterface* hw,
                    ros::NodeHandle& nh) override;
  virtual void update(const ros::Time&, const ros::Duration& period) override;
  virtual void starting(const ros::Time&) override;
  virtual void stopping(const ros::Time&) override;

protected:
  typedef CoordinatedControllerBase Base;
  typedef taskspace_controllers::PoseTwistSetpoint Setpoint;
  typedef taskspace_controllers::PoseControllerConfig ControllerConfig;
  typedef dynamic_reconfigure::Server<ControllerConfig> ReconfigureServer;

  void reconfCallback(ControllerConfig& config, uint16_t /*level*/);
  void setpointCallback(
      const taskspace_control_msgs::PoseTwistSetpointConstPtr& msg);

  // kinematic solvers
  std::unique_ptr<KDL::ChainJntToJacSolver> coordinated_jacobian_solver_;

  // setpoint
  realtime_tools::RealtimeBuffer<Setpoint> setpoint_;
  ros::Subscriber sub_setpoint_;

  // redundancy resolution objective
  std::shared_ptr<task_priority_controllers::RRObjective> rr_objective_;
  std::unique_ptr<
      pluginlib::ClassLoader<task_priority_controllers::RRObjective>>
      rr_objective_loader_;

  // positioner objective
  std::shared_ptr<PositionerObjective> positioner_objective_;
  std::unique_ptr<pluginlib::ClassLoader<PositionerObjective>>
      positioner_objective_loader_;

  // dynamic reconfigure
  struct DynamicParams
  {
    double k_position = 1.0;  // gain values
    double k_orient = 1.0;
  };
  realtime_tools::RealtimeBuffer<DynamicParams> dynamic_params_;
  std::shared_ptr<ReconfigureServer> dyn_reconf_server_;
};

}  // namespace coordinated_motion_controllers

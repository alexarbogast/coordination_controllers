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

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <taskspace_controllers/utility.h>
#include <taskspace_controllers/setpoint.h>
#include <taskspace_controllers/PoseControllerConfig.h>

#include <kdl/jntarrayvel.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <sensor_msgs/JointState.h>
#include <taskspace_control_msgs/PoseTwistSetpoint.h>
#include <taskspace_control_msgs/QueryPose.h>
#include <coordinated_control_msgs/PositionerSetpoint.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

namespace coordinated_motion_controllers
{

class CoordinatedControllerBase
  : public controller_interface::Controller<
        hardware_interface::PositionJointInterface>
{
public:
  virtual bool init(hardware_interface::PositionJointInterface* hw,
                    ros::NodeHandle& nh) override;

protected:
  void synchronizeJointStates();
  void writeRobotCommand(const ctrl::VectorND& cmd);
  void writePositionerCommand(const ctrl::VectorND& cmd);

  void posJointStateCallback(const sensor_msgs::JointStateConstPtr& msg);
  bool queryPoseService(taskspace_control_msgs::QueryPose::Request& req,
                        taskspace_control_msgs::QueryPose::Response& resp);

protected:
  unsigned int n_robot_joints_, n_pos_joints_;
  std::vector<hardware_interface::JointHandle> joint_handles_;
  std::string base_link_, eef_link_, positioner_link_;
  std::string setpoint_topic_;

  // kinematics
  KDL::Chain robot_chain_, coordinated_chain_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> coordinated_fk_solver_;
  KDL::JntArray upper_pos_limits_;
  KDL::JntArray lower_pos_limits_;

  // state feedback
  KDL::JntArrayVel robot_state_;

  // positioner
  realtime_tools::RealtimeBuffer<KDL::JntArrayVel> positioner_state_;
  ros::Subscriber sub_positioner_joint_states_;
  std::unique_ptr<realtime_tools::RealtimePublisher<
      coordinated_control_msgs::PositionerSetpoint>>
      positioner_setpoint_pub_;

  ros::ServiceServer query_pose_service_;
};

}  // namespace coordinated_motion_controllers

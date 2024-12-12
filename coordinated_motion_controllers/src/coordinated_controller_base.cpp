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

#include <coordinated_motion_controllers/coordinated_controller_base.h>
#include <axially_symmetric_controllers/utility.h>

#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <pluginlib/class_list_macros.h>

#define LOAD_ROS_PARAM(nh, param_name, variable)                               \
  if (!nh.getParam(param_name, variable))                                      \
  {                                                                            \
    ROS_ERROR_STREAM("Failed to load parameter '"                              \
                     << param_name << "' from the parameter server.");         \
    return false;                                                              \
  }

namespace coordinated_motion_controllers
{

static const std::string POS_SETPOINT_NS = "pos_setpoint";

bool CoordinatedControllerBase::init(
    hardware_interface::PositionJointInterface* hw, ros::NodeHandle& nh)
{
  const std::string ns = nh.getNamespace();

  // Load robot description and link nameCoordinatedControllerBases
  std::string robot_description;
  LOAD_ROS_PARAM(nh, "/robot_description", robot_description);
  LOAD_ROS_PARAM(nh, "positioner_link", positioner_link_);
  LOAD_ROS_PARAM(nh, "base_link", base_link_);
  LOAD_ROS_PARAM(nh, "eef_link", eef_link_);

  // Initialize kinematics solvers
  urdf::Model urdf_model;
  KDL::Tree kdl_tree;
  if (!urdf_model.initString(robot_description))
  {
    ROS_ERROR("Failed to initialize urdf model from 'robot_description'");
    return false;
  }
  if (!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree))
  {
    ROS_FATAL("Failed to parse KDL tree from urdf model");
    return false;
  }

  if (!kdl_tree.getChain(base_link_, eef_link_, robot_chain_))
  {
    ROS_FATAL_STREAM("Failed to build kinematic chain from '"
                     << base_link_ << "' to '" << eef_link_
                     << "'. Make sure these links exist in the URDF.");
    return false;
  }
  if (!kdl_tree.getChain(positioner_link_, eef_link_, coordinated_chain_))
  {
    ROS_FATAL_STREAM("Failed to build kinematic chain from '"
                     << positioner_link_ << "' to '" << eef_link_
                     << "'. Make sure these links exist in the URDF.");
    return false;
  }

  coordinated_fk_solver_ =
      std::make_unique<KDL::ChainFkSolverPos_recursive>(coordinated_chain_);

  // Parse joint limits
  std::vector<std::string> joint_names;
  LOAD_ROS_PARAM(nh, "joints", joint_names);
  n_robot_joints_ = joint_names.size();

  upper_pos_limits_.resize(n_robot_joints_);
  lower_pos_limits_.resize(n_robot_joints_);
  for (size_t i = 0; i < n_robot_joints_; ++i)
  {
    if (!urdf_model.getJoint(joint_names[i]))
    {
      ROS_ERROR_STREAM("Joint " + joint_names[i] + " does not exist in URDF");
      return false;
    }
    if (urdf_model.getJoint(joint_names[i])->type == urdf::Joint::CONTINUOUS)
    {
      upper_pos_limits_(i) = std::nan("0");
      lower_pos_limits_(i) = std::nan("0");
    }
    else
    {
      upper_pos_limits_(i) = urdf_model.getJoint(joint_names[i])->limits->upper;
      lower_pos_limits_(i) = urdf_model.getJoint(joint_names[i])->limits->lower;
    }
  }

  // Get joint handles from hardware interface
  joint_handles_.resize(n_robot_joints_);
  for (size_t i = 0; i < n_robot_joints_; ++i)
  {
    try
    {
      joint_handles_[i] = hw->getHandle(joint_names[i]);
    }
    catch (const hardware_interface::HardwareInterfaceException& e)
    {
      ROS_ERROR_STREAM("Exception getting joint handles from hw interface"
                       << e.what());
      return false;
    }
  }

  // Setup positioner joint state
  std::string positioner_topic;
  LOAD_ROS_PARAM(nh, "positioner_topic", positioner_topic);

  sensor_msgs::JointStateConstPtr pos_joint_state =
      ros::topic::waitForMessage<sensor_msgs::JointState>(positioner_topic,
                                                          ros::Duration(10));
  if (pos_joint_state == NULL)
  {
    ROS_ERROR_STREAM("Timed out waiting for topic:" << positioner_topic);
    return false;
  }
  n_pos_joints_ = pos_joint_state->name.size();
  posJointStateCallback(pos_joint_state);

  robot_state_.resize(n_robot_joints_);

  // Find setpoint topic
  LOAD_ROS_PARAM(nh, "setpoint_topic", setpoint_topic_);

  // Setup ROS components
  sub_positioner_joint_states_ =
      nh.subscribe(positioner_topic, 1,
                   &CoordinatedControllerBase::posJointStateCallback, this);

  positioner_setpoint_pub_ = std::make_unique<realtime_tools::RealtimePublisher<
      coordinated_control_msgs::PositionerSetpoint>>(nh, POS_SETPOINT_NS, 1);
  positioner_setpoint_pub_->msg_.coordinated = false;
  positioner_setpoint_pub_->msg_.velocity.resize(n_pos_joints_);

  // Services
  query_pose_service_ = nh.advertiseService(
      "query_pose", &CoordinatedControllerBase::queryPoseService, this);

  return true;
}

void CoordinatedControllerBase::synchronizeJointStates()
{
  // Synchronize the internal state with the hardware
  for (unsigned int i = 0; i < n_robot_joints_; ++i)
  {
    robot_state_.q(i) = joint_handles_[i].getPosition();
    robot_state_.qdot(i) = joint_handles_[i].getVelocity();
  }
}

void CoordinatedControllerBase::writeRobotCommand(const ctrl::VectorND& cmd)
{
  for (unsigned int i = 0; i < n_robot_joints_; ++i)
  {
    joint_handles_[i].setCommand(cmd[i]);
  }
}

void CoordinatedControllerBase::writePositionerCommand(
    const ctrl::VectorND& cmd)
{
  if (positioner_setpoint_pub_->trylock())
  {
    Eigen::VectorXd::Map(&positioner_setpoint_pub_->msg_.velocity[0],
                         cmd.size()) = cmd;
    positioner_setpoint_pub_->unlockAndPublish();
  }
}

void CoordinatedControllerBase::posJointStateCallback(
    const sensor_msgs::JointStateConstPtr& msg)
{
  KDL::JntArrayVel pos_state(n_pos_joints_);
  for (size_t i = 0; i < n_pos_joints_; ++i)
  {
    pos_state.q(i) = msg->position[i];
    pos_state.qdot(i) = msg->velocity[i];
  }
  positioner_state_.writeFromNonRT(pos_state);
}

bool CoordinatedControllerBase::queryPoseService(
    taskspace_control_msgs::QueryPose::Request& req,
    taskspace_control_msgs::QueryPose::Response& resp)
{
  KDL::JntArray robot_state(n_robot_joints_);
  for (unsigned int i = 0; i < n_robot_joints_; ++i)
  {
    robot_state(i) = joint_handles_[i].getPosition();
  }

  KDL::JntArray combined_positions(n_robot_joints_ + n_pos_joints_);
  combined_positions.data << positioner_state_.readFromNonRT()
                                 ->q.data.reverse(),
      robot_state.data;

  KDL::Frame pose;
  coordinated_fk_solver_->JntToCart(combined_positions, pose);

  resp.pose.position.x = pose.p.x();
  resp.pose.position.y = pose.p.y();
  resp.pose.position.z = pose.p.z();

  pose.M.GetQuaternion(resp.pose.orientation.x, resp.pose.orientation.y,
                       resp.pose.orientation.z, resp.pose.orientation.w);
  return true;
}

}  // namespace coordinated_motion_controllers

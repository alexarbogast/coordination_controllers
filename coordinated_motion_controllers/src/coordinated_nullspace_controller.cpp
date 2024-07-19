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

#include <coordinated_motion_controllers/coordinated_nullspace_controller.h>
#include <axially_symmetric_controllers/utility.h>

#include <urdf/model.h>
#include <kdl/jntarrayvel.hpp>
#include <kdl/tree.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <pluginlib/class_list_macros.h>

namespace coordinated_motion_controllers
{

static const Eigen::Matrix<double, 5, 5> identity5x5 =
    Eigen::Matrix<double, 5, 5>::Identity();

static const double MANIP_THRESHOLD = 1e-6;

static const std::string POS_SETPOINT_NS = "pos_setpoint";
static const std::string DIAGNOSTICS_NS = "diagnostics";

bool CoordinatedNullspaceController::init(
    hardware_interface::PositionJointInterface* hw, ros::NodeHandle& nh)
{
  const std::string ns = nh.getNamespace();

  // Load robot description and link names
  std::string robot_description;
  if (!nh.getParam("/robot_description", robot_description))
  {
    ROS_ERROR_STREAM("Failed to load " << robot_description
                                       << " from parameter server");
    return false;
  }
  if (!nh.getParam("positioner_link", positioner_link_))
  {
    ROS_ERROR_STREAM("Failed to load " << nh.getNamespace() + "/positioner_link"
                                       << " from parameter server");
    return false;
  }
  if (!nh.getParam("base_link", base_link_))
  {
    ROS_ERROR_STREAM("Failed to load " << nh.getNamespace() + "/base_link"
                                       << " from parameter server");
    return false;
  }
  if (!nh.getParam("eef_link", eef_link_))
  {
    ROS_ERROR_STREAM("Failed to load " << nh.getNamespace() + "/eef_link"
                                       << " from parameter server");
    return false;
  }

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

  if (!kdl_tree.getChain(positioner_link_, eef_link_, coordinated_chain_))
  {
    ROS_FATAL_STREAM("Failed to build kinematic chain from '"
                     << positioner_link_ << "' to '" << eef_link_
                     << "'. Make sure these links exist in the URDF.");
    return false;
  }
  if (!kdl_tree.getChain(base_link_, eef_link_, robot_chain_))
  {
    ROS_FATAL_STREAM("Failed to build kinematic chain from '"
                     << base_link_ << "' to '" << eef_link_
                     << "'. Make sure these links exist in the URDF.");
    return false;
  }

  coordinated_jacobian_solver_ =
      std::make_unique<KDL::ChainJntToJacSolver>(coordinated_chain_);
  robot_jacobian_solver_ =
      std::make_unique<KDL::ChainJntToJacSolver>(robot_chain_);
  robot_jacobian_dot_solver_ =
      std::make_unique<KDL::ChainJntToJacDotSolver>(robot_chain_);

  coordinated_fk_solver_ =
      std::make_unique<KDL::ChainFkSolverPos_recursive>(coordinated_chain_);
  robot_fk_solver_ =
      std::make_unique<KDL::ChainFkSolverPos_recursive>(robot_chain_);

  // Parse joint limits
  std::vector<std::string> joint_names;
  if (!nh.getParam("joints", joint_names))
  {
    ROS_ERROR_STREAM("Failed to load " << ns << "/joints"
                                       << " from parameter server");
    return false;
  }
  n_robot_joints_ = joint_names.size();

  KDL::JntArray upper_pos_limits(n_robot_joints_);
  KDL::JntArray lower_pos_limits(n_robot_joints_);
  for (size_t i = 0; i < n_robot_joints_; ++i)
  {
    if (!urdf_model.getJoint(joint_names[i]))
    {
      ROS_ERROR_STREAM("Joint " + joint_names[i] + " does not exist in URDF");
      return false;
    }
    if (urdf_model.getJoint(joint_names[i])->type == urdf::Joint::CONTINUOUS)
    {
      upper_pos_limits(i) = std::nan("0");
      lower_pos_limits(i) = std::nan("0");
    }
    else
    {
      upper_pos_limits(i) = urdf_model.getJoint(joint_names[i])->limits->upper;
      lower_pos_limits(i) = urdf_model.getJoint(joint_names[i])->limits->lower;
    }
  }

  limits_avg_.resize(n_robot_joints_);
  limits_avg_.data = (upper_pos_limits.data + lower_pos_limits.data) / 2;

  limits_bounds_.resize(n_robot_joints_);
  limits_bounds_.data = (upper_pos_limits.data - lower_pos_limits.data) / 2;

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
  if (!nh.getParam("positioner_topic", positioner_topic))
  {
    ROS_ERROR_STREAM("Failed to load " << ns << "/positioner_topic"
                                       << " from parameter server");
    return false;
  }

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
  std::string setpoint_topic;
  if (!nh.getParam("setpoint_topic", setpoint_topic))
  {
    ROS_ERROR_STREAM("Failed to load " << ns << "/setpoint_topic"
                                       << " from parameter server");
    return false;
  }

  // Read eef aiming vector (in base frame)
  std::vector<double> aiming_vec;
  if (!nh.getParam("aiming_vec", aiming_vec))
  {
    ROS_ERROR_STREAM("Failed to load " << ns << "/aiming_vec"
                                       << " from parameter server");
    return false;
  }
  aiming_vec_ = Eigen::Vector3d(aiming_vec.data());

  // Read home configuration
  std::vector<double> home_config;
  if (!nh.getParam("home_config", home_config))
  {
    ROS_ERROR_STREAM("Failed to load " << ns << "/home_config"
                                       << " from parameter server");
    return false;
  }
  home_config_ = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      home_config.data(), home_config.size());

  // Setup ROS components
  sub_positioner_joint_states_ = nh.subscribe(
      positioner_topic, 1,
      &CoordinatedNullspaceController::posJointStateCallback, this);

  sub_setpoint_ =
      nh.subscribe(setpoint_topic, 1,
                   &CoordinatedNullspaceController::setpointCallback, this);

  positioner_setpoint_pub_ = std::make_unique<realtime_tools::RealtimePublisher<
      coordinated_control_msgs::PositionerSetpoint>>(nh, POS_SETPOINT_NS, 1);
  positioner_setpoint_pub_->msg_.coordinated = false;
  positioner_setpoint_pub_->msg_.velocity.resize(n_pos_joints_);

  diagnostics_pub_ = std::make_unique<
      realtime_tools::RealtimePublisher<coordinated_control_msgs::Diagnostics>>(
      nh, DIAGNOSTICS_NS, 1);

  // Dynamic reconfigure
  dyn_reconf_server_ = std::make_shared<ReconfigureServer>(nh);
  dyn_reconf_server_->setCallback(
      std::bind(&CoordinatedNullspaceController::reconfCallback, this,
                std::placeholders::_1, std::placeholders::_2));

  // Services
  query_pose_service_ = nh.advertiseService(
      "query_pose", &CoordinatedNullspaceController::queryPoseService, this);

  return true;
}

void CoordinatedNullspaceController::update(const ros::Time&,
                                            const ros::Duration& period)
{
  synchronizeJointStates();  // update state

  const DynamicParams* params = dynamic_params_.readFromRT();
  const Setpoint* setpoint = setpoint_.readFromRT();

  KDL::JntArray combined_positions(n_robot_joints_ + n_pos_joints_);
  combined_positions.data << positioner_state_.readFromRT()->q.data.reverse(),
      robot_state_.q.data;

  KDL::Jacobian rob_jac(n_robot_joints_);
  KDL::Jacobian coord_jac(n_robot_joints_ + n_pos_joints_);
  robot_jacobian_solver_->JntToJac(robot_state_.q, rob_jac);
  coordinated_jacobian_solver_->JntToJac(combined_positions, coord_jac);

  KDL::Frame pose_pf, pose_bf;
  robot_fk_solver_->JntToCart(robot_state_.q, pose_bf);
  coordinated_fk_solver_->JntToCart(combined_positions, pose_pf);

  /* error */
  Eigen::Vector3d aim_current(pose_pf.M.UnitZ().data);
  Eigen::Vector3d aim_desired(setpoint->pose.M.UnitZ().data);

  Eigen::Vector3d rot_axis =
      axially_symmetric_controllers::axisBetween(aim_current, aim_desired);
  double rot_angle =
      axially_symmetric_controllers::angleBetween(aim_current, aim_desired);

  Eigen::Vector2d orient_error(rot_axis.x(), rot_axis.y());
  orient_error *= rot_angle;

  Eigen::Vector3d pos_error((setpoint->pose.p - pose_pf.p).data);

  Eigen::Matrix<double, 5, 1> cart_cmd;
  cart_cmd << params->k_position * pos_error + setpoint->velocity,
      params->k_aiming * orient_error;

  /* redundancy resolution */
  // manipulability maximization
  Eigen::MatrixXd J_JT = rob_jac.data * rob_jac.data.transpose();
  double manip = sqrt(J_JT.determinant());

  Eigen::VectorXd manip_grad(n_robot_joints_);
  if (manip > MANIP_THRESHOLD)
  {
    Eigen::MatrixXd J_JT_inv = J_JT.inverse();
    KDL::JntArrayVel current_state(robot_state_);
    KDL::Jacobian hessian_block(n_robot_joints_);

    for (std::size_t i = 0; i < n_robot_joints_; ++i)
    {
      current_state.qdot.data.setZero();
      current_state.qdot(i) = 1.0;

      robot_jacobian_dot_solver_->JntToJacDot(current_state, hessian_block);
      manip_grad[i] = (rob_jac.data * hessian_block.data.transpose())
                          .cwiseProduct(J_JT_inv)
                          .sum();
    }
    manip_grad *= manip * params->k_manip;
  }
  else
  {
    manip_grad.setZero();
  }
  Eigen::VectorXd q_manip = manip_grad;

  // joint limit avoidance
  Eigen::VectorXd eq = robot_state_.q.data - limits_avg_.data;
  Eigen::VectorXd eq_b = eq.array() / limits_bounds_.data.array();
  Eigen::VectorXd eq_trans = ((1 + eq_b.array()) / (1 - eq_b.array())).log();
  Eigen::VectorXd pT = 2 / ((limits_bounds_.data + eq).array() *
                            (limits_bounds_.data - eq).array());

  Eigen::VectorXd qd_lim = -params->k_limits * pT.array() * eq_trans.array();
  Eigen::VectorXd lambda = eq_b.cwiseAbs();

  Eigen::VectorXd h =
      (lambda.array() * qd_lim.array()) +
      ((1 - lambda.array()) * q_manip.array());  // secondary task command

  /* control */
  Eigen::MatrixXd Jr =
      coord_jac.data.block(0, n_pos_joints_, 5, n_robot_joints_);
  Eigen::MatrixXd Jr_pinv = Jr.transpose() * (Jr * Jr.transpose()).inverse();
  Eigen::MatrixXd Jp = coord_jac.data.block(0, 0, 5, n_pos_joints_);

  Eigen::MatrixXd I =
      Eigen::MatrixXd::Identity(n_robot_joints_, n_robot_joints_);
  Eigen::VectorXd q_dot_pos =
      positioner_state_.readFromRT()->qdot.data.reverse();

  Eigen::VectorXd joint_cmd =
      Jr_pinv * (cart_cmd - Jp * q_dot_pos) + ((I - Jr_pinv * Jr) * h);

  auto new_position = robot_state_.q.data + (joint_cmd * period.toSec());
  for (unsigned int i = 0; i < n_robot_joints_; ++i)
  {
    joint_handles_[i].setCommand(new_position[i]);
  }

  /* desired positioner command */
  Eigen::VectorXd robot_qdot_attempt = (home_config_ - robot_state_.q.data);

  // Eigen::MatrixXd Jp_pinv =
  //     Jp.transpose() *
  //     (Jp * Jp.transpose() + params->alpha * params->alpha * identity5x5)
  //         .inverse();

  Eigen::MatrixXd Jp_pinv = (Jp.transpose() * Jp).inverse() * Jp.transpose();
  Eigen::VectorXd pos_setpoint = Jp_pinv * (cart_cmd - Jr * robot_qdot_attempt);

  pos_setpoint = pos_setpoint.reverse();
  if (positioner_setpoint_pub_->trylock())
  {
    Eigen::VectorXd::Map(&positioner_setpoint_pub_->msg_.velocity[0],
                         pos_setpoint.size()) = pos_setpoint;
    positioner_setpoint_pub_->unlockAndPublish();
  }

  /* diagnostics */
  if (diagnostics_pub_->trylock())
  {
    diagnostics_pub_->msg_.tracking_error_x = pos_error[0];
    diagnostics_pub_->msg_.tracking_error_y = pos_error[1];
    diagnostics_pub_->msg_.tracking_error_z = pos_error[2];
    diagnostics_pub_->msg_.tracking_error_norm = pos_error.norm();
    diagnostics_pub_->msg_.manipulability = manip;
    diagnostics_pub_->msg_.aiming_error = rot_angle;
    diagnostics_pub_->unlockAndPublish();
  }
}

void CoordinatedNullspaceController::starting(const ros::Time&)
{
  synchronizeJointStates();

  KDL::JntArray combined_positions(n_robot_joints_ + n_pos_joints_);
  combined_positions.data << positioner_state_.readFromRT()->q.data.reverse(),
      robot_state_.q.data;

  Setpoint init_setpoint;
  coordinated_fk_solver_->JntToCart(combined_positions, init_setpoint.pose);

  setpoint_.initRT(init_setpoint);

  positioner_setpoint_pub_->msg_.coordinated = true;
}

void CoordinatedNullspaceController::stopping(const ros::Time&)
{
  positioner_setpoint_pub_->msg_.coordinated = false;
  positioner_setpoint_pub_->unlockAndPublish();
}

void CoordinatedNullspaceController::synchronizeJointStates()
{
  // Synchronize the internal state with the hardware
  for (unsigned int i = 0; i < n_robot_joints_; ++i)
  {
    robot_state_.q(i) = joint_handles_[i].getPosition();
    robot_state_.qdot(i) = joint_handles_[i].getVelocity();
  }
}

void CoordinatedNullspaceController::posJointStateCallback(
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

void CoordinatedNullspaceController::setpointCallback(
    const coordinated_control_msgs::AxiallySymmetricSetpointConstPtr& msg)
{
  Setpoint setpoint;

  setpoint.pose.p = KDL::Vector(msg->pose.position.x, msg->pose.position.y,
                                msg->pose.position.z);

  setpoint.pose.M = KDL::Rotation::Quaternion(
      msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z,
      msg->pose.orientation.w);

  setpoint.velocity << msg->velocity.x, msg->velocity.y, msg->velocity.z;

  setpoint_.writeFromNonRT(setpoint);
}

void CoordinatedNullspaceController::reconfCallback(ControllerConfig& config,
                                                    uint16_t /*level*/)
{
  DynamicParams dynamic_params;
  dynamic_params.alpha = config.alpha;
  dynamic_params.k_position = config.k_position;
  dynamic_params.k_aiming = config.k_aiming;
  dynamic_params.k_manip = config.k_manip;
  dynamic_params.k_limits = config.k_limits;

  dynamic_params_.writeFromNonRT(dynamic_params);
}

bool CoordinatedNullspaceController::queryPoseService(
    coordinated_control_msgs::QueryPose::Request& req,
    coordinated_control_msgs::QueryPose::Response& resp)
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

PLUGINLIB_EXPORT_CLASS(
    coordinated_motion_controllers::CoordinatedNullspaceController,
    controller_interface::ControllerBase)

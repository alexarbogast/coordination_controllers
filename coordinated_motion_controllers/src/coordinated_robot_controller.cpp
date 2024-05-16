#include <coordinated_motion_controllers/coordinated_robot_controller.h>

#include <Eigen/Dense>
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <pluginlib/class_list_macros.h>

namespace coordinated_motion_controllers
{

static const Eigen::Matrix<double, 6, 6> identity6x6 =
    Eigen::Matrix<double, 6, 6>::Identity();

Setpoint::Setpoint()
  : position(Eigen::Vector3d::Zero())
  , aiming(Eigen::Vector3d::Zero())
  , velocity(Eigen::Vector3d::Zero())
{
}

Setpoint::Setpoint(const Eigen::Vector3d& position,
                   const Eigen::Vector3d& aiming,
                   const Eigen::Vector3d& velocity)
  : position(position), aiming(aiming), velocity(velocity)
{
}

bool CoordinatedRobotController::init(
    hardware_interface::PositionJointInterface* hw, ros::NodeHandle& nh)
{
  std::string ns = nh.getNamespace();

  // Load robot description and link names
  std::string robot_description;
  if (!ros::param::search("robot_description", robot_description))
  {
    ROS_ERROR("robot_description not found in enclosing namespaces");
    return false;
  }
  if (!nh.getParam(robot_description, robot_description))
  {
    ROS_ERROR_STREAM("Failed to load " << robot_description
                                       << " from parameter server");
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
    ROS_FATAL("Failed to parse KDL tree from  urdf model");
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

  coordinated_fk_solver_ =
      std::make_unique<KDL::ChainFkSolverPos_recursive>(coordinated_chain_);
  robot_fk_solver_ =
      std::make_unique<KDL::ChainFkSolverPos_recursive>(robot_chain_);

  // Get joint handles from hardware interface
  std::vector<std::string> joint_names;
  if (!nh.getParam("joints", joint_names))
  {
    ROS_ERROR_STREAM("Failed to load " << ns << "/joints"
                                       << " from parameter server");
    return false;
  }
  n_robot_joints_ = joint_names.size();

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

  robot_position_.data = Eigen::VectorXd::Zero(n_robot_joints_);
  robot_velocity_.data = Eigen::VectorXd::Zero(n_robot_joints_);

  posJointStateCallback(pos_joint_state);

  // Find setpoint topic
  std::string setpoint_topic;
  if (!nh.getParam("setpoint_topic", setpoint_topic))
  {
    ROS_ERROR_STREAM("Failed to load " << ns << "/setpoint_topic"
                                       << " from parameter server");
    return false;
  }

  // Read controller parameters
  alpha_ = 0.1;
  k_position_ = 1.0;
  k_aiming_ = 1.0;

  nh.getParam("alpha", alpha_);
  nh.getParam("gains/position", k_position_);
  nh.getParam("gains/aiming", k_aiming_);

  std::vector<double> aiming_vec;
  if (!nh.getParam("aiming_vec", aiming_vec))
  {
    ROS_ERROR_STREAM("Failed to load " << ns << "/aiming_vec"
                                       << " from parameter server");
    return false;
  }
  aiming_vec_ = Eigen::Vector3d(aiming_vec.data());

  // Setup ROS components
  sub_positioner_joint_states_ =
      nh.subscribe(positioner_topic, 1,
                   &CoordinatedRobotController::posJointStateCallback, this);

  sub_setpoint_ = nh.subscribe(
      setpoint_topic, 1, &CoordinatedRobotController::setpointCallback, this);
  return true;
}

void CoordinatedRobotController::update(const ros::Time&,
                                        const ros::Duration& period)
{
  synchronizeJointStates();  // update state
  baseFrameControl(period);

  // Eigen::Matrix<double, 6, 1> setpoint;
  // setpoint << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  // KDL::JntArray combined_position(n_robot_joints_ + n_pos_joints_);
  // combined_position.data << positioner_position_.readFromRT()->data,
  //     robot_position_.data;
  //
  // KDL::Jacobian jac(combined_position.rows());
  // coordinated_jacobian_solver_->JntToJac(combined_position, jac);
  //
  // Eigen::MatrixXd Jr = jac.data.block(0, n_pos_joints_, 6, n_robot_joints_);
  // Eigen::MatrixXd Jp = jac.data.block(0, 0, 6, n_pos_joints_);

  // Eigen::Matrix<double, 6, 1> cmd =
  //    (Jr.transpose() *
  //     (Jr * Jr.transpose() + alpha * alpha * identity6x6).inverse()) *
  //    (setpoint - Jp * positioner_velocity_.readFromRT()->data);

  // auto new_position = robot_position_.data + (cmd * period.toSec());
  //
  // for (unsigned int i = 0; i < n_robot_joints_; ++i)
  // {
  //   joint_handles_[i].setCommand(new_position[i]);
  // }
}

void CoordinatedRobotController::starting(const ros::Time&)
{
  synchronizeJointStates();

  // initialize setpoint pose with current position
  KDL::Frame init_pose;
  robot_fk_solver_->JntToCart(robot_position_, init_pose);

  Setpoint init_setpoint;
  init_setpoint.position = Eigen::Vector3d(init_pose.p.data);
  init_setpoint.velocity = Eigen::Vector3d::Zero();
  init_setpoint.aiming =
      Eigen::Matrix3d(init_pose.M.Inverse().data) * aiming_vec_;
  setpoint_.initRT(init_setpoint);
}

void CoordinatedRobotController::stopping(const ros::Time&) {}

void CoordinatedRobotController::coordinatedControl(const ros::Duration& period)
{
}

void CoordinatedRobotController::baseFrameControl(const ros::Duration& period)
{
  Setpoint* setpoint = setpoint_.readFromRT();

  KDL::Jacobian jac(n_robot_joints_);
  robot_jacobian_solver_->JntToJac(robot_position_, jac);

  KDL::Frame pose;
  robot_fk_solver_->JntToCart(robot_position_, pose);

  // orientation error
  Eigen::Vector3d aiming_base =
      Eigen::Matrix3d(pose.M.Inverse().data) * aiming_vec_;

  Eigen::Vector3d rot_axis = aiming_base.cross(setpoint->aiming).normalized();
  double rot_angle = acos(aiming_base.dot(setpoint->aiming) /
                          (aiming_base.norm() * setpoint->aiming.norm()));
  rot_axis *= rot_angle;

  // control
  Eigen::Matrix<double, 5, 1> cart_cmd;
  cart_cmd << 0.0, 0.0, 0.0, rot_axis.x(), rot_axis.y();

  Eigen::MatrixXd Jr = jac.data.block(0, 0, 6, n_robot_joints_);

  Eigen::Matrix<double, 6, 1> joint_cmd =
      (Jr.transpose() *
       (Jr * Jr.transpose() + alpha_ * alpha_ * identity6x6).inverse()) *
      cart_cmd;

  auto new_position = robot_position_.data + (joint_cmd * period.toSec());
  for (unsigned int i = 0; i < n_robot_joints_; ++i)
  {
    joint_handles_[i].setCommand(new_position[i]);
  }
}

void CoordinatedRobotController::synchronizeJointStates()
{
  // Synchronize the internal state with the hardware
  for (unsigned int i = 0; i < n_robot_joints_; ++i)
  {
    robot_position_(i) = joint_handles_[i].getPosition();
    robot_velocity_(i) = joint_handles_[i].getVelocity();
  }
}

void CoordinatedRobotController::posJointStateCallback(
    const sensor_msgs::JointStateConstPtr& msg)
{
  KDL::JntArray position(n_pos_joints_);
  KDL::JntArray velocity(n_pos_joints_);
  for (size_t i = 0; i < n_pos_joints_; ++i)
  {
    position(i) = msg->position[i];
    velocity(i) = msg->velocity[i];
  }
  positioner_position_.writeFromNonRT(position);
  positioner_velocity_.writeFromNonRT(velocity);
}

void CoordinatedRobotController::setpointCallback(
    const coordinated_control_msgs::SetpointConstPtr& msg)
{
  Setpoint new_setpoint;
  // clang-format off
  new_setpoint.position << msg->pose.position.x,
                           msg->pose.position.y,
                           msg->pose.position.z;

  new_setpoint.aiming << msg->pose.aiming.x,
                         msg->pose.aiming.y,
                         msg->pose.aiming.z;

  new_setpoint.velocity << msg->velocity.x,
                           msg->velocity.y,
                           msg->velocity.z;
  // clang-format on
  setpoint_.writeFromNonRT(new_setpoint);
}

}  // namespace coordinated_motion_controllers

PLUGINLIB_EXPORT_CLASS(
    coordinated_motion_controllers::CoordinatedRobotController,
    controller_interface::ControllerBase)

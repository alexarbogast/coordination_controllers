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

static const Eigen::Matrix<double, 5, 5> identity5x5 =
    Eigen::Matrix<double, 5, 5>::Identity();

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
  // baseFrameControl(period);
  coordinatedControl(period);
}

void CoordinatedRobotController::starting(const ros::Time&)
{
  synchronizeJointStates();

  // initialize setpoint pose with current position
  // TODO: switch initialization based on coordinated or bf control
  static bool coordinated = true;

  KDL::Frame init_pose_bf;
  robot_fk_solver_->JntToCart(robot_position_, init_pose_bf);

  Setpoint init_setpoint;
  init_setpoint.velocity = Eigen::Vector3d::Zero();
  init_setpoint.aiming =
      Eigen::Matrix3d(init_pose_bf.M.Inverse().data) * aiming_vec_;

  if (coordinated)
  {
    KDL::JntArray combined_positions(n_robot_joints_ + n_pos_joints_);
    combined_positions.data << positioner_position_.readFromRT()->data,
        robot_position_.data;

    KDL::Frame init_pose_pf;
    coordinated_fk_solver_->JntToCart(combined_positions, init_pose_pf);
    init_setpoint.position = Eigen::Vector3d(init_pose_pf.p.data);
  }
  else
  {
    init_setpoint.position = Eigen::Vector3d(init_pose_bf.p.data);
  }

  setpoint_.initRT(init_setpoint);
}

void CoordinatedRobotController::stopping(const ros::Time&) {}

void CoordinatedRobotController::coordinatedControl(const ros::Duration& period)
{
  Setpoint* setpoint = setpoint_.readFromRT();

  KDL::JntArray combined_positions(n_robot_joints_ + n_pos_joints_);
  combined_positions.data << positioner_position_.readFromRT()->data,
      robot_position_.data;

  KDL::Jacobian rob_jac(n_robot_joints_);
  KDL::Jacobian coord_jac(n_robot_joints_ + n_pos_joints_);
  robot_jacobian_solver_->JntToJac(robot_position_, rob_jac);
  coordinated_jacobian_solver_->JntToJac(combined_positions, coord_jac);

  KDL::Frame pose_pf, pose_bf;
  robot_fk_solver_->JntToCart(robot_position_, pose_bf);
  coordinated_fk_solver_->JntToCart(combined_positions, pose_pf);

  // orientation error
  Eigen::Vector3d aiming_bf =
      Eigen::Matrix3d(pose_bf.M.Inverse().data) * aiming_vec_;

  Eigen::Vector3d rot_axis = aiming_bf.cross(setpoint->aiming).normalized();
  double rot_angle = acos(aiming_bf.dot(setpoint->aiming) /
                          (aiming_bf.norm() * setpoint->aiming.norm()));

  Eigen::Vector2d orient_error(rot_axis.x(), rot_axis.y());
  orient_error *= rot_angle;

  // position error
  Eigen::Vector3d pos_error =
      setpoint->position - Eigen::Vector3d(pose_pf.p.data);

  // control
  Eigen::Matrix<double, 5, 1> cart_cmd;
  cart_cmd << k_position_ * pos_error + setpoint->velocity,
      k_aiming_ * orient_error;

  Eigen::MatrixXd Jr =
      coord_jac.data.block(0, n_pos_joints_, 5, n_robot_joints_);
  Jr.block(3, 0, 2, n_robot_joints_) =
      rob_jac.data.block(3, 0, 2, n_robot_joints_);

  Eigen::MatrixXd Jp = coord_jac.data.block(0, 0, 5, n_pos_joints_);
  Jp.block(3, 0, 2, n_pos_joints_) = Eigen::Vector2d::Zero();

  Eigen::Matrix<double, 6, 1> joint_cmd =
      (Jr.transpose() *
       (Jr * Jr.transpose() + alpha_ * alpha_ * identity5x5).inverse()) *
      (cart_cmd - Jp * positioner_velocity_.readFromRT()->data);

  auto new_position = robot_position_.data + (joint_cmd * period.toSec());
  for (unsigned int i = 0; i < n_robot_joints_; ++i)
  {
    joint_handles_[i].setCommand(new_position[i]);
  }
}

void CoordinatedRobotController::baseFrameControl(const ros::Duration& period)
{
  Setpoint* setpoint = setpoint_.readFromRT();

  KDL::Jacobian jac(n_robot_joints_);
  robot_jacobian_solver_->JntToJac(robot_position_, jac);

  KDL::Frame pose;
  robot_fk_solver_->JntToCart(robot_position_, pose);

  // orientation error
  Eigen::Vector3d aiming_b =
      Eigen::Matrix3d(pose.M.Inverse().data) * aiming_vec_;

  Eigen::Vector3d rot_axis = aiming_b.cross(setpoint->aiming).normalized();
  double rot_angle = acos(aiming_b.dot(setpoint->aiming) /
                          (aiming_b.norm() * setpoint->aiming.norm()));

  Eigen::Vector2d orient_error(rot_axis.x(), rot_axis.y());
  orient_error *= rot_angle;

  // position error
  Eigen::Vector3d pos_error = setpoint->position - Eigen::Vector3d(pose.p.data);

  // control
  Eigen::Matrix<double, 5, 1> cart_cmd;
  cart_cmd << k_position_ * pos_error + setpoint->velocity,
      k_aiming_ * orient_error;

  Eigen::MatrixXd Jr = jac.data.block(0, 0, 5, n_robot_joints_);

  Eigen::Matrix<double, 6, 1> joint_cmd =
      (Jr.transpose() *
       (Jr * Jr.transpose() + alpha_ * alpha_ * identity5x5).inverse()) *
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

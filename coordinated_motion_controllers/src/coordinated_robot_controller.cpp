#include <coordinated_motion_controllers/coordinated_robot_controller.h>

#include <Eigen/Dense>
#include <urdf/model.h>
#include <kdl/chain.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <pluginlib/class_list_macros.h>

namespace coordinated_motion_controllers
{

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
    ROS_ERROR_STREAM("Failed to load " << nh.getNamespace() + "/robot_base_link"
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
      std::make_unique<KDL::ChainFkSolverVel_recursive>(coordinated_chain_);
  robot_fk_solver_ =
      std::make_unique<KDL::ChainFkSolverVel_recursive>(robot_chain_);

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

  // Setup ROS components
  sub_positioner_joint_states_ =
      nh.subscribe(positioner_topic, 1,
                   &CoordinatedRobotController::posJointStateCallback, this);

  return true;
}

void CoordinatedRobotController::update(const ros::Time&,
                                        const ros::Duration& period)
{
  static const double alpha = 0.1;  // pinv damping value
  static const Eigen::Matrix<double, 6, 6> identity6x6 =
      Eigen::Matrix<double, 6, 6>::Identity();

  synchronizeJointStates();

  Eigen::Matrix<double, 6, 1> setpoint;
  setpoint << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  KDL::JntArray combined_position(n_robot_joints_ + n_pos_joints_);
  combined_position.data << positioner_position_.readFromRT()->data,
      robot_position_.data;

  KDL::Jacobian jac(combined_position.rows());
  coordinated_jacobian_solver_->JntToJac(combined_position, jac);

  Eigen::MatrixXd Jr = jac.data.block(0, n_pos_joints_, 6, n_robot_joints_);
  Eigen::MatrixXd Jp = jac.data.block(0, 0, 6, n_pos_joints_);

  Eigen::Matrix<double, 6, 1> cmd =
      (Jr.transpose() *
       (Jr * Jr.transpose() + alpha * alpha * identity6x6).inverse()) *
      (setpoint - Jp * positioner_velocity_.readFromRT()->data);

  auto new_position = robot_position_.data + (cmd * period.toSec());

  for (unsigned int i = 0; i < n_robot_joints_; ++i)
  {
    joint_handles_[i].setCommand(new_position[i]);
  }
}

void CoordinatedRobotController::starting(const ros::Time&)
{
  synchronizeJointStates();
}

void CoordinatedRobotController::stopping(const ros::Time&) {}

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

}  // namespace coordinated_motion_controllers

PLUGINLIB_EXPORT_CLASS(
    coordinated_motion_controllers::CoordinatedRobotController,
    controller_interface::ControllerBase)

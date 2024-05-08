#include <coordinated_motion_controllers/coordinated_robot_controller.h>

#include <Eigen/Dense>
#include <urdf/model.h>
#include <kdl/chain.hpp>
#include <kdl/jacobian.hpp>
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
  n_joints_ = joint_names.size();

  joint_handles_.resize(n_joints_);
  for (size_t i = 0; i < n_joints_; ++i)
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

  // Initialize state
  robot_position_.data = Eigen::VectorXd::Zero(n_joints_);
  positioner_position_.data =
      Eigen::VectorXd::Zero(1);  // TODO: find # of positioner joints

  robot_velocity_.data = Eigen::VectorXd::Zero(n_joints_);
  positioner_velocity_.data = Eigen::VectorXd::Zero(1);
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
  setpoint << -0.1, -0.05, 0.01, 0.0, 0.0, 0.01;

  KDL::Jacobian jac(n_joints_);
  robot_jacobian_solver_->JntToJac(robot_position_, jac);

  Eigen::Matrix<double, 6, 1> cmd =
      jac.data.transpose() *
      (jac.data * jac.data.transpose() + alpha * alpha * identity6x6)
          .inverse() *
      setpoint;

  auto new_position = robot_position_.data + (cmd * period.toSec());

  for (unsigned int i = 0; i < n_joints_; ++i)
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
  // Read feedback
  for (unsigned int i = 0; i < n_joints_; ++i)
  {
    robot_position_(i) = joint_handles_[i].getPosition();
    robot_velocity_(i) = joint_handles_[i].getVelocity();
  }
}

}  // namespace coordinated_motion_controllers
//

PLUGINLIB_EXPORT_CLASS(
    coordinated_motion_controllers::CoordinatedRobotController,
    controller_interface::ControllerBase)

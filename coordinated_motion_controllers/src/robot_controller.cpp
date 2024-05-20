#include <coordinated_motion_controllers/robot_controller.h>

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

bool RobotController::init(hardware_interface::PositionJointInterface* hw,
                           ros::NodeHandle& nh)
{
  const std::string ns = nh.getNamespace();

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
  if (!kdl_tree.getChain(base_link_, eef_link_, robot_chain_))
  {
    ROS_FATAL_STREAM("Failed to build kinematic chain from '"
                     << base_link_ << "' to '" << eef_link_
                     << "'. Make sure these links exist in the URDF.");
    return false;
  }

  robot_jacobian_solver_ =
      std::make_unique<KDL::ChainJntToJacSolver>(robot_chain_);

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

  robot_state_.resize(n_joints_);

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

  sub_setpoint_ =
      nh.subscribe(setpoint_topic, 1, &RobotController::setpointCallback, this);

  // Dynamic reconfigure
  dyn_reconf_server_ = std::make_shared<ReconfigureServer>(nh);
  dyn_reconf_server_->setCallback(std::bind(&RobotController::reconfCallback,
                                            this, std::placeholders::_1,
                                            std::placeholders::_2));

  return true;
}

void RobotController::update(const ros::Time&, const ros::Duration& period)
{
  synchronizeJointStates();  // update state

  const DynamicParams* params = dynamic_params_.readFromRT();
  const Setpoint* setpoint = setpoint_.readFromRT();

  KDL::Jacobian jac(n_joints_);
  robot_jacobian_solver_->JntToJac(robot_state_.q, jac);

  KDL::Frame pose;
  robot_fk_solver_->JntToCart(robot_state_.q, pose);

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
  cart_cmd << params->k_position * pos_error + setpoint->velocity,
      params->k_aiming * orient_error;

  Eigen::MatrixXd Jr = jac.data.block(0, 0, 5, n_joints_);

  Eigen::MatrixXd Jr_pinv =
      Jr.transpose() *
      (Jr * Jr.transpose() + params->alpha * params->alpha * identity5x5)
          .inverse();
  Eigen::VectorXd joint_cmd = Jr_pinv * cart_cmd;

  auto new_position = robot_state_.q.data + (joint_cmd * period.toSec());
  for (unsigned int i = 0; i < n_joints_; ++i)
  {
    joint_handles_[i].setCommand(new_position[i]);
  }
}

void RobotController::starting(const ros::Time&)
{
  synchronizeJointStates();

  KDL::Frame init_pose_bf;
  robot_fk_solver_->JntToCart(robot_state_.q, init_pose_bf);

  Setpoint init_setpoint;
  init_setpoint.velocity = Eigen::Vector3d::Zero();
  init_setpoint.aiming =
      Eigen::Matrix3d(init_pose_bf.M.Inverse().data) * aiming_vec_;
  init_setpoint.position = Eigen::Vector3d(init_pose_bf.p.data);

  setpoint_.initRT(init_setpoint);
}

void RobotController::stopping(const ros::Time&) {}

void RobotController::synchronizeJointStates()
{
  // Synchronize the internal state with the hardware
  for (unsigned int i = 0; i < n_joints_; ++i)
  {
    robot_state_.q(i) = joint_handles_[i].getPosition();
    robot_state_.qdot(i) = joint_handles_[i].getVelocity();
  }
}

void RobotController::setpointCallback(
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

void RobotController::reconfCallback(CoordinatedControllerConfig& config,
                                     uint16_t /*level*/)
{
  DynamicParams dynamic_params;
  dynamic_params.alpha = config.alpha;
  dynamic_params.k_position = config.k_position;
  dynamic_params.k_aiming = config.k_aiming;

  dynamic_params_.writeFromNonRT(dynamic_params);
}

}  // namespace coordinated_motion_controllers

PLUGINLIB_EXPORT_CLASS(coordinated_motion_controllers::RobotController,
                       controller_interface::ControllerBase)

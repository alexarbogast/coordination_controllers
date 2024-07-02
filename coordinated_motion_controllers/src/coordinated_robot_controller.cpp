#include <coordinated_motion_controllers/coordinated_robot_controller.h>
#include <coordinated_motion_controllers/utility.h>

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

static double MANIP_THRESHOLD = 1e-6;

bool CoordinatedRobotController::init(
    hardware_interface::PositionJointInterface* hw, ros::NodeHandle& nh)
{
  const std::string ns = nh.getNamespace();

  // Load robot description and link names
  std::string robot_description;
  if (!ros::param::search("robot_description", robot_description))
  {
    ROS_ERROR("robot_description not found in enclosing namespaces");
    return false;
  }
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
  robot_jacobian_dot_solver_ =
      std::make_unique<KDL::ChainJntToJacDotSolver>(robot_chain_);

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

  // Setup ROS components
  sub_positioner_joint_states_ =
      nh.subscribe(positioner_topic, 1,
                   &CoordinatedRobotController::posJointStateCallback, this);

  sub_setpoint_ = nh.subscribe(
      setpoint_topic, 1, &CoordinatedRobotController::setpointCallback, this);

  // Dynamic reconfigure
  dyn_reconf_server_ = std::make_shared<ReconfigureServer>(nh);
  dyn_reconf_server_->setCallback(
      std::bind(&CoordinatedRobotController::reconfCallback, this,
                std::placeholders::_1, std::placeholders::_2));

  return true;
}

void CoordinatedRobotController::update(const ros::Time&,
                                        const ros::Duration& period)
{
  synchronizeJointStates();  // update state

  const DynamicParams* params = dynamic_params_.readFromRT();
  const Setpoint* setpoint = setpoint_.readFromRT();

  KDL::JntArray combined_positions(n_robot_joints_ + n_pos_joints_);
  combined_positions.data << positioner_state_.readFromRT()->q.data,
      robot_state_.q.data;

  KDL::Jacobian rob_jac(n_robot_joints_);
  KDL::Jacobian coord_jac(n_robot_joints_ + n_pos_joints_);
  robot_jacobian_solver_->JntToJac(robot_state_.q, rob_jac);
  coordinated_jacobian_solver_->JntToJac(combined_positions, coord_jac);

  KDL::Frame pose_pf, pose_bf;
  robot_fk_solver_->JntToCart(robot_state_.q, pose_bf);
  coordinated_fk_solver_->JntToCart(combined_positions, pose_pf);

  /* orientation error */
  Eigen::Vector3d aiming_bf =
      Eigen::Matrix3d(pose_bf.M.Inverse().data) * aiming_vec_;

  Eigen::Vector3d rot_axis = axisBetween(aiming_bf, setpoint->aiming);
  double rot_angle = angleBetween(aiming_bf, setpoint->aiming);

  Eigen::Vector2d orient_error(rot_axis.x(), rot_axis.y());
  orient_error *= rot_angle;

  /* position error */
  Eigen::Vector3d pos_error =
      setpoint->position - Eigen::Vector3d(pose_pf.p.data);

  /* redundancy resolution */
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

  /* control */
  Eigen::Matrix<double, 5, 1> cart_cmd;
  cart_cmd << params->k_position * pos_error + setpoint->velocity,
      params->k_aiming * orient_error;

  Eigen::MatrixXd Jr =
      coord_jac.data.block(0, n_pos_joints_, 5, n_robot_joints_);
  Jr.block(3, 0, 2, n_robot_joints_) =
      rob_jac.data.block(3, 0, 2, n_robot_joints_);

  Eigen::MatrixXd Jr_pinv =
      Jr.transpose() *
      (Jr * Jr.transpose() + params->alpha * params->alpha * identity5x5)
          .inverse();

  Eigen::MatrixXd Jp = coord_jac.data.block(0, 0, 5, n_pos_joints_);
  Jp.block(3, 0, 2, n_pos_joints_) = Eigen::Vector2d::Zero();

  Eigen::MatrixXd I =
      Eigen::MatrixXd::Identity(n_robot_joints_, n_robot_joints_);
  Eigen::VectorXd joint_cmd =
      Jr_pinv * (cart_cmd - Jp * positioner_state_.readFromRT()->qdot.data) +
      ((I - Jr_pinv * Jr) * manip_grad);

  auto new_position = robot_state_.q.data + (joint_cmd * period.toSec());
  for (unsigned int i = 0; i < n_robot_joints_; ++i)
  {
    joint_handles_[i].setCommand(new_position[i]);
  }
}

void CoordinatedRobotController::starting(const ros::Time&)
{
  synchronizeJointStates();

  KDL::Frame init_pose_bf;
  robot_fk_solver_->JntToCart(robot_state_.q, init_pose_bf);

  Setpoint init_setpoint;
  init_setpoint.velocity = Eigen::Vector3d::Zero();
  init_setpoint.aiming =
      Eigen::Matrix3d(init_pose_bf.M.Inverse().data) * aiming_vec_;

  KDL::JntArray combined_positions(n_robot_joints_ + n_pos_joints_);
  combined_positions.data << positioner_state_.readFromRT()->q.data,
      robot_state_.q.data;

  KDL::Frame init_pose_pf;
  coordinated_fk_solver_->JntToCart(combined_positions, init_pose_pf);
  init_setpoint.position = Eigen::Vector3d(init_pose_pf.p.data);

  setpoint_.initRT(init_setpoint);
}

void CoordinatedRobotController::stopping(const ros::Time&) {}

void CoordinatedRobotController::synchronizeJointStates()
{
  // Synchronize the internal state with the hardware
  for (unsigned int i = 0; i < n_robot_joints_; ++i)
  {
    robot_state_.q(i) = joint_handles_[i].getPosition();
    robot_state_.qdot(i) = joint_handles_[i].getVelocity();
  }
}

void CoordinatedRobotController::posJointStateCallback(
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

void CoordinatedRobotController::reconfCallback(
    CoordinatedControllerConfig& config, uint16_t /*level*/)
{
  DynamicParams dynamic_params;
  dynamic_params.alpha = config.alpha;
  dynamic_params.k_position = config.k_position;
  dynamic_params.k_aiming = config.k_aiming;
  dynamic_params.k_manip = config.k_manip;

  dynamic_params_.writeFromNonRT(dynamic_params);
}

}  // namespace coordinated_motion_controllers

PLUGINLIB_EXPORT_CLASS(
    coordinated_motion_controllers::CoordinatedRobotController,
    controller_interface::ControllerBase)

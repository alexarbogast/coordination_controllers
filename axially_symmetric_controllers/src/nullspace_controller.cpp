#include <axially_symmetric_controllers/nullspace_controller.h>
#include <axially_symmetric_controllers/utility.h>

#include <urdf/model.h>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/tree.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <pluginlib/class_list_macros.h>

namespace axially_symmetric_controllers
{

static const Eigen::Matrix<double, 5, 5> identity5x5 =
    Eigen::Matrix<double, 5, 5>::Identity();

static double MANIP_THRESHOLD = 1e-10;

bool NullspaceController::init(hardware_interface::PositionJointInterface* hw,
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
  if (!kdl_tree.getChain(base_link_, eef_link_, robot_chain_))
  {
    ROS_FATAL_STREAM("Failed to build kinematic chain from '"
                     << base_link_ << "' to '" << eef_link_
                     << "'. Make sure these links exist in the URDF.");
    return false;
  }

  robot_jacobian_solver_ =
      std::make_unique<KDL::ChainJntToJacSolver>(robot_chain_);

  robot_jacobian_dot_solver_ =
      std::make_unique<KDL::ChainJntToJacDotSolver>(robot_chain_);

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
  n_joints_ = joint_names.size();

  KDL::JntArray upper_pos_limits(n_joints_);
  KDL::JntArray lower_pos_limits(n_joints_);
  for (size_t i = 0; i < n_joints_; ++i)
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

  limits_avg_.resize(n_joints_);
  limits_avg_.data = (upper_pos_limits.data + lower_pos_limits.data) / 2;

  limits_bounds_.resize(n_joints_);
  limits_bounds_.data = (upper_pos_limits.data - lower_pos_limits.data) / 2;

  // Get joint handles from hardware interface
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

  sub_setpoint_ = nh.subscribe(setpoint_topic, 1,
                               &NullspaceController::setpointCallback, this);

  // Dynamic reconfigure
  dyn_reconf_server_ = std::make_shared<ReconfigureServer>(nh);
  dyn_reconf_server_->setCallback(
      std::bind(&NullspaceController::reconfCallback, this,
                std::placeholders::_1, std::placeholders::_2));

  // Services
  query_pose_service_ = nh.advertiseService(
      "query_pose", &NullspaceController::queryPoseService, this);

  return true;
}

void NullspaceController::update(const ros::Time&, const ros::Duration& period)
{
  synchronizeJointStates();  // update state

  const DynamicParams* params = dynamic_params_.readFromRT();
  const AxiallySymmetricSetpoint* setpoint = setpoint_.readFromRT();

  KDL::Jacobian jac(n_joints_);
  robot_jacobian_solver_->JntToJac(robot_state_.q, jac);

  KDL::Frame pose;
  robot_fk_solver_->JntToCart(robot_state_.q, pose);

  /* orientation error */
  Eigen::Vector3d aiming_bf =
      Eigen::Matrix3d(pose.M.Inverse().data) * aiming_vec_;

  Eigen::Vector3d rot_axis = axisBetween(aiming_bf, setpoint->aiming);
  double rot_angle = angleBetween(aiming_bf, setpoint->aiming);

  Eigen::Vector2d orient_error(rot_axis.x(), rot_axis.y());
  orient_error *= rot_angle;

  /* position error */
  Eigen::Vector3d pos_error = setpoint->position - Eigen::Vector3d(pose.p.data);

  /* redundancy resolution */
  // manipulability maximization
  Eigen::MatrixXd J_JT = jac.data * jac.data.transpose();
  double manip = sqrt(J_JT.determinant());

  Eigen::VectorXd manip_grad(n_joints_);
  if (manip > MANIP_THRESHOLD)
  {
    Eigen::MatrixXd J_JT_inv = J_JT.inverse();
    KDL::JntArrayVel current_state(robot_state_);
    KDL::Jacobian hessian_block(n_joints_);

    for (std::size_t i = 0; i < n_joints_; ++i)
    {
      current_state.qdot.data.setZero();
      current_state.qdot(i) = 1.0;

      robot_jacobian_dot_solver_->JntToJacDot(current_state, hessian_block);
      manip_grad[i] = (jac.data * hessian_block.data.transpose())
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
  Eigen::Matrix<double, 5, 1> cart_cmd;
  cart_cmd << params->k_position * pos_error + setpoint->velocity,
      params->k_aiming * orient_error;

  Eigen::MatrixXd Jr = jac.data.block(0, 0, 5, n_joints_);
  Eigen::MatrixXd Jr_pinv =
      Jr.transpose() *
      (Jr * Jr.transpose() + params->alpha * params->alpha * identity5x5)
          .inverse();

  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n_joints_, n_joints_);
  Eigen::VectorXd joint_cmd = Jr_pinv * cart_cmd + ((I - Jr_pinv * Jr) * h);

  auto new_position = robot_state_.q.data + (joint_cmd * period.toSec());
  for (unsigned int i = 0; i < n_joints_; ++i)
  {
    joint_handles_[i].setCommand(new_position[i]);
  }
}

void NullspaceController::starting(const ros::Time&)
{
  synchronizeJointStates();

  KDL::Frame init_pose_bf;
  robot_fk_solver_->JntToCart(robot_state_.q, init_pose_bf);

  AxiallySymmetricSetpoint init_setpoint;
  init_setpoint.velocity = Eigen::Vector3d::Zero();
  init_setpoint.aiming =
      Eigen::Matrix3d(init_pose_bf.M.Inverse().data) * aiming_vec_;
  init_setpoint.position = Eigen::Vector3d(init_pose_bf.p.data);

  setpoint_.initRT(init_setpoint);
}

void NullspaceController::stopping(const ros::Time&) {}

void NullspaceController::synchronizeJointStates()
{
  // Synchronize the internal state with the hardware
  for (unsigned int i = 0; i < n_joints_; ++i)
  {
    robot_state_.q(i) = joint_handles_[i].getPosition();
    robot_state_.qdot(i) = joint_handles_[i].getVelocity();
  }
}

void NullspaceController::setpointCallback(
    const coordinated_control_msgs::RobotSetpointConstPtr& msg)
{
  AxiallySymmetricSetpoint new_setpoint;
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

void NullspaceController::reconfCallback(
    AxiallySymmetricControllerConfig& config, uint16_t /*level*/)
{
  DynamicParams dynamic_params;
  dynamic_params.alpha = config.alpha;
  dynamic_params.k_position = config.k_position;
  dynamic_params.k_aiming = config.k_aiming;
  dynamic_params.k_manip = config.k_manip;
  dynamic_params.k_limits = config.k_limits;

  dynamic_params_.writeFromNonRT(dynamic_params);
}

bool NullspaceController::queryPoseService(
    coordinated_control_msgs::QueryPose::Request& req,
    coordinated_control_msgs::QueryPose::Response& resp)
{
  KDL::JntArray robot_state(n_joints_);
  for (unsigned int i = 0; i < n_joints_; ++i)
  {
    robot_state(i) = joint_handles_[i].getPosition();
  }

  KDL::Frame pose;
  robot_fk_solver_->JntToCart(robot_state, pose);

  resp.pose.position.x = pose.p.x();
  resp.pose.position.y = pose.p.y();
  resp.pose.position.z = pose.p.z();

  pose.M.GetQuaternion(resp.pose.orientation.x, resp.pose.orientation.y,
                       resp.pose.orientation.z, resp.pose.orientation.w);
  return true;
}

}  // namespace axially_symmetric_controllers

PLUGINLIB_EXPORT_CLASS(axially_symmetric_controllers::NullspaceController,
                       controller_interface::ControllerBase)

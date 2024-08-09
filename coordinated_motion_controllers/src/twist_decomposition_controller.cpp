#include <coordinated_motion_controllers/twist_decomposition_controller.h>
#include <coordinated_motion_controllers/utility.h>

#include <urdf/model.h>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/tree.hpp>
#include <kdl/jacobian.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <pluginlib/class_list_macros.h>

namespace coordinated_motion_controllers
{

static const Eigen::Matrix<double, 5, 5> identity5x5 =
    Eigen::Matrix<double, 5, 5>::Identity();

static double MANIP_THRESHOLD = 1e-10;

bool TwistDecompositionController::init(
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

  sub_setpoint_ = nh.subscribe(
      setpoint_topic, 1, &TwistDecompositionController::setpointCallback, this);

  // Dynamic reconfigure
  dyn_reconf_server_ = std::make_shared<ReconfigureServer>(nh);
  dyn_reconf_server_->setCallback(
      std::bind(&TwistDecompositionController::reconfCallback, this,
                std::placeholders::_1, std::placeholders::_2));

  // Services
  query_pose_service_ = nh.advertiseService(
      "query_pose", &TwistDecompositionController::queryPoseService, this);

  return true;
}

void TwistDecompositionController::update(const ros::Time&,
                                          const ros::Duration& period)
{
  synchronizeJointStates();  // update state

  const DynamicParams* params = dynamic_params_.readFromRT();
  const TwistDecompositionSetpoint* setpoint = setpoint_.readFromRT();

  KDL::Jacobian jac(n_joints_);
  robot_jacobian_solver_->JntToJac(robot_state_.q, jac);

  KDL::Frame pose;
  robot_fk_solver_->JntToCart(robot_state_.q, pose);

  /* error */
  KDL::Frame frame_error;
  frame_error.p = setpoint->pose.p - pose.p;
  frame_error.M = setpoint->pose.M * pose.M.Inverse();

  KDL::Vector rot_axis = KDL::Vector::Zero();
  double rot_angle = frame_error.M.GetRotAngle(rot_axis);

  Eigen::Vector3d pos_error(frame_error.p.data);
  // Eigen::Vector3d rot_error((rot_angle * rot_axis).data);

  // temp rotation error
  Eigen::Quaterniond current_q;
  pose.M.GetQuaternion(current_q.x(), current_q.y(), current_q.z(),
                       current_q.w());

  Eigen::Quaterniond setpoint_q;
  setpoint->pose.M.GetQuaternion(setpoint_q.x(), setpoint_q.y(), setpoint_q.z(),
                                 setpoint_q.w());

  Eigen::Vector3d rot_error = (setpoint_q * current_q.inverse()).vec();

  Eigen::Matrix<double, 6, 1> cart_cmd;
  cart_cmd << params->k_position * pos_error + setpoint->velocity,
      params->k_aiming * rot_error;

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

  /* task decomposition */
  Eigen::Vector3d e(pose.M.UnitZ().data);
  Eigen::Matrix3d eeT = e * e.transpose();

  Eigen::Matrix<double, 6, 6> T = Eigen::Matrix<double, 6, 6>::Zero();
  T.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  T.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() - eeT;

  Eigen::Vector3d perp_cmd = eeT * jac.data.block(3, 0, 3, n_joints_) * h;

  /* control */
  Eigen::MatrixXd Jr = jac.data;
  Eigen::MatrixXd Jr_pinv = Jr.transpose() * (Jr * Jr.transpose()).inverse();

  Eigen::Matrix<double, 6, 1> mod_cart_cmd = T * cart_cmd;
  mod_cart_cmd.block<3, 1>(3, 0) += perp_cmd;

  Eigen::VectorXd joint_cmd = Jr_pinv * mod_cart_cmd;

  auto new_position = robot_state_.q.data + (joint_cmd * period.toSec());
  for (unsigned int i = 0; i < n_joints_; ++i)
  {
    joint_handles_[i].setCommand(new_position[i]);
  }
}

void TwistDecompositionController::starting(const ros::Time&)
{
  synchronizeJointStates();

  TwistDecompositionSetpoint init_setpoint;
  robot_fk_solver_->JntToCart(robot_state_.q, init_setpoint.pose);
  init_setpoint.velocity = Eigen::Vector3d::Zero();
  setpoint_.initRT(init_setpoint);
}

void TwistDecompositionController::stopping(const ros::Time&) {}

void TwistDecompositionController::synchronizeJointStates()
{
  // Synchronize the internal state with the hardware
  for (unsigned int i = 0; i < n_joints_; ++i)
  {
    robot_state_.q(i) = joint_handles_[i].getPosition();
    robot_state_.qdot(i) = joint_handles_[i].getVelocity();
  }
}

void TwistDecompositionController::setpointCallback(
    const coordinated_control_msgs::TwistDecompositionSetpointConstPtr& msg)
{
  TwistDecompositionSetpoint setpoint;

  setpoint.pose.p = KDL::Vector(msg->pose.position.x, msg->pose.position.y,
                                msg->pose.position.z);
  setpoint.pose.M = KDL::Rotation::Quaternion(
      msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z,
      msg->pose.orientation.w);

  setpoint.velocity << msg->velocity.x, msg->velocity.y, msg->velocity.z;

  setpoint_.writeFromNonRT(setpoint);
}

void TwistDecompositionController::reconfCallback(
    CoordinatedControllerConfig& config, uint16_t /*level*/)
{
  DynamicParams dynamic_params;
  dynamic_params.alpha = config.alpha;
  dynamic_params.k_position = config.k_position;
  dynamic_params.k_aiming = config.k_aiming;
  dynamic_params.k_manip = config.k_manip;
  dynamic_params.k_limits = config.k_limits;

  dynamic_params_.writeFromNonRT(dynamic_params);
}

bool TwistDecompositionController::queryPoseService(
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

}  // namespace coordinated_motion_controllers

PLUGINLIB_EXPORT_CLASS(
    coordinated_motion_controllers::TwistDecompositionController,
    controller_interface::ControllerBase)

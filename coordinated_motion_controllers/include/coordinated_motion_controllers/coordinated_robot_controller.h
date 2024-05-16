#pragma once

#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarray.hpp>

#include <sensor_msgs/JointState.h>
#include <coordinated_control_msgs/Setpoint.h>
#include <realtime_tools/realtime_buffer.h>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

namespace coordinated_motion_controllers
{

struct Setpoint
{
  Setpoint();
  Setpoint(const Eigen::Vector3d& position, const Eigen::Vector3d& aiming,
           const Eigen::Vector3d& velocity);

  Eigen::Vector3d position;
  Eigen::Vector3d aiming;
  Eigen::Vector3d velocity;
};

class CoordinatedRobotController
  : public controller_interface::Controller<
        hardware_interface::PositionJointInterface>
{
public:
  virtual bool init(hardware_interface::PositionJointInterface* hw,
                    ros::NodeHandle& nh) override;
  virtual void update(const ros::Time&, const ros::Duration& period) override;
  virtual void starting(const ros::Time&) override;
  virtual void stopping(const ros::Time&) override;

private:
  void synchronizeJointStates();
  void posJointStateCallback(const sensor_msgs::JointStateConstPtr& msg);
  void setpointCallback(const coordinated_control_msgs::SetpointConstPtr& msg);

  // control methods
  void coordinatedControl(const ros::Duration& period);
  void baseFrameControl(const ros::Duration& period);

private:
  unsigned int n_robot_joints_, n_pos_joints_;
  std::vector<hardware_interface::JointHandle> joint_handles_;
  std::string positioner_link_, base_link_, eef_link_;

  double alpha_;                  // pinv damping value
  double k_position_, k_aiming_;  // gain values
  Eigen::Vector3d aiming_vec_;

  // kinematics
  KDL::Chain coordinated_chain_, robot_chain_;
  std::unique_ptr<KDL::ChainJntToJacSolver> coordinated_jacobian_solver_;
  std::unique_ptr<KDL::ChainJntToJacSolver> robot_jacobian_solver_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> coordinated_fk_solver_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> robot_fk_solver_;

  // state
  KDL::JntArray robot_position_, robot_velocity_;
  realtime_tools::RealtimeBuffer<KDL::JntArray> positioner_position_;
  realtime_tools::RealtimeBuffer<KDL::JntArray> positioner_velocity_;

  ros::Subscriber sub_positioner_joint_states_;

  // setpoint
  realtime_tools::RealtimeBuffer<Setpoint> setpoint_;
  ros::Subscriber sub_setpoint_;
};

}  // namespace coordinated_motion_controllers

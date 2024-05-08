#pragma once

#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/jntarray.hpp>

#include <sensor_msgs/JointState.h>
#include <realtime_tools/realtime_buffer.h>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

namespace coordinated_motion_controllers
{

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

private:
  unsigned int n_robot_joints_, n_pos_joints_;
  std::vector<hardware_interface::JointHandle> joint_handles_;
  std::string positioner_link_, base_link_, eef_link_;

  // kinematics
  KDL::Chain coordinated_chain_, robot_chain_;
  std::unique_ptr<KDL::ChainJntToJacSolver> coordinated_jacobian_solver_;
  std::unique_ptr<KDL::ChainJntToJacSolver> robot_jacobian_solver_;
  std::unique_ptr<KDL::ChainFkSolverVel_recursive> coordinated_fk_solver_;
  std::unique_ptr<KDL::ChainFkSolverVel_recursive> robot_fk_solver_;

  // state
  KDL::JntArray robot_position_, robot_velocity_;
  realtime_tools::RealtimeBuffer<KDL::JntArray> positioner_position_;
  realtime_tools::RealtimeBuffer<KDL::JntArray> positioner_velocity_;

  ros::Subscriber sub_positioner_joint_states_;
};

}  // namespace coordinated_motion_controllers

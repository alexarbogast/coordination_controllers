#pragma once

#include <coordinated_motion_controllers/setpoint.h>

#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarrayvel.hpp>

#include <sensor_msgs/JointState.h>
#include <coordinated_control_msgs/Setpoint.h>
#include <realtime_tools/realtime_buffer.h>
#include <dynamic_reconfigure/server.h>
#include <coordinated_motion_controllers/CoordinatedControllerConfig.h>

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
  void setpointCallback(const coordinated_control_msgs::SetpointConstPtr& msg);
  void reconfCallback(CoordinatedControllerConfig& config, uint16_t /*level*/);

private:
  unsigned int n_robot_joints_, n_pos_joints_;
  std::vector<hardware_interface::JointHandle> joint_handles_;
  std::string positioner_link_, base_link_, eef_link_;

  Eigen::Vector3d aiming_vec_;

  // kinematics
  KDL::Chain coordinated_chain_, robot_chain_;
  std::unique_ptr<KDL::ChainJntToJacSolver> coordinated_jacobian_solver_;
  std::unique_ptr<KDL::ChainJntToJacSolver> robot_jacobian_solver_;
  std::unique_ptr<KDL::ChainJntToJacDotSolver> robot_jacobian_dot_solver_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> coordinated_fk_solver_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> robot_fk_solver_;

  // state
  KDL::JntArrayVel robot_state_;

  realtime_tools::RealtimeBuffer<KDL::JntArrayVel> positioner_state_;
  ros::Subscriber sub_positioner_joint_states_;

  // setpoint
  realtime_tools::RealtimeBuffer<Setpoint> setpoint_;
  ros::Subscriber sub_setpoint_;

  // dynamic reconfigure
  struct DynamicParams
  {
    DynamicParams() : alpha(1.0), k_position(1.0), k_aiming(1.0) {}

    double alpha;       // pinv damping value
    double k_position;  // gain values
    double k_aiming;
    double k_manip;
  };
  realtime_tools::RealtimeBuffer<DynamicParams> dynamic_params_;

  typedef dynamic_reconfigure::Server<CoordinatedControllerConfig>
      ReconfigureServer;
  std::shared_ptr<ReconfigureServer> dyn_reconf_server_;
};

}  // namespace coordinated_motion_controllers

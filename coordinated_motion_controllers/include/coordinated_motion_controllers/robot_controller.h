#pragma once

#include <coordinated_motion_controllers/setpoint.h>

#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarrayvel.hpp>

#include <coordinated_control_msgs/Setpoint.h>
#include <realtime_tools/realtime_buffer.h>
#include <dynamic_reconfigure/server.h>
#include <coordinated_motion_controllers/CoordinatedControllerConfig.h>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

namespace coordinated_motion_controllers
{

class RobotController : public controller_interface::Controller<
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
  void setpointCallback(const coordinated_control_msgs::SetpointConstPtr& msg);
  void reconfCallback(CoordinatedControllerConfig& config, uint16_t /*level*/);

private:
  unsigned int n_joints_;
  std::vector<hardware_interface::JointHandle> joint_handles_;
  std::string base_link_, eef_link_;

  Eigen::Vector3d aiming_vec_;

  // kinematics
  KDL::Chain robot_chain_;
  std::unique_ptr<KDL::ChainJntToJacSolver> robot_jacobian_solver_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> robot_fk_solver_;

  // state
  KDL::JntArrayVel robot_state_;

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
  };
  realtime_tools::RealtimeBuffer<DynamicParams> dynamic_params_;

  typedef dynamic_reconfigure::Server<CoordinatedControllerConfig>
      ReconfigureServer;
  std::shared_ptr<ReconfigureServer> dyn_reconf_server_;
};

}  // namespace coordinated_motion_controllers

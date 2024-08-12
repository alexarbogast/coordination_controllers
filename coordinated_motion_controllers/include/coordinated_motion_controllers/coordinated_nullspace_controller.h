#pragma once

#include <axially_symmetric_controllers/setpoint.h>
#include <axially_symmetric_controllers/AxiallySymmetricControllerConfig.h>

#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarrayvel.hpp>

#include <sensor_msgs/JointState.h>
#include <coordinated_control_msgs/AxiallySymmetricSetpoint.h>
#include <coordinated_control_msgs/PositionerSetpoint.h>
#include <coordinated_control_msgs/QueryPose.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <dynamic_reconfigure/server.h>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

namespace coordinated_motion_controllers
{

class CoordinatedNullspaceController
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
  typedef axially_symmetric_controllers::AxiallySymmetricSetpoint Setpoint;
  typedef axially_symmetric_controllers::AxiallySymmetricControllerConfig
      ControllerConfig;
  typedef dynamic_reconfigure::Server<ControllerConfig> ReconfigureServer;

  void synchronizeJointStates();

  void reconfCallback(ControllerConfig& config, uint16_t /*level*/);
  void posJointStateCallback(const sensor_msgs::JointStateConstPtr& msg);
  void setpointCallback(
      const coordinated_control_msgs::AxiallySymmetricSetpointConstPtr& msg);
  bool queryPoseService(coordinated_control_msgs::QueryPose::Request& req,
                        coordinated_control_msgs::QueryPose::Response& resp);

private:
  unsigned int n_robot_joints_, n_pos_joints_;
  std::vector<hardware_interface::JointHandle> joint_handles_;
  std::string positioner_link_, base_link_, eef_link_;

  Eigen::Vector3d aiming_vec_;
  Eigen::VectorXd home_config_;

  // kinematics
  KDL::Chain coordinated_chain_, robot_chain_;
  std::unique_ptr<KDL::ChainJntToJacSolver> coordinated_jacobian_solver_;
  std::unique_ptr<KDL::ChainJntToJacSolver> robot_jacobian_solver_;
  std::unique_ptr<KDL::ChainJntToJacDotSolver> robot_jacobian_dot_solver_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> coordinated_fk_solver_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> robot_fk_solver_;

  // state
  KDL::JntArrayVel robot_state_;

  // limits
  KDL::JntArray limits_avg_;
  KDL::JntArray limits_bounds_;

  // positioner
  realtime_tools::RealtimeBuffer<KDL::JntArrayVel> positioner_state_;
  ros::Subscriber sub_positioner_joint_states_;
  std::unique_ptr<realtime_tools::RealtimePublisher<
      coordinated_control_msgs::PositionerSetpoint>>
      positioner_setpoint_pub_;

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
    double k_limits;
  };
  realtime_tools::RealtimeBuffer<DynamicParams> dynamic_params_;
  std::shared_ptr<ReconfigureServer> dyn_reconf_server_;

  ros::ServiceServer query_pose_service_;
};

}  // namespace coordinated_motion_controllers

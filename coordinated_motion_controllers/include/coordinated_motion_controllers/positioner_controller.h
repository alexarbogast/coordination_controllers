#pragma once

#include <unordered_map>
#include <kdl/jntarray.hpp>

#include <coordinated_control_msgs/PositionerSetpoint.h>
#include <realtime_tools/realtime_buffer.h>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

namespace coordinated_motion_controllers
{

class CoordinatedRobotData
{
public:
  CoordinatedRobotData(const std::string& controller_ns,
                       unsigned int n_pos_joints);

  realtime_tools::RealtimeBuffer<KDL::JntArray> rec_setpoint;
  std::atomic<bool> coord;

private:
  void setpointCallback(
      const coordinated_control_msgs::PositionerSetpointConstPtr& msg);

private:
  std::string controller_ns_;
  unsigned int n_pos_joints_;

  ros::NodeHandle pnh_;
  ros::Subscriber sub_rec_setpoint_;
};

class PositionerController : public controller_interface::Controller<
                                 hardware_interface::PositionJointInterface>
{
public:
  virtual bool init(hardware_interface::PositionJointInterface* hw,
                    ros::NodeHandle& nh) override;
  virtual void update(const ros::Time& time,
                      const ros::Duration& period) override;
  virtual void starting(const ros::Time&) override;
  virtual void stopping(const ros::Time&) override;

private:
  unsigned int n_joints_;
  std::vector<hardware_interface::JointHandle> joint_handles_;
  std::unordered_map<std::string, std::shared_ptr<CoordinatedRobotData>>
      robot_data_;

  ros::Subscriber sub_joint_states_;
};

}  // namespace coordinated_motion_controllers

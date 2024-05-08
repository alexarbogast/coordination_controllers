#pragma once

#include <kdl/jntarray.hpp>
#include <memory>
#include <unordered_map>

#include <sensor_msgs/JointState.h>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

namespace coordinated_motion_controllers
{

struct CoordinatedRobotData
{
  CoordinatedRobotData(const std::string& controller_ns,
                       const std::vector<std::string>& joints);

  bool init(const sensor_msgs::JointState& joint_state);

  std::string controller_ns;
  std::vector<std::string> joints;

  // state
  std::vector<size_t> joint_state_indices;
  KDL::JntArray positions, velocities;
};

class PositionerController : public controller_interface::Controller<
                                 hardware_interface::PositionJointInterface>
{
public:
  virtual bool init(hardware_interface::PositionJointInterface* hw,
                    ros::NodeHandle& nh) override;
  virtual void update(const ros::Time&, const ros::Duration& period) override;
  virtual void starting(const ros::Time&) override;
  virtual void stopping(const ros::Time&) override;

protected:
  std::vector<hardware_interface::JointHandle> joint_handles_;
  std::unordered_map<std::string, std::shared_ptr<CoordinatedRobotData>>
      robot_data_;
};

}  // namespace coordinated_motion_controllers

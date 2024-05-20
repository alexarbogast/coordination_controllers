#pragma once

#include <Eigen/Dense>

namespace coordinated_motion_controllers
{

struct Setpoint
{
  Setpoint()
    : position(Eigen::Vector3d::Zero())
    , aiming(Eigen::Vector3d::Zero())
    , velocity(Eigen::Vector3d::Zero())
  {
  }

  Eigen::Vector3d position;
  Eigen::Vector3d aiming;
  Eigen::Vector3d velocity;
};

}  // namespace coordinated_motion_controllers

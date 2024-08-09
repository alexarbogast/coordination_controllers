#pragma once

#include <Eigen/Dense>
#include <kdl/frames.hpp>

namespace coordinated_motion_controllers
{

struct AxiallySymmetricSetpoint
{
  AxiallySymmetricSetpoint()
    : position(Eigen::Vector3d::Zero())
    , aiming(Eigen::Vector3d::Zero())
    , velocity(Eigen::Vector3d::Zero())
  {
  }

  Eigen::Vector3d position;
  Eigen::Vector3d aiming;
  Eigen::Vector3d velocity;
};

struct TwistDecompositionSetpoint
{
  TwistDecompositionSetpoint()
    : pose(KDL::Frame::Identity()), velocity(Eigen::Vector3d::Zero())
  {
  }

  KDL::Frame pose;
  Eigen::Vector3d velocity;
};

}  // namespace coordinated_motion_controllers

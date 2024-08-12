#pragma once

#include <Eigen/Dense>
#include <kdl/frames.hpp>

namespace axially_symmetric_controllers
{

struct AxiallySymmetricSetpoint
{
  AxiallySymmetricSetpoint()
    : pose(KDL::Frame::Identity()), velocity(Eigen::Vector3d::Zero())
  {
  }

  KDL::Frame pose;
  Eigen::Vector3d velocity;
};

}  // namespace axially_symmetric_controllers

#include <axially_symmetric_controllers/utility.h>

namespace axially_symmetric_controllers
{

double angleBetween(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2)
{
  return acos(
      std::min(1.0, std::max(-1.0, v1.dot(v2) / (v1.norm() * v2.norm()))));
}

Eigen::Vector3d axisBetween(const Eigen::Vector3d& v1,
                            const Eigen::Vector3d& v2, double tol)
{
  Eigen::Vector3d axis = v1.cross(v2);
  double norm = axis.norm();
  if (norm > tol)
  {
    return axis / norm;
  }
  return Eigen::Vector3d::Zero();
}

}  // namespace axially_symmetric_controllers

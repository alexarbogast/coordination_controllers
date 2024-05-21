#include <coordinated_motion_controllers/utility.h>

namespace coordinated_motion_controllers
{

double angleBetween(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2)
{
  return acos(v1.dot(v2) / (v1.norm() * v2.norm()));
}

Eigen::Vector3d axisBetween(const Eigen::Vector3d& v1,
                            const Eigen::Vector3d& v2)
{
  Eigen::Vector3d axis = v1.cross(v2);
  double norm = axis.norm();
  if (norm > 1e-6)
  {
    return axis / norm;
  }
  return Eigen::Vector3d::Zero();
}

}  // namespace coordinated_motion_controllers

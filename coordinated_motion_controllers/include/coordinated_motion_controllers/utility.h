#pragma once

#include <Eigen/Dense>

namespace coordinated_motion_controllers
{

/**
 * @brief Find the angle (in radians) from vectors v1 to v2
 *
 * No checks are performed on the magnitude of the input vectors.
 *
 * @param v1 the first vector
 * @param v1 the second vector
 * @returns the angle in radians between v1 and v2
 */
double angleBetween(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2);

/**
 * @brief Find the unit magnitude axis of rotation to rotate vector v1 to v2
 *
 * Returns zero if the vectors are collinear. No checks are performed on the
 * magnitude of the input vectors.
 *
 * @param v1 the first vector
 * @param v1 the second vector
 * @returns the unit vector axis of rotation
 */
Eigen::Vector3d axisBetween(const Eigen::Vector3d& v1,
                            const Eigen::Vector3d& v2);

}  // namespace coordinated_motion_controllers

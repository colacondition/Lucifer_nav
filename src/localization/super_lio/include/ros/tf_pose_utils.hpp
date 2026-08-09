#ifndef SUPER_LIO_TF_POSE_UTILS_HPP_
#define SUPER_LIO_TF_POSE_UTILS_HPP_

#include <cmath>

#include <tf2/LinearMath/Transform.h>

namespace LI2Sup
{

inline bool finiteTransform(const tf2::Transform& transform)
{
  const auto& origin = transform.getOrigin();
  const auto rotation = transform.getRotation();
  return std::isfinite(origin.x()) &&
         std::isfinite(origin.y()) &&
         std::isfinite(origin.z()) &&
         std::isfinite(rotation.x()) &&
         std::isfinite(rotation.y()) &&
         std::isfinite(rotation.z()) &&
         std::isfinite(rotation.w());
}

inline bool composeImuPoseToLidar(
    const tf2::Transform& lidar_odom_to_imu,
    const tf2::Transform& imu_to_lidar,
    tf2::Transform& lidar_odom_to_lidar)
{
  if (!finiteTransform(lidar_odom_to_imu) ||
      !finiteTransform(imu_to_lidar)) {
    return false;
  }

  lidar_odom_to_lidar = lidar_odom_to_imu * imu_to_lidar;
  return finiteTransform(lidar_odom_to_lidar);
}

inline bool composeLidarOdomToBase(
    const tf2::Transform& lidar_odom_to_livox,
    const tf2::Transform& livox_to_base,
    tf2::Transform& lidar_odom_to_base)
{
  if (!finiteTransform(lidar_odom_to_livox) ||
      !finiteTransform(livox_to_base)) {
    return false;
  }

  lidar_odom_to_base = lidar_odom_to_livox * livox_to_base;
  return finiteTransform(lidar_odom_to_base);
}

}  // namespace LI2Sup

#endif

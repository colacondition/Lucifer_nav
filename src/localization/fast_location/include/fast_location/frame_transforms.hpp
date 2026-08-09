#pragma once

#include <array>
#include <cmath>

#include <Eigen/Dense>

namespace fast_location
{

inline Eigen::Matrix4f planarPoseMatrix(const std::array<double, 3> & pose)
{
  const float yaw = static_cast<float>(pose[2]);
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform(0, 0) = std::cos(yaw);
  transform(0, 1) = -std::sin(yaw);
  transform(1, 0) = std::sin(yaw);
  transform(1, 1) = std::cos(yaw);
  transform(0, 3) = static_cast<float>(pose[0]);
  transform(1, 3) = static_cast<float>(pose[1]);
  return transform;
}

inline bool isFiniteTransform(const Eigen::Matrix4f & transform)
{
  return transform.array().isFinite().all();
}

inline Eigen::Matrix4f composeMapToOdom(
  const Eigen::Matrix4f & map_from_pcd,
  const Eigen::Matrix4f & pcd_from_odom)
{
  return map_from_pcd * pcd_from_odom;
}

inline Eigen::Matrix4f initialPoseToPcdOdom(
  const Eigen::Matrix4f & map_from_pcd,
  const Eigen::Matrix4f & map_from_base,
  const Eigen::Matrix4f & odom_from_base)
{
  return map_from_pcd.inverse() * map_from_base * odom_from_base.inverse();
}

}  // namespace fast_location

#ifndef NAVIGATION2__DISTANCE_TRANSFORM_HPP_
#define NAVIGATION2__DISTANCE_TRANSFORM_HPP_

#include <cmath>
#include <cstdint>
#include <vector>

namespace navigation2
{

// 精确欧氏距离变换（Felzenszwalb & Huttenlocher 两遍抛物线下包络），返回每格到
// 最近种子格（seeds[i] != 0）的欧氏距离（单位：格）。O(n)，无堆、cache 友好。
//
// 不沿用八邻域 Dijkstra：那个版本对 45° 之外的方向系统性高估最多约 8%，误差
// 直接落在代价梯度上。也不引 OpenCV —— 为一个距离变换拖进整个 imgproc 不值得。
std::vector<double> exactSquaredDistanceTransform(
  const std::vector<std::uint8_t> & seeds, int width, int height);

// 离期望净空越近，惩罚越小；达到 desired_clearance 后惩罚为 0。
inline double clearancePenalty(
  double distance_to_obstacle, double desired_clearance, double weight) noexcept
{
  if (!std::isfinite(distance_to_obstacle) || desired_clearance <= 0.0 || weight <= 0.0 ||
    distance_to_obstacle >= desired_clearance)
  {
    return 0.0;
  }

  const double clearance_error =
    (desired_clearance - std::max(0.0, distance_to_obstacle)) / desired_clearance;
  return weight * clearance_error * clearance_error;
}

}  // namespace navigation2

#endif  // NAVIGATION2__DISTANCE_TRANSFORM_HPP_

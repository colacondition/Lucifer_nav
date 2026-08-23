#ifndef NAVIGATION2__DISTANCE_TRANSFORM_HPP_
#define NAVIGATION2__DISTANCE_TRANSFORM_HPP_

#include <cmath>
#include <cstdint>
#include <vector>

namespace navigation2
{

// EDT 工作区。地图尺寸不变时复用全部临时数组，避免 local costmap 10Hz 热路径反复
// malloc/free。一个工作区只能被一个调用线程使用；当前 local costmap 回调组串行，符合约束。
struct DistanceTransformWorkspace
{
  std::vector<double> squared;
  std::vector<double> f;
  std::vector<double> line_out;
  std::vector<int> envelope_indices;
  std::vector<double> envelope_breaks;
};

// 写入式精确欧氏距离变换（Felzenszwalb & Huttenlocher 两遍抛物线下包络）。
// output 返回每格到最近种子格（seeds[i] != 0）的欧氏距离（单位：格）。O(n)、无堆。
void exactDistanceTransform(
  const std::vector<std::uint8_t> & seeds, int width, int height,
  DistanceTransformWorkspace & workspace, std::vector<double> & output);

// 兼容非热路径调用的返回式封装；内部临时工作区只在本次调用存在。
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

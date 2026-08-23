#ifndef NAVIGATION2__MINCO_TIME_ALLOCATION_HPP_
#define NAVIGATION2__MINCO_TIME_ALLOCATION_HPP_

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <vector>

namespace navigation2
{

// 为几何路径分配固定的 MINCO 段时间。
//
// 基础时间仍是 distance / velocity，不把时间塞进 L-BFGS，避免扩大优化变量和破坏当前
// 单线程优化热路径。转角附加时间按相邻两段长度比例分摊到拐角两侧，使急弯附近控制点
// 获得更大的时间余量；总附加量为 turn_time_weight * |turn_angle|（秒）。这对应
// RoboWalker 报告中“路径长度 + 转角总量”的前端时间度量，但保持 Lucifer 当前固定时间
// MINCO 的内存规模和求解结构不变。
inline std::vector<double> allocateMincoSegmentTimes(
  const std::vector<Eigen::Vector2d> & waypoints, double velocity,
  double min_segment_time, double turn_time_weight)
{
  if (waypoints.size() < 2) {
    return {};
  }

  velocity = std::max(velocity, 1e-6);
  min_segment_time = std::max(min_segment_time, 1e-6);
  turn_time_weight = std::max(0.0, turn_time_weight);

  const std::size_t segment_count = waypoints.size() - 1;
  std::vector<double> lengths(segment_count, 0.0);
  std::vector<double> times(segment_count, min_segment_time);
  for (std::size_t i = 0; i < segment_count; ++i) {
    lengths[i] = (waypoints[i + 1] - waypoints[i]).norm();
    times[i] = std::max(lengths[i] / velocity, min_segment_time);
  }

  if (turn_time_weight <= 0.0 || segment_count < 2) {
    return times;
  }

  constexpr double kLengthEpsilon = 1e-9;
  for (std::size_t i = 1; i + 1 < waypoints.size(); ++i) {
    const Eigen::Vector2d incoming = waypoints[i] - waypoints[i - 1];
    const Eigen::Vector2d outgoing = waypoints[i + 1] - waypoints[i];
    const double len_in = incoming.norm();
    const double len_out = outgoing.norm();
    if (len_in <= kLengthEpsilon || len_out <= kLengthEpsilon) {
      continue;
    }

    const double cosine = std::clamp(incoming.dot(outgoing) / (len_in * len_out), -1.0, 1.0);
    const double turn_angle = std::acos(cosine);
    const double extra_time = turn_time_weight * turn_angle;
    const double length_sum = len_in + len_out;

    // 长段承担更多转角过渡时间，避免很短的折线段被附加时间完全支配。
    times[i - 1] += extra_time * len_in / length_sum;
    times[i] += extra_time * len_out / length_sum;
  }

  return times;
}

}  // namespace navigation2

#endif  // NAVIGATION2__MINCO_TIME_ALLOCATION_HPP_

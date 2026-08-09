#include "path_stitching.hpp"

#include <algorithm>
#include <cmath>

namespace navigation2
{

namespace
{

double distance2d(
  const geometry_msgs::msg::Point & lhs, const geometry_msgs::msg::Point & rhs) noexcept
{
  return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
}  // namespace

}  // namespace

std::optional<StitchTarget> findStitchTarget(
  const nav_msgs::msg::Path & previous_path,
  const geometry_msgs::msg::Point & current_position,
  double min_lookahead_distance,
  double max_stitch_distance)
{
  // 先找当前位置最近的旧路径点。
  if (previous_path.poses.size() < 2 || max_stitch_distance <= 0.0) {
    return std::nullopt;
  }

  const double min_lookahead = std::max(0.0, min_lookahead_distance);

  std::size_t nearest_index = 0;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < previous_path.poses.size(); ++i) {
    const double distance = distance2d(current_position, previous_path.poses[i].pose.position);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest_index = i;
    }
  }

  for (std::size_t i = nearest_index + 1; i < previous_path.poses.size(); ++i) {
    const double distance = distance2d(current_position, previous_path.poses[i].pose.position);
    if (distance >= min_lookahead && distance <= max_stitch_distance) {
      return StitchTarget{i, previous_path.poses[i], distance};
    }
  }

  return std::nullopt;
}

nav_msgs::msg::Path stitchPath(
  const nav_msgs::msg::Path & local_prefix,
  const nav_msgs::msg::Path & previous_path,
  std::size_t stitch_index)
{
  // 保留局部前缀，再拼上旧路径尾巴。
  nav_msgs::msg::Path out;
  out.header = !local_prefix.header.frame_id.empty() ? local_prefix.header : previous_path.header;

  out.poses.reserve(local_prefix.poses.size() + previous_path.poses.size());
  out.poses.insert(out.poses.end(), local_prefix.poses.begin(), local_prefix.poses.end());

  if (previous_path.poses.empty()) {
    return out;
  }

  stitch_index = std::min(stitch_index, previous_path.poses.size() - 1);
  const std::size_t tail_begin = local_prefix.poses.empty() ? stitch_index : stitch_index + 1;
  for (std::size_t i = tail_begin; i < previous_path.poses.size(); ++i) {
    auto pose = previous_path.poses[i];
    pose.header = out.header;
    out.poses.push_back(pose);
  }

  return out;
}

}  // namespace navigation2

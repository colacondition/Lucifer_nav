#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

namespace navigation2
{

// 路径拼接时使用的连接点。
struct StitchTarget
{
  std::size_t index{0};
  geometry_msgs::msg::PoseStamped pose;
  double distance{0.0};
};

// 在旧路径里找适合接上的位置。
std::optional<StitchTarget> findStitchTarget(
  const nav_msgs::msg::Path & previous_path,
  const geometry_msgs::msg::Point & current_position,
  double min_lookahead_distance,
  double max_stitch_distance);

// 把局部前缀和旧路径尾段拼起来。
nav_msgs::msg::Path stitchPath(
  const nav_msgs::msg::Path & local_prefix,
  const nav_msgs::msg::Path & previous_path,
  std::size_t stitch_index);

}  // namespace navigation2

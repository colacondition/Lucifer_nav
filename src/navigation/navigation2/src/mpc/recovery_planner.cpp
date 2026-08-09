#include "mpc/recovery_planner.hpp"

#include "grid_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace navigation2::mpc {

namespace {

bool gridUsable(const nav_msgs::msg::OccupancyGrid & grid) noexcept
{
  const auto width = static_cast<std::size_t>(grid.info.width);
  const auto height = static_cast<std::size_t>(grid.info.height);
  return width > 0 && height > 0 && grid.info.resolution > 0.0F &&
         width <= std::numeric_limits<std::size_t>::max() / height &&
         grid.data.size() == width * height;
}

// 读格子值。越界返回 nullopt，交给调用方按策略处理。
std::optional<int> cellValue(
  const nav_msgs::msg::OccupancyGrid & grid, const Eigen::Vector2d & pos) noexcept
{
  int map_x = 0;
  int map_y = 0;
  if (!worldToMap(grid, pos.x(), pos.y(), map_x, map_y)) {
    return std::nullopt;
  }
  if (!inBounds(grid, map_x, map_y)) {
    return std::nullopt;
  }
  return static_cast<int>(grid.data[gridIndex(grid, map_x, map_y)]);
}

// 判断一个格值是否达到某阈值，含未知格策略。
bool valueExceeds(int value, int threshold, bool unknown_counts) noexcept
{
  if (value < 0) {
    return unknown_counts;
  }
  return value >= threshold;
}

}  // namespace

bool isHazardous(
  const nav_msgs::msg::OccupancyGrid & grid, const Eigen::Vector2d & pos,
  const HazardPolicy & policy) noexcept
{
  if (!gridUsable(grid) || !pos.allFinite()) {
    return policy.out_of_map_is_hazard;
  }

  const auto value = cellValue(grid, pos);
  if (!value) {
    return policy.out_of_map_is_hazard;
  }
  return valueExceeds(*value, policy.hazard_cost, policy.unknown_is_hazard);
}

std::optional<Eigen::Vector2d> findSafePoint(
  const nav_msgs::msg::OccupancyGrid & grid, const Eigen::Vector2d & pos,
  const HazardPolicy & policy, const SafePointSearchParams & params) noexcept
{
  if (!gridUsable(grid) || !pos.allFinite()) {
    return std::nullopt;
  }

  int origin_x = 0;
  int origin_y = 0;
  if (!worldToMap(grid, pos.x(), pos.y(), origin_x, origin_y)) {
    return std::nullopt;
  }

  const int samples = std::max(params.samples_per_ring, 1);

  // 半径升序，配合距离惩罚让近点在同等安全时胜出。
  std::vector<double> radii = params.ring_radii;
  std::sort(radii.begin(), radii.end());

  std::optional<Eigen::Vector2d> best_point;
  double best_score = std::numeric_limits<double>::infinity();

  for (const double radius : radii) {
    if (!(radius > 0.0)) {
      continue;
    }
    for (int i = 0; i < samples; ++i) {
      const double angle = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(samples);
      const Eigen::Vector2d candidate(
        pos.x() + radius * std::cos(angle), pos.y() + radius * std::sin(angle));

      int candidate_x = 0;
      int candidate_y = 0;
      if (!worldToMap(grid, candidate.x(), candidate.y(), candidate_x, candidate_y)) {
        continue;  // 采到图外，跳过。
      }
      if (!inBounds(grid, candidate_x, candidate_y)) {
        continue;
      }

      const int candidate_value =
        static_cast<int>(grid.data[gridIndex(grid, candidate_x, candidate_y)]);
      if (valueExceeds(candidate_value, policy.lethal_cost, policy.unknown_is_hazard)) {
        continue;  // 候选点自身是致命障碍。
      }

      // 射线检查：安全点必须可达，不能在墙对面。
      bool ray_blocked = false;
      int max_cost_on_ray = 0;
      for (const auto & cell : raytraceLine(origin_x, origin_y, candidate_x, candidate_y)) {
        if (!inBounds(grid, cell.x, cell.y)) {
          ray_blocked = true;
          break;
        }
        // 跳过车自己所在的格。车已经站在这儿了，若因「自己的格是 lethal」
        // 就判所有方向不可达、返回 nullopt，恢复恰好在最需要的时候失效。
        // 局部代价图正常会用 markRobotFootprintFree 清掉车身足迹，但代价图
        // 与 MPC 用的位姿存在容差时不保证覆盖，而脱困场景正是两者最容易
        // 不一致的时候。
        if (cell.x == origin_x && cell.y == origin_y) {
          continue;
        }
        const int value = static_cast<int>(grid.data[gridIndex(grid, cell.x, cell.y)]);
        if (valueExceeds(value, policy.lethal_cost, policy.unknown_is_hazard)) {
          ray_blocked = true;
          break;
        }
        // 未知格按 0 参与打分（是否算危险已由上面的 lethal 判据处理）。
        max_cost_on_ray = std::max(max_cost_on_ray, std::max(value, 0));
      }
      if (ray_blocked) {
        continue;
      }

      // 打分：优先低代价，其次近距离。
      const double score =
        static_cast<double>(max_cost_on_ray) + params.distance_penalty * radius;
      if (score < best_score) {
        best_score = score;
        best_point = candidate;
      }
    }
  }

  return best_point;
}

}  // namespace navigation2::mpc

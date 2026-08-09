#include "local_path_safety.hpp"

#include "grid_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace navigation2
{

bool isPathSafe(
  const nav_msgs::msg::OccupancyGrid & grid,
  const std::vector<Eigen::Vector2d> & positions,
  const LocalPathSafetyPolicy & policy) noexcept
{
  // 栅格和预测点都得先合法。
  const auto width = static_cast<std::size_t>(grid.info.width);
  const auto height = static_cast<std::size_t>(grid.info.height);
  if (width == 0 || height == 0 || grid.info.resolution <= 0.0F ||
    width > std::numeric_limits<std::size_t>::max() / height ||
    grid.data.size() != width * height || positions.empty())
  {
    return false;
  }

  // 只检查前面几个预测点。
  const std::size_t endpoint_count = policy.check_steps > 0 ?
    std::min<std::size_t>(static_cast<std::size_t>(policy.check_steps), positions.size() - 1) :
    positions.size() - 1;
  const std::size_t position_count = endpoint_count + 1;

  int previous_x = 0;
  int previous_y = 0;
  for (std::size_t i = 0; i < position_count; ++i) {
    const auto & point = positions[i];
    if (!point.allFinite()) {
      return false;
    }

    int map_x = 0;
    int map_y = 0;
    if (!worldToMap(grid, point.x(), point.y(), map_x, map_y)) {
      return false;
    }

    // 起点只看自己，其余点看连线。
    const auto cells = i == 0 ? std::vector<GridCell>{{map_x, map_y}} :
    raytraceLine(previous_x, previous_y, map_x, map_y);
    for (const auto & cell : cells) {
      if (!inBounds(grid, cell.x, cell.y)) {
        return false;
      }
      const auto value = grid.data[gridIndex(grid, cell.x, cell.y)];
      if (isOccupied(value, policy.obstacle_threshold, policy.unknown_is_obstacle)) {
        return false;
      }
    }

    previous_x = map_x;
    previous_y = map_y;
  }
  return true;
}

}  // namespace navigation2

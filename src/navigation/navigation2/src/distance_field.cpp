#include "distance_field.hpp"

#include "grid_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>

namespace navigation2
{

namespace
{

struct QueueItem
{
  std::size_t index{0};
  double distance{0.0};

  bool operator>(const QueueItem & other) const noexcept
  {
    return distance > other.distance;
  }
};

bool gridShapeValid(const nav_msgs::msg::OccupancyGrid & grid) noexcept
{
  const auto width = static_cast<std::size_t>(grid.info.width);
  const auto height = static_cast<std::size_t>(grid.info.height);
  return width > 0 && height > 0 && grid.info.resolution > 0.0F &&
         width <= std::numeric_limits<std::size_t>::max() / height &&
         grid.data.size() == width * height;
}

}  // namespace

bool GridDistanceField::valid() const noexcept
{
  // 尺寸和缓存长度要一致。
  const auto expected = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  return width > 0 && height > 0 && resolution > 0.0 && distances.size() == expected;
}

double GridDistanceField::distanceAt(std::size_t index) const noexcept
{
  if (index >= distances.size()) {
    return std::numeric_limits<double>::infinity();
  }
  return distances[index];
}

GridDistanceField buildDistanceField(
  const nav_msgs::msg::OccupancyGrid & grid, int obstacle_threshold,
  bool unknown_is_obstacle) noexcept
{
  // 先把障碍格作为种子点。
  if (!gridShapeValid(grid)) {
    return {};
  }

  GridDistanceField field;
  field.width = grid.info.width;
  field.height = grid.info.height;
  field.resolution = grid.info.resolution;
  field.distances.assign(grid.data.size(), std::numeric_limits<double>::infinity());

  std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> open;
  for (int y = 0; y < static_cast<int>(grid.info.height); ++y) {
    for (int x = 0; x < static_cast<int>(grid.info.width); ++x) {
      const auto index = gridIndex(grid, x, y);
      if (!isOccupied(grid.data[index], obstacle_threshold, unknown_is_obstacle)) {
        continue;
      }
      field.distances[index] = 0.0;
      open.push({index, 0.0});
    }
  }

  // 八邻域扩散。
  constexpr int directions[8][2] = {
    {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

  while (!open.empty()) {
    const auto current = open.top();
    open.pop();
    if (current.distance > field.distances[current.index] + 1e-12) {
      continue;
    }

    const int cx = static_cast<int>(current.index % grid.info.width);
    const int cy = static_cast<int>(current.index / grid.info.width);
    for (const auto & direction : directions) {
      const int nx = cx + direction[0];
      const int ny = cy + direction[1];
      if (!inBounds(grid, nx, ny)) {
        continue;
      }

      const bool diagonal = direction[0] != 0 && direction[1] != 0;
      const double step = field.resolution * (diagonal ? std::sqrt(2.0) : 1.0);
      const auto neighbor = gridIndex(grid, nx, ny);
      const double tentative = current.distance + step;
      if (tentative >= field.distances[neighbor]) {
        continue;
      }

      field.distances[neighbor] = tentative;
      open.push({neighbor, tentative});
    }
  }

  return field;
}

double clearancePenalty(
  double distance_to_obstacle, double desired_clearance, double weight) noexcept
{
  // 离期望净空越近，惩罚越小。
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

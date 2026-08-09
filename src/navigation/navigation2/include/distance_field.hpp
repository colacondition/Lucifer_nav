#ifndef NAVIGATION2__DISTANCE_FIELD_HPP_
#define NAVIGATION2__DISTANCE_FIELD_HPP_

#include <cstddef>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>

namespace navigation2
{

// 记录一张栅格地图对应的距离场。
struct GridDistanceField
{
  unsigned int width{0};
  unsigned int height{0};
  double resolution{0.0};
  std::vector<double> distances;

  // 判断距离场是否完整可用。
  bool valid() const noexcept;

  // 按下标取距离，越界时返回无穷大。
  double distanceAt(std::size_t index) const noexcept;
};

// 从占据栅格里生成距离场。
GridDistanceField buildDistanceField(
  const nav_msgs::msg::OccupancyGrid & grid, int obstacle_threshold,
  bool unknown_is_obstacle) noexcept;

// 把离障碍太近的状态转成惩罚值。
double clearancePenalty(
  double distance_to_obstacle, double desired_clearance, double weight) noexcept;

}  // namespace navigation2

#endif  // NAVIGATION2__DISTANCE_FIELD_HPP_

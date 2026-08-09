#ifndef NAVIGATION2__LOCAL_PATH_SAFETY_HPP_
#define NAVIGATION2__LOCAL_PATH_SAFETY_HPP_

#include <vector>

#include <Eigen/Core>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace navigation2
{

// 局部路径安全检查规则。
struct LocalPathSafetyPolicy
{
  int obstacle_threshold{50};
  bool unknown_is_obstacle{true};
  int check_steps{0};
};

// 检查预测路径是否穿障。
bool isPathSafe(
  const nav_msgs::msg::OccupancyGrid & grid,
  const std::vector<Eigen::Vector2d> & positions,
  const LocalPathSafetyPolicy & policy) noexcept;

}  // namespace navigation2

#endif  // NAVIGATION2__LOCAL_PATH_SAFETY_HPP_

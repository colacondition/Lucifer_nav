#pragma once
// 脱困用的危险判定与安全点采样。
//
// 为什么恢复期要单独一套阈值：正常跟踪用 local_safety.obstacle_threshold(50)
// 判定预测轨迹是否可行。但车已经贴到障碍上时，它所在的格子本身就在膨胀圈
// 里、代价远高于 50，用同一个阈值检查倒车轨迹会让**所有**方向立即被否，
// 恢复永远无法启动。
//
// 因此恢复期只做 lethal 检查：局部代价图里障碍格写 100、膨胀值按指数衰减
// 严格低于 100，所以用 lethal_cost(99) 就能区分「真障碍」与「膨胀圈」。
// 允许在膨胀圈内低速通行是脱困的必要放宽，靠限速控制风险。
#include <optional>
#include <vector>

#include <Eigen/Dense>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace navigation2::mpc {

struct HazardPolicy
{
  // 当前格代价达到此值即判为危险（膨胀圈内也算）。
  int hazard_cost = 80;
  // 达到此值算致命障碍，恢复期的通行判据只看这个。
  int lethal_cost = 99;
  // 未知格（-1）是否算危险。默认算，与 local_safety.unknown_is_obstacle 一致。
  bool unknown_is_hazard = true;
  // 地图外是否算危险。默认算。
  bool out_of_map_is_hazard = true;
};

struct SafePointSearchParams
{
  // 同心环采样半径（m），由近到远。
  std::vector<double> ring_radii{0.3, 0.6, 0.9, 1.2};
  // 每环的角度采样数。
  int samples_per_ring = 12;
  // 距离惩罚权重：同等安全时优先更近的点，减少脱困位移。
  double distance_penalty = 20.0;
};

// 当前位置是否危险。地图不可用时返回 out_of_map_is_hazard。
bool isHazardous(
  const nav_msgs::msg::OccupancyGrid & grid, const Eigen::Vector2d & pos,
  const HazardPolicy & policy) noexcept;

// 在周围采样一个可达的安全点。
//
// 候选点需同时满足：自身非 lethal，且从当前位置到它的射线上无 lethal
// （否则「安全点」在墙对面，驶过去会撞墙）。打分 = 射线上的最大代价
// + distance_penalty * 距离，取最小分。全被封死时返回 nullopt。
std::optional<Eigen::Vector2d> findSafePoint(
  const nav_msgs::msg::OccupancyGrid & grid, const Eigen::Vector2d & pos,
  const HazardPolicy & policy, const SafePointSearchParams & params) noexcept;

}  // namespace navigation2::mpc

#include "mpc/recovery_planner.hpp"

#include "grid_utils.hpp"

#include <cmath>
#include <cstddef>

#include <gtest/gtest.h>

namespace navigation2::mpc
{
namespace
{

// 81x81 @ 0.05 m，原点 (-2,-2)：世界 (0,0) 附近四周留出 2 m，
// 足够容纳最大 1.2 m 的采样环。
//
// 注意：格号一律通过 worldToMap 反查，不硬编码。resolution 在消息里是
// float，2.0 / 0.05f = 39.9999994 会 floor 到 39 而不是 40，硬编码会
// 让测试断在自己的算术上而不是被测代码上。
constexpr int kWidth = 81;
constexpr int kHeight = 81;
constexpr double kResolution = 0.05;

nav_msgs::msg::OccupancyGrid makeGrid(int8_t fill = 0)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.width = static_cast<unsigned int>(kWidth);
  grid.info.height = static_cast<unsigned int>(kHeight);
  grid.info.resolution = static_cast<float>(kResolution);
  grid.info.origin.position.x = -2.0;
  grid.info.origin.position.y = -2.0;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(static_cast<std::size_t>(kWidth) * kHeight, fill);
  return grid;
}

// 按世界坐标写格值，格号由 worldToMap 决定。
void setCellAtWorld(
  nav_msgs::msg::OccupancyGrid & grid, double wx, double wy, int8_t value)
{
  int mx = 0;
  int my = 0;
  ASSERT_TRUE(worldToMap(grid, wx, wy, mx, my));
  grid.data[gridIndex(grid, mx, my)] = value;
}

// 把世界坐标落在 [x_lo, x_hi] x [y_lo, y_hi] 内的格子写成 value。
void fillWorldRect(
  nav_msgs::msg::OccupancyGrid & grid, double x_lo, double x_hi, double y_lo, double y_hi,
  int8_t value)
{
  for (int my = 0; my < kHeight; ++my) {
    for (int mx = 0; mx < kWidth; ++mx) {
      double wx = 0.0;
      double wy = 0.0;
      mapToWorld(grid, mx, my, wx, wy);
      if (wx >= x_lo && wx <= x_hi && wy >= y_lo && wy <= y_hi) {
        grid.data[gridIndex(grid, mx, my)] = value;
      }
    }
  }
}

TEST(RecoveryPlanner, OpenSpaceIsNotHazardous)
{
  const auto grid = makeGrid(0);
  EXPECT_FALSE(isHazardous(grid, Eigen::Vector2d(0.0, 0.0), HazardPolicy{}));
}

TEST(RecoveryPlanner, HighCostCellIsHazardous)
{
  auto grid = makeGrid(0);
  HazardPolicy policy;

  // 刚好达到 hazard_cost 即算危险。
  setCellAtWorld(grid, 0.0, 0.0, static_cast<int8_t>(policy.hazard_cost));
  EXPECT_TRUE(isHazardous(grid, Eigen::Vector2d(0.0, 0.0), policy));

  // 低一档则不算。
  setCellAtWorld(grid, 0.0, 0.0, static_cast<int8_t>(policy.hazard_cost - 1));
  EXPECT_FALSE(isHazardous(grid, Eigen::Vector2d(0.0, 0.0), policy));
}

TEST(RecoveryPlanner, OutOfMapFollowsPolicy)
{
  const auto grid = makeGrid(0);
  HazardPolicy hazardous;
  HazardPolicy permissive;
  permissive.out_of_map_is_hazard = false;

  const Eigen::Vector2d far_away(50.0, 50.0);
  EXPECT_TRUE(isHazardous(grid, far_away, hazardous));
  EXPECT_FALSE(isHazardous(grid, far_away, permissive));
}

TEST(RecoveryPlanner, UnknownCellFollowsPolicy)
{
  auto grid = makeGrid(0);
  setCellAtWorld(grid, 0.0, 0.0, -1);

  HazardPolicy unknown_hazardous;
  HazardPolicy unknown_free;
  unknown_free.unknown_is_hazard = false;

  EXPECT_TRUE(isHazardous(grid, Eigen::Vector2d(0.0, 0.0), unknown_hazardous));
  EXPECT_FALSE(isHazardous(grid, Eigen::Vector2d(0.0, 0.0), unknown_free));
}

TEST(RecoveryPlanner, MalformedGridIsHazardous)
{
  auto grid = makeGrid(0);
  grid.data.pop_back();
  EXPECT_TRUE(isHazardous(grid, Eigen::Vector2d(0.0, 0.0), HazardPolicy{}));
}

TEST(RecoveryPlanner, PrefersNearestRingInOpenSpace)
{
  const auto grid = makeGrid(0);
  const auto point = findSafePoint(
    grid, Eigen::Vector2d(0.0, 0.0), HazardPolicy{}, SafePointSearchParams{});

  // 全空旷时代价项处处为 0，距离惩罚让最近的环胜出。
  ASSERT_TRUE(point.has_value());
  EXPECT_NEAR(point->norm(), 0.3, 1e-6);
}

TEST(RecoveryPlanner, PicksOpenCorridorWhenBoxedIn)
{
  // 除了一条沿 +x 的窄走廊，其余全部致命。
  // 注意 mapToWorld 返回的是格中心：车在世界 (0,0) 时所在格的中心是
  // (-0.025, -0.025)，所以 x 下界必须取负值才能把车自己的格包进走廊。
  auto grid = makeGrid(100);
  fillWorldRect(grid, -0.15, 1.5, -0.12, 0.12, 0);

  const auto point = findSafePoint(
    grid, Eigen::Vector2d(0.0, 0.0), HazardPolicy{}, SafePointSearchParams{});

  ASSERT_TRUE(point.has_value());
  // 唯一的出口在 +x 方向。
  EXPECT_GT(point->x(), 0.0);
  EXPECT_NEAR(point->y(), 0.0, 0.12);
}

// 车自己所在的格是致命的时候仍然要能找到出路。这正是最需要恢复的场景：
// 局部代价图与位姿不一致、或代价图偏旧时，车的格子可能被标成障碍。若射线
// 检查把起点格也算进去，findSafePoint 会直接返回 nullopt，恢复无从启动。
TEST(RecoveryPlanner, FindsPointEvenWhenOwnCellIsLethal)
{
  auto grid = makeGrid(0);
  setCellAtWorld(grid, 0.0, 0.0, 100);

  const auto point = findSafePoint(
    grid, Eigen::Vector2d(0.0, 0.0), HazardPolicy{}, SafePointSearchParams{});

  ASSERT_TRUE(point.has_value());
  EXPECT_NEAR(point->norm(), 0.3, 1e-6);
}

TEST(RecoveryPlanner, ReturnsNulloptWhenFullyEnclosed)
{
  const auto grid = makeGrid(100);
  const auto point = findSafePoint(
    grid, Eigen::Vector2d(0.0, 0.0), HazardPolicy{}, SafePointSearchParams{});

  EXPECT_FALSE(point.has_value());
}

TEST(RecoveryPlanner, RejectsCandidateBehindLethalWall)
{
  // +x 方向 0.10~0.20 m 处一道致命墙，纵向贯穿整图。
  auto grid = makeGrid(0);
  fillWorldRect(grid, 0.10, 0.20, -2.0, 2.0, 100);

  const auto point = findSafePoint(
    grid, Eigen::Vector2d(0.0, 0.0), HazardPolicy{}, SafePointSearchParams{});

  ASSERT_TRUE(point.has_value());
  // 墙后的候选点本身非致命，但射线被挡，必须被排除。
  EXPECT_LT(point->x(), 0.10);
}

TEST(RecoveryPlanner, InflationRingIsTraversableDuringRecovery)
{
  // 恢复期的核心放宽：膨胀圈（代价高但非致命）必须可通行，
  // 否则车贴到障碍上时所有方向都被否，永远无法脱困。
  auto grid = makeGrid(0);
  fillWorldRect(grid, -1.5, 1.5, -1.5, 1.5, 90);

  const auto point = findSafePoint(
    grid, Eigen::Vector2d(0.0, 0.0), HazardPolicy{}, SafePointSearchParams{});

  ASSERT_TRUE(point.has_value());
  // 代价 90 >= hazard_cost(80) 但 < lethal_cost(99)，仍算可通行。
  EXPECT_TRUE(isHazardous(grid, Eigen::Vector2d(0.0, 0.0), HazardPolicy{}));
}

}  // namespace
}  // namespace navigation2::mpc

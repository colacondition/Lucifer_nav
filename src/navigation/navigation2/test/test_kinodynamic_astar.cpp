#include "kinodynamic_astar.hpp"

#include "grid_utils.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace navigation2::mpc
{
namespace
{

// 一张 10 m x 10 m、0.05 m/格的空地图（origin 非零，覆盖 TDT origin bug 场景）。
nav_msgs::msg::OccupancyGrid emptyGrid(double ox, double oy)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.width = 200;
  grid.info.height = 200;
  grid.info.resolution = 0.05F;
  grid.info.origin.position.x = ox;
  grid.info.origin.position.y = oy;
  grid.data.assign(200u * 200u, 0);
  return grid;
}

// 在格矩形（格坐标，含端点）里写致命值。
void markLethal(nav_msgs::msg::OccupancyGrid & grid, int x0, int y0, int x1, int y1)
{
  for (int y = y0; y <= y1; ++y) {
    for (int x = x0; x <= x1; ++x) {
      if (x >= 0 && x < static_cast<int>(grid.info.width) &&
        y >= 0 && y < static_cast<int>(grid.info.height))
      {
        grid.data[static_cast<std::size_t>(y) * grid.info.width + x] = 100;
      }
    }
  }
}

KinoConfig baseConfig()
{
  KinoConfig c;
  c.max_vel = 1.0;
  c.max_acc = 1.0;
  c.max_tau = 1.0;
  c.vel_resolution = 0.25;
  c.sample_time = 0.05;
  c.max_nodes = 40000;
  return c;
}

// 把轨迹采样点的运动学极值全部算出来，供多条断言复用。
struct TrajectoryStats
{
  double max_vel = 0.0;
  double max_acc = 0.0;
  double end_error = 0.0;
  double end_speed = 0.0;
  bool collision_free = true;
};

TrajectoryStats stats(
  const KinoResult & result, const nav_msgs::msg::OccupancyGrid & grid,
  const Eigen::Vector2d & goal)
{
  TrajectoryStats s;
  for (std::size_t i = 0; i < result.trajectory.size(); ++i) {
    const auto & sample = result.trajectory[i];
    s.max_vel = std::max(s.max_vel, sample.state.tail<2>().cwiseAbs().maxCoeff());
    s.max_acc = std::max(s.max_acc, sample.acceleration.cwiseAbs().maxCoeff());
    if (i > 0) {
      const Eigen::Vector2d prev(result.trajectory[i - 1].state.x(),
        result.trajectory[i - 1].state.y());
      const Eigen::Vector2d cur(sample.state.x(), sample.state.y());
      // 逐采样段连线检查（复用恢复规划器同一套射线实现）。
      int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
      if (worldToMap(grid, prev.x(), prev.y(), x0, y0) &&
        worldToMap(grid, cur.x(), cur.y(), x1, y1))
      {
        for (const auto & cell : raytraceLine(x0, y0, x1, y1)) {
          const auto v = grid.data[static_cast<std::size_t>(cell.y) * grid.info.width + cell.x];
          if (v >= 100) {
            s.collision_free = false;
            break;
          }
        }
      }
      if (!s.collision_free) {
        break;
      }
    }
  }
  if (!result.trajectory.empty()) {
    const auto & last = result.trajectory.back();
    s.end_error = (Eigen::Vector2d(last.state.x(), last.state.y()) - goal).norm();
    s.end_speed = last.state.tail<2>().norm();
  }
  return s;
}

TEST(KinoParamsValid, RejectsInvalid)
{
  KinoConfig c = baseConfig();
  EXPECT_TRUE(kinoParamsValid(c));
  c.max_vel = 0.0;
  EXPECT_FALSE(kinoParamsValid(c));
  c = baseConfig();
  c.max_nodes = 0;
  EXPECT_FALSE(kinoParamsValid(c));
  c = baseConfig();
  c.vel_resolution = 0.0;
  EXPECT_FALSE(kinoParamsValid(c));
}

TEST(KinodynamicSearch, StraightLineFromRest)
{
  auto grid = emptyGrid(-2.48, -8.65);
  const Eigen::Vector2d start(grid.info.origin.position.x + 2.0,
    grid.info.origin.position.y + 2.0);
  const Eigen::Vector2d goal(grid.info.origin.position.x + 6.0,
    grid.info.origin.position.y + 2.0);
  const auto result = kinodynamicSearch(
    grid, baseConfig(), start, Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), goal);
  ASSERT_TRUE(result.success);
  const auto s = stats(result, grid, goal);
  EXPECT_LT(s.end_error, 0.15);
  EXPECT_LT(s.end_speed, 1e-6);
  EXPECT_LE(s.max_vel, 1.0 + 1e-6);
  EXPECT_LE(s.max_acc, 1.0 + 1e-6);
  EXPECT_TRUE(s.collision_free);
}

TEST(KinodynamicSearch, RespectsWallWithNonZeroOrigin)
{
  // 竖墙把起点和终点隔开，只留一个 1 m 宽的口，且开口**不在**起点-终点连线的
  // y 上 —— 直线必然撞墙，搜出来的解必须真的绕到开口。origin 非零是
  // TDT 原实现的坑（两套坐标约定互相矛盾），这里同时覆盖。
  auto grid = emptyGrid(-2.48, -8.65);
  const int wall_x = 100;  // 世界 x = -2.48 + 100*0.05 = 2.52
  markLethal(grid, wall_x, 0, wall_x, 199);
  // 开口在第 115..135 行（世界 y = -2.90 .. -1.90）。
  for (int y = 115; y <= 135; ++y) {
    grid.data[static_cast<std::size_t>(y) * grid.info.width + wall_x] = 0;
  }
  const double wall_world_x = grid.info.origin.position.x + wall_x * 0.05;
  // 起点在墙左侧 y≈-6.65，终点在墙右侧 y≈-0.65：连线在 x=wall 处的 y≈-5.9，
  // 离开口（y≈-2.4）差 3.5 m，直线必撞墙。
  const Eigen::Vector2d start(grid.info.origin.position.x + 2.0,
    grid.info.origin.position.y + 2.0);
  const Eigen::Vector2d goal(grid.info.origin.position.x + 8.0,
    grid.info.origin.position.y + 8.0);
  KinoConfig c = baseConfig();
  c.max_vel = 1.5;
  c.max_acc = 2.0;
  c.max_nodes = 80000;
  const auto result = kinodynamicSearch(
    grid, c, start, Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), goal);
  ASSERT_TRUE(result.success);
  const auto s = stats(result, grid, goal);
  EXPECT_LT(s.end_error, 0.2);
  EXPECT_TRUE(s.collision_free);
  // 必须真的越过墙（终点在墙右侧 5.5 m，没越过就说明解是错的）。
  bool crossed = false;
  for (const auto & sample : result.trajectory) {
    if (sample.state.x() > wall_world_x + 0.5) {
      crossed = true;
      break;
    }
  }
  EXPECT_TRUE(crossed);
}

TEST(KinodynamicSearch, MovingStartIsFeasible)
{
  // 从 0.8 m/s 的初速出发：这正是普通 A* 完全给不了的东西。
  auto grid = emptyGrid(0.0, 0.0);
  const Eigen::Vector2d start(2.0, 5.0);
  const Eigen::Vector2d goal(5.0, 5.0);
  const Eigen::Vector2d vel(0.8, 0.0);
  KinoConfig c = baseConfig();
  c.max_vel = 1.0;
  const auto result = kinodynamicSearch(
    grid, c, start, vel, Eigen::Vector2d::Zero(), goal);
  ASSERT_TRUE(result.success);
  // 首段必须与初速度连续：第一个采样点的速度不应瞬间反向或归零。
  ASSERT_GT(result.trajectory.size(), 1u);
  const auto & first = result.trajectory[1];
  EXPECT_GT(first.state.tail<2>().x(), 0.0);
  EXPECT_LE(first.state.tail<2>().cwiseAbs().maxCoeff(), 1.0 + 1e-6);
}

TEST(KinodynamicSearch, UnreachableGoalFailsBounded)
{
  // 终点封在致命盒子里：必须失败，且节点数不超过 max_nodes（内存有界）。
  auto grid = emptyGrid(0.0, 0.0);
  markLethal(grid, 100, 100, 140, 140);
  const Eigen::Vector2d start(2.0, 2.0);
  const Eigen::Vector2d goal(6.0, 6.0);  // 盒子中心
  KinoConfig c = baseConfig();
  c.max_nodes = 3000;
  const auto result = kinodynamicSearch(
    grid, c, start, Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), goal);
  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.trajectory.empty());
  EXPECT_LE(result.nodes, c.max_nodes);
}

TEST(KinodynamicSearch, RejectsBlockedEndpointsAndOverspeed)
{
  auto grid = emptyGrid(0.0, 0.0);
  markLethal(grid, 100, 100, 100, 100);
  const Eigen::Vector2d start(2.0, 2.0);
  const Eigen::Vector2d goal(5.0, 5.0);
  // 致命格中心的世界坐标。
  const Eigen::Vector2d lethal_center(
    grid.info.origin.position.x + 100 * 0.05 + 0.025,
    grid.info.origin.position.y + 100 * 0.05 + 0.025);

  // 终点在致命格上。
  const auto blocked = kinodynamicSearch(
    grid, baseConfig(), start, Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), lethal_center);
  EXPECT_FALSE(blocked.success);

  // 初速超上限。
  const auto overspeed = kinodynamicSearch(
    grid, baseConfig(), start, Eigen::Vector2d(5.0, 0.0), Eigen::Vector2d::Zero(), goal);
  EXPECT_FALSE(overspeed.success);

  // 配置非法。
  KinoConfig bad = baseConfig();
  bad.max_acc = 0.0;
  const auto invalid = kinodynamicSearch(
    grid, bad, start, Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), goal);
  EXPECT_FALSE(invalid.success);
}

TEST(KinodynamicSearch, StartEqualsGoal)
{
  auto grid = emptyGrid(0.0, 0.0);
  const Eigen::Vector2d start(2.0, 2.0);
  const auto result = kinodynamicSearch(
    grid, baseConfig(), start, Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), start);
  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.trajectory.size(), 1u);
}

}  // namespace
}  // namespace navigation2::mpc

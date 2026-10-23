// 中心走位的纯逻辑测试。
//
// 现在这套逻辑只剩两件事：**在区域里生成网格点**（蛇形顺序）、**按顺序取一组点**。
// 没有激进/保守之分、也不看时间 —— 走位本身不改变占领权，它只是"别站着不动"。
//
// 两处飘了之后的现场表现都是「车在中心乱走/跳来跳去」或者「站着不动」，
// 没有一行日志会报错，所以在这里钉住。

#include <cmath>
#include <set>
#include <vector>

#include <gtest/gtest.h>

#include "decision/center_patrol.hpp"
#include "decision/center_zone.hpp"

namespace
{

decision::Pose pt(double x, double y)
{
  decision::Pose p;
  p.x = x;
  p.y = y;
  return p;
}

// 边长 2.4 的正方形（中心 2.6, 4.8），形状参照真实中心区
constexpr double kCx = 2.6;
constexpr double kCy = 4.8;
constexpr double kHalf = 1.2;

decision::CenterZone makeSquare(double margin = 0.0)
{
  return decision::makeCenterZone(
    {pt(kCx - kHalf, kCy - kHalf), pt(kCx + kHalf, kCy - kHalf),
     pt(kCx + kHalf, kCy + kHalf), pt(kCx - kHalf, kCy + kHalf)},
    margin);
}

double dist(const decision::Pose & a, const decision::Pose & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

// ==================== 区域网格 ====================

TEST(GenerateZoneGrid, PutsEveryPointInsideTheSafeZone)
{
  const auto zone = makeSquare(0.3);
  const auto grid = decision::generateZoneGrid(zone, 0.5);
  ASSERT_FALSE(grid.empty());
  for (const auto & p : grid) {
    EXPECT_TRUE(decision::zoneContains(zone, p))
      << "网格点 (" << p.x << ", " << p.y << ") 跑到安全区外了";
  }
}

TEST(GenerateZoneGrid, WalksSerpentineSoConsecutiveStepsAreShort)
{
  // 顺序必须"列表里相邻的点在场上也相邻"：游标是顺序推进的，中间冒出一个远点车就会跳。
  const auto zone = makeSquare(0.3);
  const double spacing = 0.5;
  const auto grid = decision::generateZoneGrid(zone, spacing);
  ASSERT_GT(grid.size(), 4u);
  for (std::size_t i = 1; i < grid.size(); ++i) {
    EXPECT_LE(dist(grid[i - 1], grid[i]), spacing * 1.5 + 1e-9)
      << "第 " << i << " 步跳了 " << dist(grid[i - 1], grid[i]) << " m";
  }
}

TEST(GenerateZoneGrid, UsesCellCentresNotBoundingBoxCorners)
{
  // 布点必须是**格子中心**：四边形上下边往往是斜的，把首末两行放在包围盒边界刻度上
  // 会让它们几乎整行落在多边形外（真机上就吃过这个亏：1.8x2.2 的区域只生出 7 个点）。
  // 正方形区域 + 0.6 格距、边长 2.4 ⇒ 整 4 格 ⇒ 4x4=16 个点，一个都不该丢。
  const auto zone = makeSquare(0.0);
  const auto grid = decision::generateZoneGrid(zone, 0.6);
  EXPECT_EQ(grid.size(), 16u);
  // 首点应该离包围盒边界有半个格距，而不是贴在边界上
  double x0 = grid.front().x, x1 = x0, y0 = grid.front().y, y1 = y0;
  for (const auto & p : grid) {
    x0 = std::min(x0, p.x); x1 = std::max(x1, p.x);
    y0 = std::min(y0, p.y); y1 = std::max(y1, p.y);
  }
  EXPECT_NEAR(x0, kCx - kHalf + 0.3, 1e-9);
  EXPECT_NEAR(y0, kCy - kHalf + 0.3, 1e-9);
  EXPECT_NEAR(x1, kCx + kHalf - 0.3, 1e-9);
  EXPECT_NEAR(y1, kCy + kHalf - 0.3, 1e-9);
}

TEST(GenerateZoneGrid, DenserSpacingYieldsMorePoints)
{
  const auto zone = makeSquare(0.3);
  EXPECT_GT(
    decision::generateZoneGrid(zone, 0.25).size(),
    decision::generateZoneGrid(zone, 0.5).size());
}

TEST(GenerateZoneGrid, DegenerateInputsReturnEmptyInsteadOfCrashing)
{
  EXPECT_TRUE(decision::generateZoneGrid(decision::CenterZone{}, 0.5).empty());
  EXPECT_TRUE(decision::generateZoneGrid(makeSquare(), 0.0).empty());
  EXPECT_TRUE(decision::generateZoneGrid(makeSquare(), -1.0).empty());
}

TEST(GenerateZoneGrid, TinyZoneStillYieldsOnePoint)
{
  // 区域比一个格距还小：至少给形心一个点。空 = 站着不动，那是最坏的结果。
  const auto tiny = decision::makeCenterZone(
    {pt(0, 0), pt(0.2, 0), pt(0.2, 0.2), pt(0, 0.2)}, 0.0);
  ASSERT_TRUE(tiny.usable);
  const auto grid = decision::generateZoneGrid(tiny, 0.5);
  ASSERT_EQ(grid.size(), 1u);
  EXPECT_NEAR(grid[0].x, 0.1, 1e-9);
  EXPECT_NEAR(grid[0].y, 0.1, 1e-9);
}

// ==================== 按顺序取点 ====================

TEST(TakePatrolPath, TakesConsecutivePointsAndAdvancesTheCursor)
{
  const auto path = decision::takePatrolPath(10, 0, 3);
  ASSERT_EQ(path.indices.size(), 3u);
  EXPECT_EQ(path.indices[0], 0u);
  EXPECT_EQ(path.indices[1], 1u);
  EXPECT_EQ(path.indices[2], 2u);
  EXPECT_EQ(path.cursor, 3u);
}

TEST(TakePatrolPath, WrapsAroundAtTheEnd)
{
  const auto path = decision::takePatrolPath(10, 8, 3);
  ASSERT_EQ(path.indices.size(), 3u);
  EXPECT_EQ(path.indices[0], 8u);
  EXPECT_EQ(path.indices[1], 9u);
  EXPECT_EQ(path.indices[2], 0u);
  EXPECT_EQ(path.cursor, 1u);
}

TEST(TakePatrolPath, CapsAtPoolSizeAndNeverRepeatsWithinAPath)
{
  const auto path = decision::takePatrolPath(10, 4, 50);
  EXPECT_EQ(path.indices.size(), 10u);
  const std::set<std::size_t> unique(path.indices.begin(), path.indices.end());
  EXPECT_EQ(unique.size(), path.indices.size());
  EXPECT_EQ(path.cursor, 4u);
}

TEST(TakePatrolPath, RejectsDegenerateInputs)
{
  EXPECT_TRUE(decision::takePatrolPath(0, 0, 3).indices.empty());
  EXPECT_TRUE(decision::takePatrolPath(10, 0, 0).indices.empty());
  EXPECT_TRUE(decision::takePatrolPath(10, 0, -1).indices.empty());
}

TEST(TakePatrolPath, OutOfRangeCursorIsTakenModulo)
{
  // 残留的旧游标不能越界读
  const auto path = decision::takePatrolPath(4, 9, 2);
  ASSERT_EQ(path.indices.size(), 2u);
  EXPECT_EQ(path.indices[0], 1u);
  EXPECT_EQ(path.indices[1], 2u);
}

TEST(TakePatrolPath, ConsecutiveTriggersCoverTheWholeGrid)
{
  // 连续触发若干次应该把整个网格走一遍 —— 这就是"在区域内来回走"的全部含义。
  const std::size_t pool = 9;
  std::size_t cursor = 0;
  std::set<std::size_t> visited;
  for (int trigger = 0; trigger < 3; ++trigger) {
    const auto path = decision::takePatrolPath(pool, cursor, 3);
    cursor = path.cursor;
    visited.insert(path.indices.begin(), path.indices.end());
  }
  EXPECT_EQ(visited.size(), pool);
  EXPECT_EQ(cursor, 0u) << "走完一整圈游标应该回到起点";
}

}  // namespace

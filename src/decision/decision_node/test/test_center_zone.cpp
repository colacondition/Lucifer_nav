// 中心占领区的夹取与判定。
//
// 这是"追击限制在区域内"的核心：夹错了的后果不是"不好看"，而是**占领权当场丢掉**
// （车被放到区外），或者哨兵贴着一条不存在的边界不动。所以边界情况要钉死。

#include <vector>

#include <gtest/gtest.h>

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

// 边长 2.4 的正方形，中心 (2.6, 4.8) —— 形状参照真实中心区（半边长 ≥1.2 是从
// patrol.csv 推出来的下界）。
constexpr double kCx = 2.6;
constexpr double kCy = 4.8;
constexpr double kHalf = 1.2;
constexpr double kMargin = 0.3;

std::vector<decision::Pose> squareCorners()
{
  return {
    pt(kCx - kHalf, kCy - kHalf), pt(kCx + kHalf, kCy - kHalf),
    pt(kCx + kHalf, kCy + kHalf), pt(kCx - kHalf, kCy + kHalf)};
}

decision::CenterZone makeSquare(double margin = kMargin)
{
  return decision::makeCenterZone(squareCorners(), margin);
}

TEST(MakeCenterZone, AcceptsAnyCornerOrder)
{
  // 角点顺序是人手录的，顺/逆时针、从哪个角开始都不该有影响。
  const auto canonical = squareCorners();
  std::vector<std::vector<decision::Pose>> orders = {
    canonical,
    {canonical[3], canonical[2], canonical[1], canonical[0]},   // 反向
    {canonical[1], canonical[3], canonical[0], canonical[2]},   // 打乱
    {canonical[2], canonical[0], canonical[3], canonical[1]},   // 打乱
  };
  for (const auto & corners : orders) {
    const auto zone = decision::makeCenterZone(corners, kMargin);
    EXPECT_TRUE(zone.usable);
    ASSERT_EQ(zone.corners.size(), 4u);
    // 排完之后必须是逆时针（叉积同号）
    EXPECT_TRUE(decision::zoneContains(zone, pt(kCx, kCy)));
  }
}

TEST(MakeCenterZone, RejectsDegenerateInput)
{
  const auto ok = squareCorners();
  // 点数不对
  EXPECT_FALSE(decision::makeCenterZone({ok[0], ok[1], ok[2]}, kMargin).usable);
  EXPECT_FALSE(decision::makeCenterZone({}, kMargin).usable);
  // 有重合点
  EXPECT_FALSE(decision::makeCenterZone({ok[0], ok[0], ok[2], ok[3]}, kMargin).usable);
  // 共线
  EXPECT_FALSE(
    decision::makeCenterZone({pt(0, 0), pt(1, 0), pt(2, 0), pt(3, 0)}, kMargin).usable);
  // 非凸（把第四个点放到内部）
  EXPECT_FALSE(
    decision::makeCenterZone({pt(0, 0), pt(2, 0), pt(2, 2), pt(1, 1)}, kMargin).usable);
}

TEST(MakeCenterZone, MarginShrinksTheSafeZone)
{
  // margin = 0 → 安全区 = 原区域
  const auto zero = decision::makeCenterZone(squareCorners(), 0.0);
  EXPECT_TRUE(zero.usable);
  EXPECT_TRUE(decision::zoneContains(zero, pt(kCx + kHalf - 1e-6, kCy)));

  // margin = 0.3 → 原边界往里 0.3 就不算安全区了
  const auto zone = makeSquare();
  EXPECT_TRUE(zone.usable);
  EXPECT_FALSE(decision::zoneContains(zone, pt(kCx + kHalf - 0.1, kCy)));
  EXPECT_TRUE(decision::zoneContains(zone, pt(kCx + kHalf - kMargin - 0.05, kCy)));
}

TEST(MakeCenterZone, MarginTooLargeIsUnusable)
{
  // 余量比半边长还大 → 安全区缩没了。此时必须 usable=false，
  // 而不是给出一个空/翻转的区域（那会把车夹到一个荒谬的位置）。
  EXPECT_FALSE(decision::makeCenterZone(squareCorners(), kHalf + 0.1).usable);
}

TEST(ZoneContains, InsideOutsideAndBoundary)
{
  const auto zone = makeSquare();
  EXPECT_TRUE(decision::zoneContains(zone, pt(kCx, kCy)));                  // 中心
  EXPECT_TRUE(decision::zoneContains(zone, pt(kCx - 0.8, kCy + 0.8)));      // 内部
  EXPECT_FALSE(decision::zoneContains(zone, pt(kCx + 1.0, kCy)));           // 东边外
  EXPECT_FALSE(decision::zoneContains(zone, pt(kCx, kCy - 1.0)));           // 南边外
  EXPECT_FALSE(decision::zoneContains(zone, pt(0.0, 0.0)));                 // 远处
  // 安全区边界上算"在内"（容差内），刚出去一点算"在外"
  EXPECT_TRUE(decision::zoneContains(zone, pt(kCx + kHalf - kMargin, kCy)));
  EXPECT_FALSE(decision::zoneContains(zone, pt(kCx + kHalf - kMargin + 0.01, kCy)));
}

TEST(ClampToZone, KeepsInsidePointsUntouched)
{
  const auto zone = makeSquare();
  const auto inside = pt(kCx + 0.2, kCy - 0.3);
  const auto clamped = decision::clampToZone(zone, inside);
  EXPECT_DOUBLE_EQ(clamped.x, inside.x);
  EXPECT_DOUBLE_EQ(clamped.y, inside.y);
}

TEST(ClampToZone, PushesOutsidePointsToTheNearestEdge)
{
  const auto zone = makeSquare();

  // 正东很远 → 夹到东边中段
  auto c = decision::clampToZone(zone, pt(10.0, kCy));
  EXPECT_NEAR(c.x, kCx + kHalf - kMargin, 1e-9);
  EXPECT_NEAR(c.y, kCy, 1e-9);

  // 正北 → 北边中段
  c = decision::clampToZone(zone, pt(kCx, kCy + 5.0));
  EXPECT_NEAR(c.x, kCx, 1e-9);
  EXPECT_NEAR(c.y, kCy + kHalf - kMargin, 1e-9);

  // 东北方向 → 夹到**角落**（这是"逐轴 clamp"和"最近点 clamp"的区别所在）
  c = decision::clampToZone(zone, pt(kCx + 5.0, kCy + 5.0));
  EXPECT_NEAR(c.x, kCx + kHalf - kMargin, 1e-9);
  EXPECT_NEAR(c.y, kCy + kHalf - kMargin, 1e-9);
}

TEST(ClampToZone, ClampsJustOutsideTheMarginBackIn)
{
  // 在原始区域内、但在安全区外：也要被拉回安全区 —— 这正是余量存在的意义
  const auto zone = makeSquare();
  const auto c = decision::clampToZone(zone, pt(kCx + kHalf - 0.05, kCy));
  EXPECT_NEAR(c.x, kCx + kHalf - kMargin, 1e-9);
  EXPECT_NEAR(c.y, kCy, 1e-9);
  EXPECT_TRUE(decision::zoneContains(zone, c));
}

TEST(ClampToZone, UnusableZoneReturnsInputUnchanged)
{
  // usable=false 时原样返回，调用方**必须先检查 usable**，不能把它当"已夹取"
  const auto bad = decision::makeCenterZone(squareCorners(), kHalf + 1.0);
  ASSERT_FALSE(bad.usable);
  const auto p = pt(99.0, -99.0);
  const auto c = decision::clampToZone(bad, p);
  EXPECT_DOUBLE_EQ(c.x, p.x);
  EXPECT_DOUBLE_EQ(c.y, p.y);
}

TEST(ZoneContainsAll, ValidatesRecordedCornersAgainstKnownInsidePoints)
{
  // 这就是拿 patrol.csv 反查角点的机制：那些点本来就应该全在区内。
  const auto zone = makeSquare(0.0);
  const std::vector<decision::Pose> inside = {
    pt(kCx - 1.0, kCy - 1.0), pt(kCx + 1.0, kCy + 1.0), pt(kCx, kCy)};
  EXPECT_TRUE(decision::zoneContainsAll(zone, inside, 0.0));

  // 混进一个区外的点 → 整体判定失败（角点录错的表现）
  auto with_outside = inside;
  with_outside.push_back(pt(kCx + 3.0, kCy));
  EXPECT_FALSE(decision::zoneContainsAll(zone, with_outside, 0.0));
}

TEST(ZoneContainsAll, HandlesEmptyInputAndUnusableZone)
{
  const auto zone = makeSquare(0.0);
  EXPECT_TRUE(decision::zoneContainsAll(zone, {}, 0.0));   // 空集：平凡为真
  const auto bad = decision::makeCenterZone(squareCorners(), kHalf + 1.0);
  EXPECT_FALSE(decision::zoneContainsAll(bad, {pt(kCx, kCy)}, 0.0));
}

}  // namespace

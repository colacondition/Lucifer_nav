#include "sfc_corridor.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace navigation2
{
namespace
{

// 造一张 width x height 的网格，把 rects 里的矩形（格坐标，含端点）标成致命。
std::vector<std::uint8_t> makeLethal(
  int width, int height,
  const std::vector<std::array<int, 4>> & rects)
{
  std::vector<std::uint8_t> lethal(
    static_cast<std::size_t>(width) * static_cast<std::size_t>(height), 0U);
  for (const auto & r : rects) {
    for (int y = r[1]; y <= r[3]; ++y) {
      for (int x = r[0]; x <= r[2]; ++x) {
        if (x >= 0 && x < width && y >= 0 && y < height) {
          lethal[static_cast<std::size_t>(y) * width + x] = 1U;
        }
      }
    }
  }
  return lethal;
}

SfcCorridorParams defaultParams()
{
  SfcCorridorParams p;
  p.max_range = 10.0;
  p.robot_radius = 0.0;
  p.obstacle_threshold = 100;
  p.unknown_is_lethal = false;
  return p;
}

// 把世界坐标换成格中心（origin=0 时格 i 的中心是 (i+0.5)*res）。
double cellCenter(int i, double res) { return (static_cast<double>(i) + 0.5) * res; }

TEST(SfcCorridor, EmptyMapGivesMaxRadius)
{
  SfcCorridor sfc;
  const int w = 41, h = 41;
  const double res = 0.1;
  SfcCorridorParams p = defaultParams();
  p.max_range = 1.0;  // 10 格，小于地图边界允许的 20 格
  sfc.updateRaw(w, h, res, 0.0, 0.0, makeLethal(w, h, {}), p);
  ASSERT_TRUE(sfc.ready());
  EXPECT_NEAR(
    sfc.clearanceRadius(cellCenter(20, res), cellCenter(20, res)), 1.0, 1e-9);
}

// max_range 取任意大的有限值都安全：钳到地图尺寸，不得溢出成 UB。
TEST(SfcCorridor, HugeMaxRangeIsClampedNotOverflowed)
{
  SfcCorridor sfc;
  const int w = 41, h = 41;
  const double res = 0.1;
  SfcCorridorParams p = defaultParams();
  p.max_range = std::numeric_limits<double>::max();
  sfc.updateRaw(w, h, res, 0.0, 0.0, makeLethal(w, h, {}), p);
  ASSERT_TRUE(sfc.ready());
  // 不再受参数限制，只受地图边界：中心格到边界 20 格 = 2.0 m。
  EXPECT_NEAR(
    sfc.clearanceRadius(cellCenter(20, res), cellCenter(20, res)), 2.0, 1e-9);
}

TEST(SfcCorridor, SingleObstacleLimitsRadiusExactly)
{
  SfcCorridor sfc;
  const int w = 41, h = 41;
  const double res = 0.1;
  // 只有 (30,20) 一个致命格。半宽 h 的方形覆盖格 [20-h, 20+h]，
  // 覆盖到格 30 需要 h >= 10，所以最大可行半宽是 9 格。
  sfc.updateRaw(
    w, h, res, 0.0, 0.0, makeLethal(w, h, {{30, 20, 30, 20}}), defaultParams());
  const double radius =
    sfc.clearanceRadius(cellCenter(20, res), cellCenter(20, res));
  EXPECT_NEAR(radius, 9 * res, 1e-9);
  // 中心挪到格 21：覆盖到 30 需要 h >= 9，最大可行 8 格。
  const double radius2 =
    sfc.clearanceRadius(cellCenter(21, res), cellCenter(20, res));
  EXPECT_NEAR(radius2, 8 * res, 1e-9);
}

TEST(SfcCorridor, LethalCellReturnsNegative)
{
  SfcCorridor sfc;
  const int w = 20, h = 20;
  sfc.updateRaw(
    w, h, 0.1, 0.0, 0.0, makeLethal(w, h, {{10, 10, 10, 10}}), defaultParams());
  // 落在致命格上：没有可用空间。
  EXPECT_LT(sfc.clearanceRadius(cellCenter(10, 0.1), cellCenter(10, 0.1)), 0.0);
  EXPECT_TRUE(sfc.pointLethal(cellCenter(10, 0.1), cellCenter(10, 0.1)));
  EXPECT_FALSE(sfc.pointLethal(cellCenter(5, 0.1), cellCenter(5, 0.1)));
}

TEST(SfcCorridor, NonZeroOriginShiftsEverything)
{
  // 这是 TDT 原实现的 bug 场景：origin 非零时它的两套坐标约定互相矛盾。
  // 这里必须与 origin=0 时给出同样的几何结果，只是整体平移。
  const int w = 41, h = 41;
  const double res = 0.1;
  const std::vector<std::uint8_t> lethal = makeLethal(w, h, {{30, 20, 30, 20}});

  SfcCorridor shifted;
  SfcCorridorParams p = defaultParams();
  // 与 GridGeometry 一致：origin 是 (0,0) 格的角点。
  const double ox = -2.48, oy = -8.65;
  shifted.updateRaw(w, h, res, ox, oy, lethal, p);

  SfcCorridor plain;
  plain.updateRaw(w, h, res, 0.0, 0.0, lethal, p);

  // 同一个格（20,20）：平移后的世界坐标 vs 原点地图的世界坐标。
  const double wx_shift = ox + cellCenter(20, res);
  const double wy_shift = oy + cellCenter(20, res);
  const double wx_plain = cellCenter(20, res);
  const double wy_plain = cellCenter(20, res);
  EXPECT_NEAR(shifted.clearanceRadius(wx_shift, wy_shift),
    plain.clearanceRadius(wx_plain, wy_plain), 1e-9);
  EXPECT_EQ(shifted.pointLethal(wx_shift, wy_shift),
    plain.pointLethal(wx_plain, wy_plain));
  EXPECT_EQ(shifted.insideMap(wx_shift, wy_shift), true);
  // origin 平移不改变图外判定。
  EXPECT_EQ(shifted.insideMap(ox - 1.0, oy + cellCenter(20, res)), false);
}

TEST(SfcCorridor, UnknownCellsFollowFlag)
{
  SfcCorridorParams p = defaultParams();
  std::vector<std::uint8_t> lethal(20 * 20, 0U);
  // 用 raw 入口无法表达 -1；这里直接验证 unknown_is_lethal 开关在 updateGrid 生效。
  p.unknown_is_lethal = true;
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.width = 20;
  grid.info.height = 20;
  grid.info.resolution = 0.1F;
  grid.info.origin.position.x = 0.0;
  grid.info.origin.position.y = 0.0;
  grid.data.assign(20 * 20, 0);
  grid.data[15 * 20 + 15] = -1;  // 未知

  SfcCorridor sfc;
  sfc.updateGrid(grid, p);
  EXPECT_LT(sfc.clearanceRadius(cellCenter(15, 0.1), cellCenter(15, 0.1)), 0.0);

  // allow_unknown 语义：未知不算致命时同一格恢复可用。
  p.unknown_is_lethal = false;
  sfc.updateGrid(grid, p);
  EXPECT_GT(sfc.clearanceRadius(cellCenter(15, 0.1), cellCenter(15, 0.1)), 0.0);
}

TEST(SfcCorridor, InflationBelowThresholdIsNotLethal)
{
  // 膨胀层给 1..99，规划器的 obstacle_threshold=100 只有致命格禁行。
  // 本类必须与规划器同一取值，否则会把 A* 走过的膨胀格判成不可行。
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.width = 20;
  grid.info.height = 20;
  grid.info.resolution = 0.1F;
  grid.data.assign(20 * 20, 0);
  grid.data[10 * 20 + 10] = 99;   // 膨胀
  grid.data[12 * 20 + 12] = 100;  // 致命

  SfcCorridorParams p = defaultParams();
  p.obstacle_threshold = 100;
  SfcCorridor sfc;
  sfc.updateGrid(grid, p);
  EXPECT_FALSE(sfc.pointLethal(cellCenter(10, 0.1), cellCenter(10, 0.1)));
  EXPECT_TRUE(sfc.pointLethal(cellCenter(12, 0.1), cellCenter(12, 0.1)));
}

TEST(SfcCorridor, RobotRadiusInsetsUsableBox)
{
  SfcCorridor sfc;
  const int w = 41, h = 41;
  const double res = 0.1;
  sfc.updateRaw(
    w, h, res, 0.0, 0.0, makeLethal(w, h, {{31, 20, 31, 20}}), defaultParams());

  SfcCorridorParams p = defaultParams();
  p.robot_radius = 0.25;  // 2.5 格
  SfcCorridor inset;
  inset.updateRaw(w, h, res, 0.0, 0.0, makeLethal(w, h, {{31, 20, 31, 20}}), p);

  const double cx = cellCenter(20, res), cy = cellCenter(20, res);
  // 自由半宽 10 格 = 1.0 m；车体半径 0.25 m，可用半宽 0.75 m。
  EXPECT_NEAR(sfc.clearanceRadius(cx, cy), 1.0, 1e-9);
  const SfcBox box = inset.usableBoxAt(cx, cy);
  ASSERT_FALSE(box.degenerate());
  EXPECT_NEAR(box.halfWidth(), 0.75, 1e-9);
  EXPECT_NEAR(box.x1 - cx, 0.75, 1e-9);

  // 车体半径超过自由半宽时退化。
  SfcCorridorParams big = defaultParams();
  big.robot_radius = 5.0;
  SfcCorridor too_big;
  too_big.updateRaw(w, h, res, 0.0, 0.0, makeLethal(w, h, {{31, 20, 31, 20}}), big);
  EXPECT_TRUE(too_big.usableBoxAt(cx, cy).degenerate());
}

TEST(SfcCorridor, PathClearRejectsNearObstacle)
{
  SfcCorridorParams p = defaultParams();
  p.robot_radius = 0.25;
  SfcCorridor sfc;
  const int w = 60, h = 60;
  const double res = 0.1;
  sfc.updateRaw(
    w, h, res, 0.0, 0.0, makeLethal(w, h, {{40, 20, 40, 20}}), p);

  const std::vector<Eigen::Vector2d> far_path{
    {cellCenter(5, res), cellCenter(5, res)},
    {cellCenter(15, res), cellCenter(15, res)},
    {cellCenter(25, res), cellCenter(25, res)}};
  EXPECT_TRUE(sfc.pathClear(far_path, p.robot_radius));

  // 有一点致命格在旁边：距 40 格的致命格 13 格 = 1.3 m > 0.25 m，仍然安全。
  const std::vector<Eigen::Vector2d> near_path{
    {cellCenter(27, res), cellCenter(20, res)}};
  EXPECT_TRUE(sfc.pathClear(near_path, p.robot_radius));

  // 贴上去：距致命格 1 格 = 0.1 m < 0.25 m，必须拒绝。
  const std::vector<Eigen::Vector2d> touching{
    {cellCenter(39, res), cellCenter(20, res)}};
  EXPECT_FALSE(sfc.pathClear(touching, p.robot_radius));

  // 图外无法判定，必须拒绝而不是静默放行。
  const std::vector<Eigen::Vector2d> outside{{-5.0, -5.0}};
  EXPECT_FALSE(sfc.pathClear(outside, p.robot_radius));
}

TEST(SfcCorridor, InvalidParamsStayUnready)
{
  // max_range 非有限 / 非正必须拒绝（有限大值由 clearanceRadius 钳到地图尺寸）。
  SfcCorridor sfc;
  const int w = 20, h = 20;
  SfcCorridorParams nan_param = defaultParams();
  nan_param.max_range = std::numeric_limits<double>::quiet_NaN();
  sfc.updateRaw(w, h, 0.1, 0.0, 0.0, makeLethal(w, h, {}), nan_param);
  EXPECT_FALSE(sfc.ready());

  SfcCorridorParams zero = defaultParams();
  zero.max_range = 0.0;
  sfc.updateRaw(w, h, 0.1, 0.0, 0.0, makeLethal(w, h, {}), zero);
  EXPECT_FALSE(sfc.ready());

  SfcCorridorParams negative = defaultParams();
  negative.max_range = -1.0;
  sfc.updateRaw(w, h, 0.1, 0.0, 0.0, makeLethal(w, h, {}), negative);
  EXPECT_FALSE(sfc.ready());

  // 非法之后给合法参数必须能恢复，不能卡在未就绪状态。
  sfc.updateRaw(w, h, 0.1, 0.0, 0.0, makeLethal(w, h, {}), defaultParams());
  EXPECT_TRUE(sfc.ready());
}

TEST(SfcCorridor, SameContentSkipsRebuild)
{
  // 内容哈希命中时不重建积分图（行为上不可直接观测，但可用「更新后查询一致」
  // 钉住语义：重复 update 同一张图不能改变结果）。
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.width = 30;
  grid.info.height = 30;
  grid.info.resolution = 0.1F;
  grid.data.assign(30 * 30, 0);
  grid.data[20 * 30 + 20] = 100;

  SfcCorridorParams p = defaultParams();
  SfcCorridor sfc;
  sfc.updateGrid(grid, p);
  const double first =
    sfc.clearanceRadius(cellCenter(10, 0.1), cellCenter(10, 0.1));
  // header 时间戳变化但内容不变 —— 必须仍然命中。
  grid.header.stamp.sec++;
  sfc.updateGrid(grid, p);
  EXPECT_NEAR(sfc.clearanceRadius(cellCenter(10, 0.1), cellCenter(10, 0.1)), first, 1e-12);

  // 内容变了必须重建。
  grid.data[5 * 30 + 5] = 100;
  sfc.updateGrid(grid, p);
  EXPECT_LT(sfc.clearanceRadius(cellCenter(5, 0.1), cellCenter(5, 0.1)), 0.0);
}

}  // namespace
}  // namespace navigation2

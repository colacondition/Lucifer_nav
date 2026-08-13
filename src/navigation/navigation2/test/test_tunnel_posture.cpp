// 收云台判定的测试。
//
// 这里锁的是「什么时候把标志位置真」。两个失败方向不对等：早收一点只是少打一会儿，
// 漏收是云台撞在顶板上。所以测试重点在「提前量够不够」和「洞里全程不放」，而不是
// 「有没有多收」。

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "tunnel_posture.hpp"

namespace navigation2
{
namespace
{

constexpr std::uint8_t kFlat = static_cast<std::uint8_t>(TerrainType::FLAT);
constexpr std::uint8_t kTunnel = static_cast<std::uint8_t>(TerrainType::TUNNEL);

// 一条沿 x 轴、宽 1 格的隧道，占 x ∈ [x_from, x_to) 的 tunnel_row 行。
// origin 非零：Lucifer 的地图 origin 是 [-2.48, -8.65]，漏掉它整张图偏 8 米。
SemanticMap makeMap(
  int width, int height, double resolution, double origin_x, double origin_y, int tunnel_row,
  int x_from, int x_to, double run_up)
{
  GridGeometry geometry;
  geometry.width = width;
  geometry.height = height;
  geometry.resolution = resolution;
  geometry.origin = Eigen::Vector2d(origin_x, origin_y);

  const std::size_t cells = geometry.cellCount();
  std::vector<std::uint8_t> terrain(cells, kFlat);
  std::vector<std::uint8_t> angle(cells, 0);
  std::vector<std::uint8_t> magnitude(cells, 0);
  std::vector<std::uint8_t> cost(cells, 0);
  std::vector<std::uint8_t> ids(cells, 0);

  for (int x = x_from; x < x_to; ++x) {
    const std::size_t index = geometry.index(x, tunnel_row);
    terrain[index] = kTunnel;
    magnitude[index] = 255;  // 本体，超过 0.95 阈值
    ids[index] = 1;
  }

  TunnelSpec spec;
  spec.clear_height = 0.4;
  spec.clear_width = 0.3;
  spec.run_up = run_up;
  return SemanticMap::fromChannels(
    geometry, terrain, angle, magnitude, cost, {spec}, ids);
}

TEST(NearestTunnelBody, ReturnsNulloptWhenMapHasNoTunnel)
{
  // 完全没收到地图。
  EXPECT_FALSE(nearestTunnelBody(SemanticMap{}, 0.0, 0.0, 1.0).has_value());
  // 收到了但图里没隧道。
  const SemanticMap empty = makeMap(20, 20, 0.05, 0.0, 0.0, 5, 5, 5, 0.5);
  EXPECT_FALSE(nearestTunnelBody(empty, 0.25, 0.25, 1.0).has_value());
}

TEST(NearestTunnelBody, MeasuresDistanceInWorldCoordinates)
{
  // 隧道在第 10 行，x ∈ [10, 14)：世界 y 中心 = -8.65 + 10.5*0.05 = -8.125。
  const SemanticMap map = makeMap(30, 30, 0.05, -2.48, -8.65, 10, 10, 14, 0.5);

  // 正好在某个本体格中心上。
  const auto on_cell = nearestTunnelBody(map, -2.48 + 10.5 * 0.05, -8.125, 1.0);
  ASSERT_TRUE(on_cell.has_value());
  EXPECT_NEAR(on_cell->distance, 0.0, 1e-9);
  ASSERT_NE(on_cell->spec, nullptr);
  EXPECT_DOUBLE_EQ(on_cell->spec->run_up, 0.5);

  // 横向退开 0.30 米。origin 参与换算，用格号做距离会算出完全不同的值。
  const auto offset = nearestTunnelBody(map, -2.48 + 10.5 * 0.05, -8.125 - 0.30, 1.0);
  ASSERT_TRUE(offset.has_value());
  EXPECT_NEAR(offset->distance, 0.30, 1e-9);
}

TEST(NearestTunnelBody, SearchBoxCoversOwnCellEvenWithTinyRadius)
{
  // 搜索半径比一格还小时仍要找到车所在的那一格 —— 否则「已经在洞里」会漏判，
  // 而那正是绝对不能漏的情形。
  const SemanticMap map = makeMap(30, 30, 0.05, 0.0, 0.0, 10, 10, 14, 0.5);
  const auto found = nearestTunnelBody(map, 10.5 * 0.05, 10.5 * 0.05, 0.001);
  ASSERT_TRUE(found.has_value());
  EXPECT_NEAR(found->distance, 0.0, 1e-9);
}

TEST(NearestTunnelBody, SearchBoxRoundsUpToCoverTheFullRadius)
{
  // 半径不是分辨率整数倍时，格数必须向上取整。截断会让恰好落在半径内的隧道格掉到
  // 框外 —— 提前量凭空少一格，而这一格就是 run_up 的余量。
  //
  // 隧道在第 20 行 x ∈ [20, 24)，分辨率 0.05。车摆在 x = 1.025（正对第 20 格中心）、
  // y = 0.84 —— 落在第 16 格里，跟隧道行差 4 格。
  //
  // 半径取 0.19：0.19 / 0.05 = 3.8，向上取整是 4（框覆盖到第 20 行，找得到），
  // 截断是 3（框只到第 19 行，整条隧道被漏掉）。到本体格中心的真实距离是
  // 1.025 - 0.84 = 0.185，确实在 0.19 之内 —— 所以漏掉纯粹是取整的错。
  const SemanticMap map = makeMap(30, 30, 0.05, 0.0, 0.0, 20, 20, 24, 0.5);
  const double robot_x = 20.5 * 0.05;
  const double robot_y = 0.84;

  const auto found = nearestTunnelBody(map, robot_x, robot_y, 0.19);
  ASSERT_TRUE(found.has_value()) << "向上取整没做：半径内的隧道格被截断出了搜索框";
  EXPECT_NEAR(found->distance, 0.185, 1e-9);
}

TEST(NearestTunnelBody, DoesNotCrashWhenRobotIsOutsideTheMap)
{
  const SemanticMap map = makeMap(30, 30, 0.05, 0.0, 0.0, 10, 10, 14, 0.5);
  // 图外远处：不该崩，也不该在搜索框里找到东西。
  EXPECT_FALSE(nearestTunnelBody(map, 100.0, 100.0, 1.0).has_value());
  EXPECT_FALSE(nearestTunnelBody(map, -100.0, -100.0, 1.0).has_value());
}

TEST(PathCrossesTunnel, ReturnsTrueWhenAnyPointIsInsideTheTunnelBody)
{
  const SemanticMap map = makeMap(60, 60, 0.05, 0.0, 0.0, 20, 20, 30, 0.5);
  const double tunnel_y = 20.5 * 0.05;
  // 横穿：洞口前 → 洞中 → 出洞后，中间那个点落在本体格内。
  const std::vector<Eigen::Vector2d> crossing{
    {19.0 * 0.05, tunnel_y}, {25.0 * 0.05, tunnel_y}, {31.0 * 0.05, tunnel_y}};
  EXPECT_TRUE(pathCrossesTunnel(map, crossing));
}

TEST(PathCrossesTunnel, ReturnsFalseWhenPathSkipsTheTunnel)
{
  const SemanticMap map = makeMap(60, 60, 0.05, 0.0, 0.0, 20, 20, 30, 0.5);
  // 贴着洞口上方路过：y 抬高 0.5 米，没有任何点落进本体格。
  const double bypass_y = 20.5 * 0.05 + 0.5;
  const std::vector<Eigen::Vector2d> bypass{
    {19.0 * 0.05, bypass_y}, {25.0 * 0.05, bypass_y}, {31.0 * 0.05, bypass_y}};
  EXPECT_FALSE(pathCrossesTunnel(map, bypass));
  // 还没收到 /plan：空路径也不该算穿洞。
  EXPECT_FALSE(pathCrossesTunnel(map, {}));
}

TEST(GimbalLowerDecider, LowersRunUpMetresBeforeTheTunnel)
{
  // run_up 0.5：距本体 0.5 米内就该请求收云台。
  const SemanticMap map = makeMap(60, 60, 0.05, 0.0, 0.0, 20, 20, 30, 0.5);
  const double tunnel_y = 20.5 * 0.05;
  const double entrance_x = 20.0 * 0.05;

  GimbalLowerDecider decider(0.3);
  // 离洞口 1.2 米，还早。
  EXPECT_FALSE(decider.update(map, entrance_x - 1.2, tunnel_y));
  // 进到 run_up 之内。
  EXPECT_TRUE(decider.update(map, entrance_x - 0.4, tunnel_y));
}

TEST(GimbalLowerDecider, StaysLoweredThroughTheWholeTunnel)
{
  // 洞里全程不能放 —— 请求覆盖整条本体，不是只在入口那一条线上。
  const SemanticMap map = makeMap(60, 60, 0.05, 0.0, 0.0, 20, 20, 40, 0.5);
  const double tunnel_y = 20.5 * 0.05;

  GimbalLowerDecider decider(0.3);
  for (int cell = 20; cell < 40; ++cell) {
    const double x = (cell + 0.5) * 0.05;
    EXPECT_TRUE(decider.update(map, x, tunnel_y)) << "cell x=" << cell;
  }
}

TEST(GimbalLowerDecider, RaisesAfterLeavingTheTunnel)
{
  const SemanticMap map = makeMap(60, 60, 0.05, 0.0, 0.0, 20, 20, 30, 0.5);
  const double tunnel_y = 20.5 * 0.05;
  const double exit_x = 30.0 * 0.05;

  GimbalLowerDecider decider(0.3);
  ASSERT_TRUE(decider.update(map, exit_x - 0.1, tunnel_y));
  // 出洞后退开 run_up + 滞回还多一截，必须回到 false，否则出了洞云台再也抬不起来。
  EXPECT_FALSE(decider.update(map, exit_x + 1.2, tunnel_y));
}

TEST(GimbalLowerDecider, HysteresisPreventsFlappingAtTheThreshold)
{
  // 判定跑在 10~20 Hz 上。阈值附近来回抖会让电控那边云台反复抬落，所以抬起的门槛
  // 必须比落下的高一档。
  const SemanticMap map = makeMap(60, 60, 0.05, 0.0, 0.0, 20, 20, 30, 0.5);
  const double tunnel_y = 20.5 * 0.05;
  const double entrance_x = 20.0 * 0.05;

  GimbalLowerDecider decider(0.3);
  // 刚好进到阈值内。
  ASSERT_TRUE(decider.update(map, entrance_x - 0.45, tunnel_y));
  // 退回到阈值外一点点：没有滞回时这里会翻成 false。
  EXPECT_TRUE(decider.update(map, entrance_x - 0.6, tunnel_y));
  // 退过滞回带才放。
  EXPECT_FALSE(decider.update(map, entrance_x - 0.9, tunnel_y));
}

TEST(GimbalLowerDecider, StaysFalseWhenMapHasNoTunnel)
{
  const SemanticMap map = makeMap(60, 60, 0.05, 0.0, 0.0, 20, 20, 20, 0.5);
  GimbalLowerDecider decider(0.3);
  EXPECT_FALSE(decider.update(map, 1.0, 1.0));
  EXPECT_FALSE(decider.update(SemanticMap{}, 1.0, 1.0));
}

TEST(GimbalLowerDecider, UsesPerTunnelRunUp)
{
  // run_up 是逐隧道的：搜索框按图里最大的 run_up 取，但阈值必须用找到的那条隧道
  // 自己的值。写死一个全局提前量的话，这条测试里 2.0 m 的隧道会提前不足。
  const SemanticMap map = makeMap(80, 80, 0.05, 0.0, 0.0, 30, 30, 40, 2.0);
  const double tunnel_y = 30.5 * 0.05;
  const double entrance_x = 30.0 * 0.05;

  GimbalLowerDecider decider(0.3);
  EXPECT_TRUE(decider.update(map, entrance_x - 1.8, tunnel_y));
  EXPECT_FALSE(decider.update(map, entrance_x - 2.6, tunnel_y));
}

TEST(GimbalLowerDecider, DoesNotLowerNearTunnelWhenPathSkipsIt)
{
  // 车贴到洞口但路径不穿洞（will_cross=false）：不该收。这正是跟旧「附近有没有隧道」
  // 纯几何判据的区别 —— 贴着洞口路过（不进洞）不该损失火力。
  const SemanticMap map = makeMap(60, 60, 0.05, 0.0, 0.0, 20, 20, 30, 0.5);
  const double tunnel_y = 20.5 * 0.05;
  const double entrance_x = 20.0 * 0.05;

  GimbalLowerDecider decider(0.3);
  EXPECT_FALSE(decider.update(map, entrance_x - 0.4, tunnel_y, /*will_cross=*/false));
}

TEST(GimbalLowerDecider, LowersNearTunnelWhenPathCrossesIt)
{
  // will_cross=true 时保留旧行为：run_up 内收。这里显式传参，锁住「穿洞」是收的充分
  // 条件（加上 inside 兜底才是必要条件）。
  const SemanticMap map = makeMap(60, 60, 0.05, 0.0, 0.0, 20, 20, 30, 0.5);
  const double tunnel_y = 20.5 * 0.05;
  const double entrance_x = 20.0 * 0.05;

  GimbalLowerDecider decider(0.3);
  EXPECT_FALSE(decider.update(map, entrance_x - 1.2, tunnel_y, /*will_cross=*/true));
  EXPECT_TRUE(decider.update(map, entrance_x - 0.4, tunnel_y, /*will_cross=*/true));
}

TEST(GimbalLowerDecider, LowersInsideTunnelEvenWhenPathDoesNotCross)
{
  // inside 兜底：车已经在本体里，路径判 false 也必须收。路径过期/重规划漏发时，
  // 车在洞里而云台立着撞顶板的代价不可逆，这个兜底不能丢。
  const SemanticMap map = makeMap(60, 60, 0.05, 0.0, 0.0, 20, 20, 30, 0.5);
  const double tunnel_y = 20.5 * 0.05;
  const double mid_x = 25.0 * 0.05;

  GimbalLowerDecider decider(0.3);
  EXPECT_TRUE(decider.update(map, mid_x, tunnel_y, /*will_cross=*/false));
}

}  // namespace
}  // namespace navigation2

// 语义地图消费端的测试。
//
// 这里验证的是「语义栅格 → 消费端栅格」这层换算，而不是语义地图本身（那部分在
// test_semantic_map.cpp）。两边分辨率和 origin 都不一样，错位一格在真机上表现为
// 顶板在洞口边缘照旧被标成障碍，很难从日志看出来，所以必须有测试盯着。

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "grid_utils.hpp"
#include "semantic_map_consumer.hpp"

namespace navigation2
{
namespace
{

constexpr std::uint8_t kFlat = static_cast<std::uint8_t>(TerrainType::FLAT);
constexpr std::uint8_t kObstacle = static_cast<std::uint8_t>(TerrainType::OBSTACLE);
constexpr std::uint8_t kTunnel = static_cast<std::uint8_t>(TerrainType::TUNNEL);

// 一条沿 x 轴、宽 1 格的隧道，穿过 y = tunnel_row。
decision_interfaces::msg::SemanticMap makeTunnelMsg(
  int width, int height, double resolution, double origin_x, double origin_y, int tunnel_row,
  double clear_height, double clear_width)
{
  decision_interfaces::msg::SemanticMap msg;
  msg.width = static_cast<std::uint32_t>(width);
  msg.height = static_cast<std::uint32_t>(height);
  msg.resolution = resolution;
  msg.origin_x = origin_x;
  msg.origin_y = origin_y;

  const std::size_t cells = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  msg.terrain.assign(cells, kFlat);
  msg.direction_angle.assign(cells, 0);
  msg.direction_magnitude.assign(cells, 0);
  msg.cost.assign(cells, 0);
  msg.tunnel_ids.assign(cells, 0);

  for (int x = 0; x < width; ++x) {
    const std::size_t index =
      static_cast<std::size_t>(tunnel_row) * static_cast<std::size_t>(width) +
      static_cast<std::size_t>(x);
    msg.terrain[index] = kTunnel;
    // 轴线沿 +x，本体模长必须超过 0.95 的阈值才算「车在隧道上」。
    msg.direction_angle[index] = 0;
    msg.direction_magnitude[index] = 255;
    msg.tunnel_ids[index] = 1;
  }

  decision_interfaces::msg::TunnelSpec spec;
  spec.clear_height = clear_height;
  spec.clear_width = clear_width;
  spec.run_up = 0.5;
  spec.velocity_min = 0.3;
  spec.velocity_max = 0.8;
  msg.tunnels.push_back(spec);
  return msg;
}

nav_msgs::msg::OccupancyGrid makeGrid(
  int width, int height, double resolution, double origin_x, double origin_y)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.width = static_cast<unsigned int>(width);
  grid.info.height = static_cast<unsigned int>(height);
  grid.info.resolution = static_cast<float>(resolution);
  grid.info.origin.position.x = origin_x;
  grid.info.origin.position.y = origin_y;
  grid.info.origin.orientation = quaternionFromYaw(0.0);
  grid.data.assign(static_cast<std::size_t>(width) * static_cast<std::size_t>(height), 0);
  return grid;
}

TEST(SemanticMapConsumer, RoundTripsGeometryAndTunnels)
{
  const auto msg = makeTunnelMsg(10, 6, 0.05, -2.48, -8.65, 3, 0.4, 0.6);
  const SemanticMap map = semanticMapFromMsg(msg);

  ASSERT_TRUE(map.valid());
  EXPECT_EQ(map.geometry().width, 10);
  EXPECT_EQ(map.geometry().height, 6);
  EXPECT_DOUBLE_EQ(map.geometry().resolution, 0.05);
  // origin 非零是 Lucifer 的实际情况（RMUC 是 [-2.48, -8.65]），漏掉它整张图偏 8 米。
  EXPECT_DOUBLE_EQ(map.geometry().origin.x(), -2.48);
  EXPECT_DOUBLE_EQ(map.geometry().origin.y(), -8.65);
  ASSERT_EQ(map.tunnels().size(), 1U);
  EXPECT_DOUBLE_EQ(map.tunnels()[0].clear_height, 0.4);
  EXPECT_DOUBLE_EQ(map.tunnels()[0].clear_width, 0.6);
  EXPECT_TRUE(map.isTunnelBodyCell(4, 3));
  EXPECT_FALSE(map.isTunnelBodyCell(4, 2));
}

TEST(SemanticMapConsumer, RejectsChannelLengthMismatch)
{
  auto msg = makeTunnelMsg(10, 6, 0.05, 0.0, 0.0, 3, 0.4, 0.6);
  msg.cost.pop_back();
  // 通道短一格就必须整帧拒收：拿半张图去规划会让隧道格错位到别的位置，在错误的
  // 地方放过顶板点云。
  EXPECT_THROW(semanticMapFromMsg(msg), std::runtime_error);
}

TEST(SemanticMapConsumer, RejectsInvalidGeometry)
{
  auto msg = makeTunnelMsg(10, 6, 0.05, 0.0, 0.0, 3, 0.4, 0.6);
  msg.resolution = 0.0;
  EXPECT_THROW(semanticMapFromMsg(msg), std::runtime_error);
}

TEST(SemanticMapConsumer, FindsTunnelSpecByWorldCoordinate)
{
  // 语义图 0.05 m/格，origin 非零：隧道行的中心在 y = -8.65 + 3.5*0.05 = -8.475。
  const auto msg = makeTunnelMsg(10, 6, 0.05, -2.48, -8.65, 3, 0.4, 0.6);
  const SemanticMap map = semanticMapFromMsg(msg);

  const TunnelSpec * inside = tunnelSpecAtPoint(map, -2.48 + 0.22, -8.475);
  ASSERT_NE(inside, nullptr);
  EXPECT_DOUBLE_EQ(inside->clear_height, 0.4);

  // 隔一行就不在本体内了。
  EXPECT_EQ(tunnelSpecAtPoint(map, -2.48 + 0.22, -8.475 - 0.05), nullptr);
  // 图外返回 nullptr 而不是崩。
  EXPECT_EQ(tunnelSpecAtPoint(map, 100.0, 100.0), nullptr);
}

TEST(SemanticMapConsumer, LimitsInflationInsideTunnelOnly)
{
  // 消费端栅格故意用不同的分辨率（0.02 vs 0.05）和不同的 origin，逼着换算走
  // 世界坐标而不是格号。
  const auto msg = makeTunnelMsg(10, 6, 0.05, 0.0, 0.0, 3, 0.4, 0.6);
  const SemanticMap map = semanticMapFromMsg(msg);

  auto grid = makeGrid(25, 20, 0.02, 0.0, 0.0);
  const double default_radius = 0.5;
  const double robot_radius = 0.25;
  const auto limits = makeInflationRadiusLimit(grid, map, default_radius, robot_radius);
  ASSERT_EQ(limits.size(), grid.data.size());

  // 隧道行覆盖世界 y ∈ [0.15, 0.20)，在 0.02 的栅格里是 y = 7、8、9 三行
  // （中心 0.15/0.17/0.19）。净宽 0.6 减车宽 0.5 后余量只有 0.05。
  for (int x = 0; x < 25; ++x) {
    EXPECT_FLOAT_EQ(limits[gridIndex(grid, x, 8)], 0.05F) << "tunnel row at x=" << x;
    // 洞外仍是全局半径 —— 压小半径不能泄漏到隧道以外。
    EXPECT_FLOAT_EQ(limits[gridIndex(grid, x, 3)], static_cast<float>(default_radius));
  }
}

TEST(SemanticMapConsumer, ReturnsNoLimitWhenMapHasNoTunnel)
{
  auto msg = makeTunnelMsg(10, 6, 0.05, 0.0, 0.0, 3, 0.4, 0.6);
  msg.terrain.assign(msg.terrain.size(), kFlat);
  msg.direction_magnitude.assign(msg.direction_magnitude.size(), 0);
  msg.tunnel_ids.assign(msg.tunnel_ids.size(), 0);
  msg.tunnels.clear();
  const SemanticMap map = semanticMapFromMsg(msg);

  auto grid = makeGrid(25, 20, 0.02, 0.0, 0.0);
  // 空 vector 表示「无逐格上限」，让常见情形（图里没隧道）零开销。
  EXPECT_TRUE(makeInflationRadiusLimit(grid, map, 0.5, 0.25).empty());
  // 完全没收到语义地图时同理。
  EXPECT_TRUE(makeInflationRadiusLimit(grid, SemanticMap{}, 0.5, 0.25).empty());
}

TEST(SemanticMapConsumer, ClampsLimitToZeroWhenTunnelNarrowerThanRobot)
{
  // 洞比车还窄：膨胀层不该在这里替规划器做否决，压到 0 就行 —— 真过不去会在壁面的
  // 致命格上挡住。
  const auto msg = makeTunnelMsg(10, 6, 0.05, 0.0, 0.0, 3, 0.4, 0.3);
  const SemanticMap map = semanticMapFromMsg(msg);

  auto grid = makeGrid(25, 20, 0.02, 0.0, 0.0);
  const auto limits = makeInflationRadiusLimit(grid, map, 0.5, 0.25);
  ASSERT_EQ(limits.size(), grid.data.size());
  EXPECT_FLOAT_EQ(limits[gridIndex(grid, 5, 8)], 0.0F);
}

TEST(SemanticMapReceiverTest, RebuildsOnlyWhenContentChanges)
{
  const auto msg = makeTunnelMsg(10, 6, 0.05, 0.0, 0.0, 3, 0.4, 0.6);
  SemanticMapReceiver receiver;

  EXPECT_FALSE(receiver.map().valid());
  EXPECT_TRUE(receiver.update(msg));
  EXPECT_TRUE(receiver.map().valid());

  // 同一张图重发：rm_map_server 每秒都会这么干，必须不重建。
  EXPECT_FALSE(receiver.update(msg));

  // header 变了但内容没变，仍然不算变 —— 重发时时间戳必然不同，把它算进去等于
  // 彻底禁用缓存。
  auto restamped = msg;
  restamped.header.stamp.sec = msg.header.stamp.sec + 5;
  restamped.header.frame_id = "other_frame";
  EXPECT_FALSE(receiver.update(restamped));

  // 真换了地图就必须重建。
  auto changed = makeTunnelMsg(10, 6, 0.05, 0.0, 0.0, 2, 0.4, 0.6);
  EXPECT_TRUE(receiver.update(changed));
  EXPECT_TRUE(receiver.map().isTunnelBodyCell(4, 2));
  EXPECT_FALSE(receiver.map().isTunnelBodyCell(4, 3));

  // 只有隧道参数变（栅格一模一样）也算变：净高直接决定顶板放行的阈值。
  auto retuned = changed;
  retuned.tunnels[0].clear_height = 0.9;
  EXPECT_TRUE(receiver.update(retuned));
  ASSERT_NE(receiver.map().tunnelSpecAtCell(4, 2), nullptr);
  EXPECT_DOUBLE_EQ(receiver.map().tunnelSpecAtCell(4, 2)->clear_height, 0.9);
}

TEST(SemanticMapReceiverTest, KeepsPreviousMapWhenNewMessageIsBad)
{
  const auto good = makeTunnelMsg(10, 6, 0.05, 0.0, 0.0, 3, 0.4, 0.6);
  SemanticMapReceiver receiver;
  ASSERT_TRUE(receiver.update(good));

  auto bad = good;
  bad.terrain.pop_back();
  EXPECT_THROW(receiver.update(bad), std::runtime_error);
  // 一帧坏消息不该把隧道语义整个丢掉 —— 顶板会立刻重新封住洞口。
  EXPECT_TRUE(receiver.map().valid());
  EXPECT_TRUE(receiver.map().isTunnelBodyCell(4, 3));
  // 坏消息也不能被记成「上一条」，否则同样的坏消息第二次就被当成「没变」而静默放过。
  EXPECT_THROW(receiver.update(bad), std::runtime_error);
}

TEST(SemanticMapConsumer, PerCellLimitKeepsTunnelInteriorFreeOfDistantInflation)
{
  // 端到端：一条被墙夹住的窄隧道，验证逐格上限确实让通道中线保持低代价。
  auto grid = makeGrid(20, 11, 0.05, 0.0, 0.0);
  // y=4 和 y=6 是墙，y=5 是通道。
  for (int x = 0; x < 20; ++x) {
    grid.data[gridIndex(grid, x, 4)] = 100;
    grid.data[gridIndex(grid, x, 6)] = 100;
  }

  decision_interfaces::msg::SemanticMap msg =
    makeTunnelMsg(20, 11, 0.05, 0.0, 0.0, 5, 0.4, 0.35);
  for (int x = 0; x < 20; ++x) {
    msg.terrain[static_cast<std::size_t>(4 * 20 + x)] = kObstacle;
    msg.terrain[static_cast<std::size_t>(6 * 20 + x)] = kObstacle;
  }
  const SemanticMap map = semanticMapFromMsg(msg);

  auto without_limit = grid;
  applyInflationCostGradient(without_limit, 0.5, 50, 3.0);
  auto with_limit = grid;
  applyInflationCostGradient(
    with_limit, 0.5, 50, 3.0, makeInflationRadiusLimit(with_limit, map, 0.5, 0.25));

  const std::size_t center = gridIndex(grid, 10, 5);
  // 不设上限时通道中线被两侧墙涂上可观代价。
  EXPECT_GT(without_limit.data[center], 50);
  // 设了上限之后中线不再被远处的墙影响。净宽 0.35 减车宽 0.5 → 余量 0，
  // 半径 0 的核只剩中心格自己，而中心格不是障碍。
  EXPECT_EQ(with_limit.data[center], 0);
  // 墙本身仍然是致命的 —— 压小膨胀半径不等于放开碰撞。
  EXPECT_EQ(with_limit.data[gridIndex(grid, 10, 4)], 100);
}

// ---- 轴线表：A* 的「只沿轴穿隧道」判据 ----------------------------------

TEST(TunnelAxisGrid, IsEmptyWithoutASemanticMap)
{
  // 没有语义地图时表是空的，A* 会整段跳过轴向判定退回纯几何。
  const auto grid = makeGrid(10, 10, 0.05, 0.0, 0.0);
  const auto axis = TunnelAxisGrid::build(grid, SemanticMap{});
  EXPECT_TRUE(axis.empty());
}

TEST(TunnelAxisGrid, IsEmptyWhenTheMapHasNoTunnel)
{
  // 常见情形：地图有效但没标隧道。表必须是空的，否则 A* 会为每一步白算一次点积。
  auto msg = makeTunnelMsg(10, 6, 0.05, 0.0, 0.0, 3, 0.4, 0.6);
  msg.terrain.assign(msg.terrain.size(), kFlat);
  msg.direction_magnitude.assign(msg.direction_magnitude.size(), 0);
  msg.tunnel_ids.assign(msg.tunnel_ids.size(), 0);

  const auto grid = makeGrid(10, 6, 0.05, 0.0, 0.0);
  const auto axis = TunnelAxisGrid::build(grid, semanticMapFromMsg(msg));
  EXPECT_TRUE(axis.empty());
}

// 轴向对齐度是软代价的输入（A* 里每步加 w * (1 - alignment)），不是通行的硬判据 ——
// 车是圆柱，朝向不影响能不能过，斜着穿洞只是该少走。下面用 0.9 做对比只是为了表达
// 「直步比斜步对齐得多」这个量级关系。
constexpr double kWellAligned = 0.9;

TEST(TunnelAxisGrid, OnlyAxialStepsAreAlignedInsideATunnel)
{
  // 沿 +x 的隧道在 y=3：直步完全对齐，斜步和横步对齐度递减。
  const auto msg = makeTunnelMsg(10, 6, 0.05, 0.0, 0.0, 3, 0.4, 0.6);
  const auto grid = makeGrid(10, 6, 0.05, 0.0, 0.0);
  const auto axis = TunnelAxisGrid::build(grid, semanticMapFromMsg(msg));
  ASSERT_FALSE(axis.empty());

  const std::size_t a = gridIndex(grid, 4, 3);
  const std::size_t b = gridIndex(grid, 5, 3);
  ASSERT_TRUE(axis.isTunnelCell(a));
  ASSERT_TRUE(axis.isTunnelCell(b));

  // 沿轴：完全对齐。
  EXPECT_NEAR(axis.stepAlignment(a, b, 1, 0), 1.0, 1e-6);
  // 斜切：|cos 45°| ≈ 0.707 —— A* 会给这一步加上可观的代价，于是整体走向被压向轴线。
  EXPECT_NEAR(axis.stepAlignment(a, gridIndex(grid, 5, 4), 1, 1), std::sqrt(0.5), 1e-6);
  EXPECT_LT(axis.stepAlignment(a, gridIndex(grid, 5, 4), 1, 1), kWellAligned);
  // 垂直于轴线：完全不对齐。
  EXPECT_NEAR(axis.stepAlignment(a, gridIndex(grid, 4, 4), 0, 1), 0.0, 1e-6);
}

TEST(TunnelAxisGrid, BackwardStepsAreAlignedToo)
{
  // 隧道是双向的：倒着穿和正着穿一样合法。这是隧道跟台阶的根本差异 —— 台阶的方向
  // 是「上行」，有正反之分，判据得用 cos θ；隧道用 |cos θ|。
  //
  // 判据里的绝对值掉了，这个测试会挂，而 OnlyAxialSteps 那个不会 —— 所以它得单独存在。
  const auto msg = makeTunnelMsg(10, 6, 0.05, 0.0, 0.0, 3, 0.4, 0.6);
  const auto grid = makeGrid(10, 6, 0.05, 0.0, 0.0);
  const auto axis = TunnelAxisGrid::build(grid, semanticMapFromMsg(msg));

  const std::size_t a = gridIndex(grid, 4, 3);
  const std::size_t b = gridIndex(grid, 3, 3);
  EXPECT_NEAR(axis.stepAlignment(a, b, -1, 0), 1.0, 1e-6);
}

TEST(TunnelAxisGrid, StepsFullyOutsideTunnelsAreUnconstrained)
{
  // 洞外不管方向，否则整张图都只能沿隧道轴线走。
  const auto msg = makeTunnelMsg(10, 6, 0.05, 0.0, 0.0, 3, 0.4, 0.6);
  const auto grid = makeGrid(10, 6, 0.05, 0.0, 0.0);
  const auto axis = TunnelAxisGrid::build(grid, semanticMapFromMsg(msg));

  const std::size_t a = gridIndex(grid, 4, 0);
  const std::size_t b = gridIndex(grid, 5, 1);
  ASSERT_FALSE(axis.isTunnelCell(a));
  ASSERT_FALSE(axis.isTunnelCell(b));
  EXPECT_NEAR(axis.stepAlignment(a, b, 1, 1), 1.0, 1e-6);
}

TEST(TunnelAxisGrid, TheEntryStepIsJudgedByTheTunnelAxis)
{
  // 起点在洞外、终点在洞内。入洞这一步必须按洞内的轴线算 —— 只判「两端都在洞内」
  // 的话，斜着切进洞口的那一步代价为 0，等于洞口那一格完全不受约束。
  const auto msg = makeTunnelMsg(10, 6, 0.05, 0.0, 0.0, 3, 0.4, 0.6);
  const auto grid = makeGrid(10, 6, 0.05, 0.0, 0.0);
  const auto axis = TunnelAxisGrid::build(grid, semanticMapFromMsg(msg));

  const std::size_t outside = gridIndex(grid, 4, 2);
  const std::size_t inside = gridIndex(grid, 5, 3);
  ASSERT_FALSE(axis.isTunnelCell(outside));
  ASSERT_TRUE(axis.isTunnelCell(inside));
  EXPECT_LT(axis.stepAlignment(outside, inside, 1, 1), kWellAligned);

  // 出洞的那一步同样按洞内轴线算（此时只有起点在洞内）。
  EXPECT_LT(axis.stepAlignment(inside, gridIndex(grid, 6, 4), 1, 1), kWellAligned);
}

TEST(TunnelAxisGrid, InflationRingIsNotTreatedAsTunnel)
{
  // 膨胀圈的方向信息是弱的（模长 ≤ 0.9），只该用于 MINCO 的软塑形。拿它做 A* 的
  // 硬否决会在洞口外面就开始拦转向，车根本对不进洞口。
  auto msg = makeTunnelMsg(10, 6, 0.05, 0.0, 0.0, 3, 0.4, 0.6);
  for (int x = 0; x < 10; ++x) {
    const std::size_t index = static_cast<std::size_t>(2 * 10 + x);
    msg.terrain[index] = kTunnel;
    msg.direction_angle[index] = 0;
    // 229/255 ≈ 0.898，正好是膨胀圈的上限。
    msg.direction_magnitude[index] = 229;
    msg.tunnel_ids[index] = 1;
  }

  const auto grid = makeGrid(10, 6, 0.05, 0.0, 0.0);
  const auto axis = TunnelAxisGrid::build(grid, semanticMapFromMsg(msg));

  EXPECT_TRUE(axis.isTunnelCell(gridIndex(grid, 4, 3)));
  EXPECT_FALSE(axis.isTunnelCell(gridIndex(grid, 4, 2)));
}

TEST(TunnelAxisGrid, MapsAcrossDifferingResolutionAndOrigin)
{
  // 语义地图是 0.05 m/格、origin [-2.48, -8.65]（RMUC 的实际值），代价地图是
  // 0.02 m/格、origin 不同。错位一格在真机上表现为洞口边缘的方向判定用错轴线。
  const auto msg = makeTunnelMsg(10, 6, 0.05, -2.48, -8.65, 3, 0.4, 0.6);
  const auto map = semanticMapFromMsg(msg);

  // 语义格 y=3 覆盖世界 y ∈ [-8.50, -8.45)。代价地图取 origin y = -8.50，
  // 0.02 m/格，于是它的 y=0 和 y=1 两行落在这条隧道里，y=3 之后不在。
  const auto grid = makeGrid(10, 6, 0.02, -2.48, -8.50);
  const auto axis = TunnelAxisGrid::build(grid, map);
  ASSERT_FALSE(axis.empty());

  EXPECT_TRUE(axis.isTunnelCell(gridIndex(grid, 3, 0)));
  EXPECT_TRUE(axis.isTunnelCell(gridIndex(grid, 3, 1)));
  EXPECT_FALSE(axis.isTunnelCell(gridIndex(grid, 3, 4)));

  // 细格上沿轴走仍然是对齐的 —— 轴线来自语义格，跟消费端的分辨率无关。
  EXPECT_NEAR(
    axis.stepAlignment(gridIndex(grid, 3, 0), gridIndex(grid, 4, 0), 1, 0), 1.0, 1e-6);
}

TEST(TunnelAxisGrid, DiagonalTunnelAcceptsTheStepsThatFollowItsAxis)
{
  // 实际地图里的洞是直的、正对栅格轴的，这个测试用一条 45° 的洞是为了钉住「轴线取自
  // 地图数据」这件事：把判据写成「侧移就是不对齐」时，所有沿 x 的直洞测试都照样过，
  // 只有这里会挂。
  auto msg = makeTunnelMsg(12, 12, 0.05, 0.0, 0.0, 0, 0.4, 0.6);
  msg.terrain.assign(msg.terrain.size(), kFlat);
  msg.direction_magnitude.assign(msg.direction_magnitude.size(), 0);
  msg.tunnel_ids.assign(msg.tunnel_ids.size(), 0);
  for (int i = 1; i < 11; ++i) {
    const std::size_t index = static_cast<std::size_t>(i * 12 + i);
    msg.terrain[index] = kTunnel;
    // 编码 0~255 → 0~2π，45° 是 32。
    msg.direction_angle[index] = 32;
    msg.direction_magnitude[index] = 255;
    msg.tunnel_ids[index] = 1;
  }

  const auto grid = makeGrid(12, 12, 0.05, 0.0, 0.0);
  const auto axis = TunnelAxisGrid::build(grid, semanticMapFromMsg(msg));
  ASSERT_FALSE(axis.empty());

  const std::size_t a = gridIndex(grid, 5, 5);
  ASSERT_TRUE(axis.isTunnelCell(a));
  // 沿 45° 轴线的斜步：对齐，不加代价。
  EXPECT_GT(axis.stepAlignment(a, gridIndex(grid, 6, 6), 1, 1), 0.99);
  // 沿栅格轴的直步：|cos 45°| ≈ 0.707 —— 在这种洞里反而是该加代价的一步。
  EXPECT_LT(axis.stepAlignment(a, gridIndex(grid, 6, 5), 1, 0), kWellAligned);
}

}  // namespace
}  // namespace navigation2

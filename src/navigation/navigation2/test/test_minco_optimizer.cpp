#include "minco/minco_optimizer.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <cmath>
#include <vector>

namespace navigation2
{
namespace
{

bool finitePieces(const std::vector<Piece<5, 2>> & pieces)
{
  for (const auto & piece : pieces) {
    if (!std::isfinite(piece.getDuration()) || piece.getDuration() <= 0.0) return false;
    for (int i = 0; i <= 5; ++i) {
      const double t = piece.getDuration() * static_cast<double>(i) / 5.0;
      if (!piece.getPos(t).allFinite() || !piece.getVel(t).allFinite() ||
        !piece.getAcc(t).allFinite()) return false;
    }
  }
  return true;
}

TEST(MincoOptimizer, ReusesOneOptimizerAcrossTwoStages)
{
  MincoOptimizer optimizer;
  MincoOptimizer::Params coarse;
  coarse.smooth_weight = 0.5;
  coarse.data_weight = 10.0;
  coarse.lbfgs_memory_size = 32;
  coarse.max_iterations = 800;
  optimizer.setParams(coarse);

  const std::vector<Eigen::Vector2d> waypoints = {
    {0.0, 0.0}, {1.0, 0.0}, {1.2, 0.8}, {2.0, 1.0}};
  const std::vector<double> coarse_times = {0.6, 0.7, 0.6};
  const auto first = optimizer.optimize(waypoints, coarse_times);
  ASSERT_EQ(first.size(), coarse_times.size());
  ASSERT_TRUE(finitePieces(first));

  auto fine = coarse;
  fine.obstacle_normal_only = true;
  fine.max_iterations = 300;
  optimizer.setParams(fine);
  const std::vector<double> fine_times = {0.8, 1.0, 0.8};
  const auto second = optimizer.optimize(waypoints, fine_times);
  ASSERT_EQ(second.size(), fine_times.size());
  EXPECT_TRUE(finitePieces(second));
}

TEST(MincoOptimizer, RejectsInvalidSegmentTime)
{
  MincoOptimizer optimizer;
  const std::vector<Eigen::Vector2d> waypoints = {{0.0, 0.0}, {1.0, 0.0}};
  EXPECT_TRUE(optimizer.optimize(waypoints, {0.0}).empty());
}

// 隧道横向走廊项：把一条明显偏轴的路径拉回轴线。
// 几何取自 RMUL 的真实隧道（净宽 0.5 m、车体半径 0.25 m -> 洞内半宽 0）。
TEST(MincoOptimizer, TunnelCorridorPullsPathOntoTheAxis)
{
  // 轴沿 +y，质心在原点，半长 1.2 m；洞内半宽 0（车体直径 == 净宽）。
  const Eigen::Vector2d axis_dir(0.0, 1.0);
  const Eigen::Vector2d centroid(0.0, 0.0);
  const double half_len = 1.2;
  const double lateral_outer = 0.25 + 0.20;  // clear_width/2 + margin

  auto corridor = [&](const Eigen::Vector2d & p, TunnelCorridorFrame & frame) {
      frame.centroid = centroid;
      frame.dir = axis_dir;
      frame.half_len = half_len;
      frame.half_width_inner = 0.0;
      frame.lateral_outer = lateral_outer;
      // 只在影响区内给约束（模拟 TunnelRegionGrid::specNearPoint 的语义）。
      const Eigen::Vector2d e = p - centroid;
      const double along = std::abs(e.dot(axis_dir));
      const double lat = std::abs(e.x() * axis_dir.y() - e.y() * axis_dir.x());
      return along <= half_len + 1.2 && lat <= lateral_outer;
    };

  // 一条偏轴 0.20 m 的直路：起点终点固定，中间点应该被拉向轴线。
  const std::vector<Eigen::Vector2d> waypoints = {
    {0.20, -1.0}, {0.20, -0.5}, {0.20, 0.0}, {0.20, 0.5}, {0.20, 1.0}};
  const std::vector<double> times = {0.5, 0.5, 0.5, 0.5};

  MincoOptimizer without;
  MincoOptimizer::Params base;
  base.smooth_weight = 0.5;
  base.data_weight = 10.0;
  base.max_iterations = 800;
  base.tunnel_corridor_weight = 0.0;
  without.setParams(base);
  const auto free_pieces = without.optimize(waypoints, times);
  ASSERT_FALSE(free_pieces.empty());

  MincoOptimizer with;
  MincoOptimizer::Params constrained = base;
  constrained.tunnel_corridor_weight = 20.0;
  with.setParams(constrained);
  with.setTunnelCorridorQuery(corridor);
  const auto constrained_pieces = with.optimize(waypoints, times);
  ASSERT_FALSE(constrained_pieces.empty());

  // 取两条轨迹在中点的横向偏移。走廊项生效的那条必须更贴轴。
  const double mid_t = 0.5 * constrained_pieces[1].getDuration();
  const double free_lat = std::abs(free_pieces[1].getPos(mid_t).x());
  const double constrained_lat = std::abs(constrained_pieces[1].getPos(mid_t).x());
  EXPECT_LT(constrained_lat, free_lat);
  EXPECT_LT(constrained_lat, 0.20);

  // 端点固定不动：走廊项不该把首尾点拉走。
  EXPECT_NEAR(constrained_pieces.front().getPos(0.0).x(), 0.20, 1e-9);
  EXPECT_NEAR(
    constrained_pieces.back().getPos(constrained_pieces.back().getDuration()).x(),
    0.20, 1e-9);
}

// 走廊查询恒返回 false 时（图里没隧道 / 轴退化），走廊项必须完全失效 ——
// 否则「没有隧道的地图」会因为一个恒真的惩罚而整体跑偏。
TEST(MincoOptimizer, TunnelCorridorInactiveWithoutQuery)
{
  const std::vector<Eigen::Vector2d> waypoints = {
    {0.0, 0.0}, {1.0, 0.2}, {2.0, 0.0}};
  const std::vector<double> times = {0.6, 0.6};

  MincoOptimizer optimizer;
  MincoOptimizer::Params params;
  params.smooth_weight = 0.5;
  params.data_weight = 10.0;
  params.tunnel_corridor_weight = 20.0;  // 权重开着，但没有注入查询
  optimizer.setParams(params);
  const auto pieces = optimizer.optimize(waypoints, times);
  ASSERT_FALSE(pieces.empty());
  EXPECT_TRUE(finitePieces(pieces));
}

}  // namespace
}  // namespace navigation2

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

}  // namespace
}  // namespace navigation2

#include "minco_time_allocation.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <cmath>
#include <numeric>
#include <vector>

namespace navigation2
{
namespace
{

TEST(MincoTimeAllocation, StraightPathKeepsDistanceOverVelocity)
{
  const std::vector<Eigen::Vector2d> points = {
    {0.0, 0.0}, {1.0, 0.0}, {3.0, 0.0}};

  const auto times = allocateMincoSegmentTimes(points, 2.0, 0.1, 0.2);

  ASSERT_EQ(times.size(), 2u);
  EXPECT_NEAR(times[0], 0.5, 1e-12);
  EXPECT_NEAR(times[1], 1.0, 1e-12);
}

TEST(MincoTimeAllocation, TurnAddsExpectedTransitionTime)
{
  const std::vector<Eigen::Vector2d> points = {
    {0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}};

  const auto times = allocateMincoSegmentTimes(points, 1.0, 0.1, 0.2);

  ASSERT_EQ(times.size(), 2u);
  const double expected_extra = 0.2 * M_PI_2;
  EXPECT_NEAR(times[0], 1.0 + 0.5 * expected_extra, 1e-12);
  EXPECT_NEAR(times[1], 1.0 + 0.5 * expected_extra, 1e-12);
}

TEST(MincoTimeAllocation, TurnTimeIsDistributedByAdjacentLength)
{
  const std::vector<Eigen::Vector2d> points = {
    {0.0, 0.0}, {1.0, 0.0}, {1.0, 3.0}};

  const auto base = allocateMincoSegmentTimes(points, 1.0, 0.1, 0.0);
  const auto turn = allocateMincoSegmentTimes(points, 1.0, 0.1, 0.4);

  ASSERT_EQ(turn.size(), 2u);
  const double extra_left = turn[0] - base[0];
  const double extra_right = turn[1] - base[1];
  EXPECT_NEAR(extra_right / extra_left, 3.0, 1e-12);
  EXPECT_NEAR(extra_left + extra_right, 0.4 * M_PI_2, 1e-12);
}

TEST(MincoTimeAllocation, DegenerateSegmentsRemainFinite)
{
  const std::vector<Eigen::Vector2d> points = {
    {0.0, 0.0}, {0.0, 0.0}, {1.0, 0.0}};

  const auto times = allocateMincoSegmentTimes(points, 0.0, 0.1, 0.5);

  ASSERT_EQ(times.size(), 2u);
  EXPECT_TRUE(std::isfinite(times[0]));
  EXPECT_TRUE(std::isfinite(times[1]));
  EXPECT_GE(times[0], 0.1);
  EXPECT_GE(times[1], 0.1);
}

}  // namespace
}  // namespace navigation2

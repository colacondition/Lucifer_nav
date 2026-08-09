#include "distance_field.hpp"

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

namespace navigation2
{
namespace
{

nav_msgs::msg::OccupancyGrid makeGrid(int width = 5, int height = 5)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.width = static_cast<unsigned int>(width);
  grid.info.height = static_cast<unsigned int>(height);
  grid.info.resolution = 0.2F;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(static_cast<std::size_t>(width * height), 0);
  return grid;
}

std::size_t indexOf(const nav_msgs::msg::OccupancyGrid & grid, int x, int y)
{
  return static_cast<std::size_t>(y) * grid.info.width + static_cast<std::size_t>(x);
}

TEST(DistanceField, MeasuresDistanceFromObstacleCells)
{
  auto grid = makeGrid();
  grid.data[indexOf(grid, 2, 2)] = 100;

  const auto field = buildDistanceField(grid, 50, true);

  ASSERT_TRUE(field.valid());
  EXPECT_NEAR(field.distanceAt(indexOf(grid, 2, 2)), 0.0, 1e-6);
  EXPECT_NEAR(field.distanceAt(indexOf(grid, 3, 2)), 0.2, 1e-6);
  EXPECT_NEAR(field.distanceAt(indexOf(grid, 3, 3)), 0.2 * std::sqrt(2.0), 1e-6);
}

TEST(DistanceField, AppliesUnknownCellPolicy)
{
  auto grid = makeGrid();
  grid.data[indexOf(grid, 1, 1)] = -1;

  const auto unknown_blocked = buildDistanceField(grid, 50, true);
  const auto unknown_free = buildDistanceField(grid, 50, false);

  ASSERT_TRUE(unknown_blocked.valid());
  ASSERT_TRUE(unknown_free.valid());
  EXPECT_NEAR(unknown_blocked.distanceAt(indexOf(grid, 1, 1)), 0.0, 1e-6);
  EXPECT_TRUE(std::isinf(unknown_free.distanceAt(indexOf(grid, 1, 1))));
}

TEST(DistanceField, RejectsMalformedGrid)
{
  auto grid = makeGrid();
  grid.data.pop_back();

  const auto field = buildDistanceField(grid, 50, true);

  EXPECT_FALSE(field.valid());
  EXPECT_TRUE(field.distances.empty());
}

TEST(DistanceField, ClearancePenaltyIsHigherNearObstacles)
{
  const double near_penalty = clearancePenalty(0.2, 0.6, 3.0);
  const double far_penalty = clearancePenalty(0.5, 0.6, 3.0);

  EXPECT_GT(near_penalty, far_penalty);
  EXPECT_DOUBLE_EQ(clearancePenalty(0.6, 0.6, 3.0), 0.0);
  EXPECT_DOUBLE_EQ(clearancePenalty(std::numeric_limits<double>::infinity(), 0.6, 3.0), 0.0);
  EXPECT_DOUBLE_EQ(clearancePenalty(0.2, 0.6, 0.0), 0.0);
}

}  // namespace
}  // namespace navigation2

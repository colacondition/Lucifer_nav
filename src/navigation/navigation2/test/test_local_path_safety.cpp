#include "local_path_safety.hpp"
#include "utils/pose_predictor.hpp"

#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

namespace navigation2
{
namespace
{

nav_msgs::msg::OccupancyGrid makeGrid()
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.width = 10;
  grid.info.height = 10;
  grid.info.resolution = 0.1F;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(100, 0);
  return grid;
}

Eigen::Vector2d cellCenter(int x, int y)
{
  return {0.1 * (static_cast<double>(x) + 0.5), 0.1 * (static_cast<double>(y) + 0.5)};
}

TEST(LocalPathSafety, AcceptsFreePath)
{
  const auto grid = makeGrid();
  const std::vector<Eigen::Vector2d> path{cellCenter(1, 1), cellCenter(8, 1)};
  EXPECT_TRUE(isPathSafe(grid, path, LocalPathSafetyPolicy{}));
}

TEST(LocalPathSafety, RejectsObstacleAtPredictionPoint)
{
  auto grid = makeGrid();
  grid.data[5 * grid.info.width + 5] = 100;
  const std::vector<Eigen::Vector2d> path{cellCenter(1, 1), cellCenter(5, 5)};
  EXPECT_FALSE(isPathSafe(grid, path, LocalPathSafetyPolicy{}));
}

TEST(LocalPathSafety, RejectsObstacleBetweenPredictionPoints)
{
  auto grid = makeGrid();
  grid.data[1 * grid.info.width + 4] = 100;
  const std::vector<Eigen::Vector2d> path{cellCenter(1, 1), cellCenter(8, 1)};
  EXPECT_FALSE(isPathSafe(grid, path, LocalPathSafetyPolicy{}));
}

TEST(LocalPathSafety, AppliesUnknownCellPolicy)
{
  auto grid = makeGrid();
  grid.data[1 * grid.info.width + 4] = -1;
  const std::vector<Eigen::Vector2d> path{cellCenter(1, 1), cellCenter(8, 1)};

  LocalPathSafetyPolicy policy;
  policy.unknown_is_obstacle = true;
  EXPECT_FALSE(isPathSafe(grid, path, policy));

  policy.unknown_is_obstacle = false;
  EXPECT_TRUE(isPathSafe(grid, path, policy));
}

TEST(LocalPathSafety, RejectsPathOutsideMap)
{
  const auto grid = makeGrid();
  const std::vector<Eigen::Vector2d> path{cellCenter(1, 1), Eigen::Vector2d(2.0, 0.15)};
  EXPECT_FALSE(isPathSafe(grid, path, LocalPathSafetyPolicy{}));
}

TEST(LocalPathSafety, RejectsMalformedMap)
{
  auto grid = makeGrid();
  grid.data.pop_back();
  EXPECT_FALSE(isPathSafe(grid, {cellCenter(1, 1)}, LocalPathSafetyPolicy{}));
}

TEST(LocalPathSafety, RejectsNonFinitePoint)
{
  const auto grid = makeGrid();
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(
    isPathSafe(
      grid, {cellCenter(1, 1), Eigen::Vector2d(nan, 0.2)},
      LocalPathSafetyPolicy{}));
}

TEST(LocalPathSafety, LimitsCheckedPredictionSteps)
{
  auto grid = makeGrid();
  grid.data[1 * grid.info.width + 8] = 100;
  const std::vector<Eigen::Vector2d> path{
    cellCenter(1, 1), cellCenter(2, 1), cellCenter(8, 1)};

  LocalPathSafetyPolicy policy;
  policy.check_steps = 1;
  EXPECT_TRUE(isPathSafe(grid, path, policy));

  policy.check_steps = 0;
  EXPECT_FALSE(isPathSafe(grid, path, policy));
}

TEST(PosePredictor, UsesImmutableOdometrySnapshot)
{
  nav_msgs::msg::Odometry shared_odom;
  shared_odom.header.stamp.sec = 10;
  shared_odom.pose.pose.orientation.w = 1.0;
  shared_odom.twist.twist.linear.x = 2.0;
  const auto snapshot = shared_odom;

  shared_odom.twist.twist.linear.x = 100.0;
  const auto predicted = utils::predict_pose(snapshot, rclcpp::Time(10500000000LL), 1.0);

  EXPECT_NEAR(predicted.pose.position.x, 1.0, 1e-9);
}

}  // namespace
}  // namespace navigation2

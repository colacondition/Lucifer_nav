#include "path_stitching.hpp"

#include <gtest/gtest.h>

namespace navigation2
{
namespace
{

geometry_msgs::msg::PoseStamped pose(double x, double y)
{
  geometry_msgs::msg::PoseStamped out;
  out.header.frame_id = "map";
  out.pose.position.x = x;
  out.pose.position.y = y;
  out.pose.orientation.w = 1.0;
  return out;
}

nav_msgs::msg::Path path(std::initializer_list<geometry_msgs::msg::PoseStamped> poses)
{
  nav_msgs::msg::Path out;
  out.header.frame_id = "map";
  out.poses.assign(poses.begin(), poses.end());
  return out;
}

TEST(PathStitching, FindsForwardTargetPastLookahead)
{
  const auto previous = path({pose(0.0, 0.0), pose(0.5, 0.0), pose(1.0, 0.0), pose(1.5, 0.0)});
  geometry_msgs::msg::Point current;
  current.x = 0.1;
  current.y = 0.0;

  const auto target = findStitchTarget(previous, current, 0.8, 1.2);

  ASSERT_TRUE(target);
  EXPECT_EQ(target->index, 2U);
  EXPECT_NEAR(target->distance, 0.9, 1e-9);
}

TEST(PathStitching, RejectsTargetsBeyondMaximumDistance)
{
  const auto previous = path({pose(0.0, 0.0), pose(2.0, 0.0), pose(3.0, 0.0)});
  geometry_msgs::msg::Point current;
  current.x = 0.0;
  current.y = 0.0;

  const auto target = findStitchTarget(previous, current, 0.5, 1.0);

  EXPECT_FALSE(target);
}

TEST(PathStitching, AppendsPreviousTailWithoutDuplicatingJoinPose)
{
  const auto local = path({pose(0.0, 0.0), pose(0.5, 0.2), pose(1.0, 0.0)});
  const auto previous = path({pose(0.0, 0.0), pose(1.0, 0.0), pose(2.0, 0.0)});

  const auto stitched = stitchPath(local, previous, 1);

  ASSERT_EQ(stitched.poses.size(), 4U);
  EXPECT_DOUBLE_EQ(stitched.poses[0].pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(stitched.poses[1].pose.position.x, 0.5);
  EXPECT_DOUBLE_EQ(stitched.poses[2].pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(stitched.poses[3].pose.position.x, 2.0);
}

}  // namespace
}  // namespace navigation2

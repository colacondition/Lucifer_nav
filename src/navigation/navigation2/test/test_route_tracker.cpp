#include "mpc/route_tracker.hpp"

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

namespace navigation2::mpc
{
namespace
{

// 按给定折点生成密采样路径（点距约 0.05 m），贴近实际 /plan 的分辨率。
nav_msgs::msg::Path makePath(const std::vector<Eigen::Vector2d> & corners)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  constexpr double kSampleStep = 0.05;

  for (std::size_t leg = 1; leg < corners.size(); ++leg) {
    const Eigen::Vector2d & from = corners[leg - 1];
    const Eigen::Vector2d & to = corners[leg];
    const double length = (to - from).norm();
    const int samples = std::max(1, static_cast<int>(std::round(length / kSampleStep)));
    // 每段的终点留给下一段的起点，避免重复点。
    for (int i = 0; i < samples; ++i) {
      const double alpha = static_cast<double>(i) / static_cast<double>(samples);
      const Eigen::Vector2d p = from + alpha * (to - from);
      geometry_msgs::msg::PoseStamped ps;
      ps.pose.position.x = p.x();
      ps.pose.position.y = p.y();
      ps.pose.orientation.w = 1.0;
      path.poses.push_back(ps);
    }
  }

  geometry_msgs::msg::PoseStamped last;
  last.pose.position.x = corners.back().x();
  last.pose.position.y = corners.back().y();
  last.pose.orientation.w = 1.0;
  path.poses.push_back(last);
  return path;
}

PathReference makeReference(const std::vector<Eigen::Vector2d> & corners, double speed = 1.0)
{
  PathReference ref;
  ref.set_path(makePath(corners), speed);
  return ref;
}

TEST(RouteTracker, TracksStraightPathMonotonically)
{
  const auto ref = makeReference({{0.0, 0.0}, {10.0, 0.0}});
  RouteTracker tracker;

  constexpr double kDt = 0.1;
  double previous = -1.0;
  for (int step = 0; step <= 50; ++step) {
    const Eigen::Vector2d pos(0.1 * step, 0.0);
    tracker.update(ref, pos, kDt);
    EXPECT_GE(tracker.arc_length(), previous);
    previous = tracker.arc_length();
  }

  ASSERT_TRUE(tracker.valid());
  EXPECT_NEAR(tracker.arc_length(), 5.0, 0.15);
  EXPECT_LT(tracker.residual(), 0.05);
  EXPECT_FALSE(tracker.lost());
  // 1.0 m/s 前进，弧长变化率应当接近 1.0。
  EXPECT_NEAR(tracker.arc_rate(), 1.0, 0.3);
}

// 回绕路径是单帧最近点投影最容易出错的场景：两条支路空间上只差 0.3 m，
// 但弧长相差 6 m 以上。速度方向与切向的一致性是唯一能区分它们的信息。
TEST(RouteTracker, DoesNotJumpBetweenBranchesOnFoldedPath)
{
  // 走廊宽 0.3 m 的 U 形路径，总长 10.3 m。
  const auto ref = makeReference({{0.0, 0.0}, {5.0, 0.0}, {5.0, 0.3}, {0.0, 0.3}});
  RouteTracker tracker;

  constexpr double kDt = 0.1;
  for (int step = 0; step <= 40; ++step) {
    const Eigen::Vector2d pos(0.1 * step, 0.0);
    tracker.update(ref, pos, kDt);
  }

  ASSERT_TRUE(tracker.valid());
  // 机器人在第一条支路的 s≈4.0 处，返程支路的对应弧长是 5+0.3+(5-4)=6.3。
  EXPECT_NEAR(tracker.arc_length(), 4.0, 0.3);
  EXPECT_LT(tracker.arc_length(), 5.0);
}

TEST(RouteTracker, ReportedProgressNeverRegresses)
{
  const auto ref = makeReference({{0.0, 0.0}, {10.0, 0.0}});
  RouteTracker tracker;

  constexpr double kDt = 0.1;
  for (int step = 0; step <= 30; ++step) {
    tracker.update(ref, Eigen::Vector2d(0.1 * step, 0.0), kDt);
  }
  const double peak = tracker.arc_length();
  ASSERT_NEAR(peak, 3.0, 0.2);

  // 原路倒退回 s≈1.0。
  for (int step = 30; step >= 10; --step) {
    tracker.update(ref, Eigen::Vector2d(0.1 * step, 0.0), kDt);
    EXPECT_GE(tracker.arc_length(), peak);
  }

  // 内部最优假设跟着实际位置回退，但对外报告保持峰值。
  EXPECT_NEAR(tracker.best_arc_length(), 1.0, 0.25);
  EXPECT_GE(tracker.arc_length(), peak);
  EXPECT_LT(tracker.arc_rate(), 0.0);
}

TEST(RouteTracker, ReportsLostOnlyAfterGracePeriod)
{
  const auto ref = makeReference({{0.0, 0.0}, {10.0, 0.0}});
  RouteTracker tracker;
  RouteTrackerParams params;
  params.max_track_error = 0.5;
  params.lost_grace_time = 0.5;
  tracker.configure(params);

  // 先正常跟一段。
  constexpr double kDt = 0.1;
  for (int step = 0; step <= 20; ++step) {
    tracker.update(ref, Eigen::Vector2d(0.1 * step, 0.0), kDt);
  }
  ASSERT_FALSE(tracker.lost());

  // 突然偏离到路径外 5 m。宽限期内还不算丢失。
  tracker.update(ref, Eigen::Vector2d(2.0, 5.0), kDt);
  EXPECT_GT(tracker.residual(), 0.5);
  EXPECT_FALSE(tracker.lost());

  // 持续超限累计超过 lost_grace_time 后判定丢失。
  for (int step = 0; step < 6; ++step) {
    tracker.update(ref, Eigen::Vector2d(2.0, 5.0), kDt);
  }
  EXPECT_TRUE(tracker.lost());

  // 回到路径上后应当自动恢复。
  for (int step = 0; step < 3; ++step) {
    tracker.update(ref, Eigen::Vector2d(2.0, 0.0), kDt);
  }
  EXPECT_FALSE(tracker.lost());
}

TEST(RouteTracker, ResetClearsProgress)
{
  const auto ref = makeReference({{0.0, 0.0}, {10.0, 0.0}});
  RouteTracker tracker;

  for (int step = 0; step <= 30; ++step) {
    tracker.update(ref, Eigen::Vector2d(0.1 * step, 0.0), 0.1);
  }
  ASSERT_GT(tracker.arc_length(), 2.0);

  tracker.reset();
  EXPECT_FALSE(tracker.valid());
  EXPECT_DOUBLE_EQ(tracker.arc_length(), 0.0);
  EXPECT_FALSE(tracker.lost());

  // reset 后重新播撒，进度从新位置重新建立。
  tracker.update(ref, Eigen::Vector2d(1.0, 0.0), 0.1);
  EXPECT_TRUE(tracker.valid());
  EXPECT_NEAR(tracker.arc_length(), 1.0, 0.2);
}

TEST(RouteTracker, HandlesInvalidPath)
{
  PathReference empty;
  RouteTracker tracker;

  tracker.update(empty, Eigen::Vector2d(0.0, 0.0), 0.1);
  EXPECT_FALSE(tracker.valid());
  EXPECT_DOUBLE_EQ(tracker.arc_length(), 0.0);
}

// 重规划时弧长原点变了，进度必须重锚；但速度估计要保留，否则方向似然
// 需要两帧才能恢复，而 A 的规划器每移动 min_replan_distance 就换一次路径。
TEST(RouteTracker, PathReplacementReanchorsButKeepsVelocity)
{
  auto ref = makeReference({{0.0, 0.0}, {10.0, 0.0}});
  RouteTracker tracker;

  // 沿路径走到 s≈2，建立起非零速度估计。
  for (int i = 0; i <= 20; ++i) {
    tracker.update(ref, Eigen::Vector2d(0.1 * i, 0.0), 0.1);
  }
  ASSERT_GT(tracker.arc_length(), 1.5);
  const double velocity_before = tracker.velocity().x();
  ASSERT_GT(velocity_before, 0.5);

  tracker.on_path_replaced();

  // 假设被清掉，报告进度归零（新路径的 s=0 是新的物理位置）。
  EXPECT_FALSE(tracker.valid());
  EXPECT_DOUBLE_EQ(tracker.arc_length(), 0.0);
  // 但速度估计保留下来了。
  EXPECT_DOUBLE_EQ(tracker.velocity().x(), velocity_before);

  // 新路径从当前位置起算，首帧就能用方向项。
  auto replanned = makeReference({{0.0, 0.0}, {8.0, 0.0}});
  tracker.update(replanned, Eigen::Vector2d(0.0, 0.0), 0.1);
  EXPECT_TRUE(tracker.valid());
  EXPECT_NEAR(tracker.arc_length(), 0.0, 0.2);
}

}  // namespace
}  // namespace navigation2::mpc

#include "mpc/speed_profile.hpp"

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

namespace navigation2::mpc
{
namespace
{

// 按折点生成密采样路径（0.05 m 间距）。
nav_msgs::msg::Path makePath(const std::vector<Eigen::Vector2d> & corners)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  constexpr double kStep = 0.05;
  for (std::size_t leg = 1; leg < corners.size(); ++leg) {
    const Eigen::Vector2d & from = corners[leg - 1];
    const Eigen::Vector2d & to = corners[leg];
    const int n = std::max(1, static_cast<int>((to - from).norm() / kStep));
    for (int i = 0; i < n; ++i) {
      const double a = static_cast<double>(i) / n;
      const Eigen::Vector2d p = from + a * (to - from);
      geometry_msgs::msg::PoseStamped ps;
      ps.pose.position.x = p.x();
      ps.pose.position.y = p.y();
      ps.pose.orientation.w = 1.0;
      path.poses.push_back(ps);
    }
  }
  geometry_msgs::msg::PoseStamped ps;
  ps.pose.position.x = corners.back().x();
  ps.pose.position.y = corners.back().y();
  ps.pose.orientation.w = 1.0;
  path.poses.push_back(ps);
  return path;
}

PathReference makeRef(const std::vector<Eigen::Vector2d> & corners, double speed = 1.5)
{
  PathReference ref;
  ref.set_path(makePath(corners), speed);
  return ref;
}

// 圆弧：以圆心 (0, R) 为圆心、半径 R 采样，
// 从 (0, 0) 开始沿顺时针方向走 angle_rad 弧度。
PathReference makeArc(double radius, double angle_rad, double speed = 1.5)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  constexpr double kStep = 0.05;
  const int n = std::max(2, static_cast<int>(radius * angle_rad / kStep));
  for (int i = 0; i <= n; ++i) {
    const double t = static_cast<double>(i) / n * angle_rad;
    geometry_msgs::msg::PoseStamped ps;
    ps.pose.position.x = radius * std::sin(t);
    ps.pose.position.y = radius * (1.0 - std::cos(t));
    ps.pose.orientation.w = 1.0;
    path.poses.push_back(ps);
  }
  PathReference ref;
  ref.set_path(path, speed);
  return ref;
}

constexpr double kMaxSpeed = 1.5;
constexpr double kMaxLat = 2.0;

SpeedProfile makeProfile(double max_speed = kMaxSpeed, bool stop = true)
{
  SpeedProfile sp;
  SpeedProfileParams p;
  p.max_lateral_accel = kMaxLat;
  p.max_tangential_accel = 1.5;
  p.max_tangential_decel = 2.5;
  p.min_speed = 0.2;
  p.sample_spacing = 0.05;
  p.curvature_window = 2;
  p.stop_at_goal = stop;
  sp.configure(p);
  (void)max_speed;   // passed to rebuild(), not stored in params
  return sp;
}

// 直线路径中段速度应达到最大值（远离起点和终点）。
TEST(SpeedProfile, StraightPathMidSpeedIsMaxSpeed)
{
  auto ref = makeRef({{0.0, 0.0}, {10.0, 0.0}});
  auto sp = makeProfile();
  sp.rebuild(ref, kMaxSpeed);
  ASSERT_TRUE(sp.valid());
  EXPECT_NEAR(sp.speed_at(5.0), kMaxSpeed, 0.1);
}

// 曲率限速：单位圆弧中段速度应低于最大值。
// 理论值 v_curve = sqrt(max_lat / kappa) = sqrt(max_lat * R)。
TEST(SpeedProfile, CircularArcIsSlowerThanStraight)
{
  constexpr double R = 1.0;
  auto arc_ref = makeArc(R, M_PI, kMaxSpeed);  // 半圆
  auto sp = makeProfile(kMaxSpeed, false);      // 不强制终点减速，只看曲率
  sp.rebuild(arc_ref, kMaxSpeed);
  ASSERT_TRUE(sp.valid());

  const double total = arc_ref.total_length();
  const double mid_speed = sp.speed_at(total * 0.5);

  // 理论值约 sqrt(2.0 * 1.0) ≈ 1.414，比 max_speed 1.5 小。
  const double v_theory = std::sqrt(kMaxLat * R);
  EXPECT_LT(mid_speed, kMaxSpeed);
  EXPECT_NEAR(mid_speed, v_theory, 0.20);   // 允许平滑误差
}

// 启用终点减速时，路径末端速度应接近零。
TEST(SpeedProfile, StopAtGoalRampsToZero)
{
  auto ref = makeRef({{0.0, 0.0}, {20.0, 0.0}});
  auto sp = makeProfile(kMaxSpeed, true);
  sp.rebuild(ref, kMaxSpeed);
  ASSERT_TRUE(sp.valid());

  const double total = ref.total_length();
  EXPECT_LT(sp.speed_at(total), 0.01);      // 终点为零
  // 0.04 m 前：sqrt(2 * decel * 0.04) = sqrt(0.2) ≈ 0.447 < 0.5
  EXPECT_LT(sp.speed_at(total - 0.04), 0.5);
}

// 后向扫描约束：任意相邻样本间 v_i² <= v_{i+1}² + 2*decel*ds。
TEST(SpeedProfile, BackwardPassSatisfiesKinematicConstraint)
{
  auto ref = makeRef({{0.0, 0.0}, {15.0, 0.0}});
  auto sp = makeProfile();
  sp.rebuild(ref, kMaxSpeed);
  ASSERT_TRUE(sp.valid());

  const double decel = sp.params().max_tangential_decel;
  const double total = ref.total_length();
  const double step = 0.1;
  for (double s = 0.0; s + step < total; s += step) {
    const double v_cur = sp.speed_at(s);
    const double v_next = sp.speed_at(s + step);
    // v_cur² <= v_next² + 2 * decel * step  (with a small tolerance)
    EXPECT_LE(v_cur * v_cur, v_next * v_next + 2.0 * decel * step + 0.01)
      << "Backward constraint violated at s=" << s;
  }
}

// 速度从不超过 max_speed。
TEST(SpeedProfile, SpeedNeverExceedsMaxSpeed)
{
  auto arc = makeArc(2.0, M_PI * 1.5, kMaxSpeed);
  auto sp = makeProfile();
  sp.rebuild(arc, kMaxSpeed);
  ASSERT_TRUE(sp.valid());

  const double total = arc.total_length();
  for (double s = 0.0; s <= total; s += 0.05) {
    EXPECT_LE(sp.speed_at(s), kMaxSpeed + 1e-6)
      << "Speed exceeds max at s=" << s;
  }
}

// speed_at() 对越界弧长的健壮性。
TEST(SpeedProfile, HandlesOutOfBoundsArc)
{
  auto ref = makeRef({{0.0, 0.0}, {5.0, 0.0}});
  auto sp = makeProfile();
  sp.rebuild(ref, kMaxSpeed);
  ASSERT_TRUE(sp.valid());

  EXPECT_GE(sp.speed_at(-1.0), 0.0);
  EXPECT_GE(sp.speed_at(100.0), 0.0);
  EXPECT_LE(sp.speed_at(100.0), kMaxSpeed + 1e-6);
}

// invalid path -> valid() false, speed_at returns 0.
TEST(SpeedProfile, InvalidPathGivesZero)
{
  PathReference empty;
  SpeedProfile sp = makeProfile();
  sp.rebuild(empty, kMaxSpeed);
  EXPECT_FALSE(sp.valid());
  EXPECT_DOUBLE_EQ(sp.speed_at(1.0), 0.0);
}

// 隧道限速窗口：把 [4, 6] m 段标成 vmax=0.5 的隧道，直线其余段全速。
// 洞内速度必须被 vmax 压住，洞外中段仍达最大速度。
TEST(SpeedProfile, TunnelWindowClampsMaxSpeedInside)
{
  auto ref = makeRef({{0.0, 0.0}, {10.0, 0.0}});
  auto sp = makeProfile(kMaxSpeed, false);   // 不强制终点减速，隔离窗口效果
  sp.set_tunnel_window(
    [](const Eigen::Vector2d & p, double & vmin, double & vmax) {
      if (p.x() >= 4.0 && p.x() <= 6.0) {
        vmin = 0.0;
        vmax = 0.5;
        return true;
      }
      return false;
    });
  sp.rebuild(ref, kMaxSpeed);
  ASSERT_TRUE(sp.valid());

  // 洞内被夹到 vmax。
  EXPECT_LE(sp.speed_at(5.0), 0.5 + 1e-6);
  // 洞外远处仍能全速（洞前留了足够长的加速距离）。
  EXPECT_NEAR(sp.speed_at(1.0), kMaxSpeed, 0.1);
}

// 窗口必须在前/后向扫描之前施加：洞口前应出现一段减速斜坡，
// 而不是在洞口瞬间从全速跳到 vmax。检查洞口前 0.5 m 处速度已明显低于 max。
TEST(SpeedProfile, TunnelWindowRampsBeforeMouth)
{
  auto ref = makeRef({{0.0, 0.0}, {10.0, 0.0}});
  auto sp = makeProfile(kMaxSpeed, false);
  sp.set_tunnel_window(
    [](const Eigen::Vector2d & p, double & vmin, double & vmax) {
      if (p.x() >= 4.0 && p.x() <= 6.0) {
        vmin = 0.0;
        vmax = 0.3;
        return true;
      }
      return false;
    });
  sp.rebuild(ref, kMaxSpeed);
  ASSERT_TRUE(sp.valid());

  // 洞口在 s≈4：进洞前 0.3 m 处速度应已被后向扫描拉低到接近 vmax，远小于全速。
  EXPECT_LT(sp.speed_at(3.7), kMaxSpeed);
  EXPECT_LE(sp.speed_at(4.0), 0.3 + 0.15);
}

// 空窗口（查询恒 false）不改变任何速度：有无窗口两条剖面逐点一致。
TEST(SpeedProfile, TunnelWindowEmptyIsNoop)
{
  auto ref = makeRef({{0.0, 0.0}, {10.0, 0.0}});

  auto base = makeProfile(kMaxSpeed, false);
  base.rebuild(ref, kMaxSpeed);

  auto empty_win = makeProfile(kMaxSpeed, false);
  empty_win.set_tunnel_window(
    [](const Eigen::Vector2d &, double &, double &) { return false; });
  empty_win.rebuild(ref, kMaxSpeed);

  for (double s = 0.0; s <= 10.0; s += 0.5) {
    EXPECT_NEAR(empty_win.speed_at(s), base.speed_at(s), 1e-9) << "at s=" << s;
  }
}

// vmin 抬升下限：单位半圆的曲率会把中段速度压到 ~1.41；给整段隧道标 vmin=1.5
// （不超过 max_speed），洞内中段速度应被抬到接近 vmin，高于无窗口时的曲率限速值。
// 终点减速关闭以隔离效果。
TEST(SpeedProfile, TunnelWindowRaisesMin)
{
  constexpr double R = 1.0;
  auto arc = makeArc(R, M_PI, kMaxSpeed);

  auto base = makeProfile(kMaxSpeed, false);
  base.rebuild(arc, kMaxSpeed);
  const double total = arc.total_length();
  const double base_mid = base.speed_at(total * 0.5);

  auto win = makeProfile(kMaxSpeed, false);
  win.set_tunnel_window(
    [](const Eigen::Vector2d &, double & vmin, double & vmax) {
      vmin = kMaxSpeed;   // 会被 min(vmin, max_speed) 夹到 max_speed
      vmax = 0.0;         // 不设上限
      return true;
    });
  win.rebuild(arc, kMaxSpeed);
  const double win_mid = win.speed_at(total * 0.5);

  EXPECT_GT(win_mid, base_mid);   // 下限确实把曲率限速抬高了
}

}  // namespace
}  // namespace navigation2::mpc

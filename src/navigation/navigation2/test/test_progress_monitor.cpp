#include "mpc/progress_monitor.hpp"

#include <gtest/gtest.h>

namespace navigation2::mpc
{
namespace
{

ProgressMonitorParams defaultParams()
{
  ProgressMonitorParams params;
  params.min_displacement = 0.15;
  params.no_progress_timeout = 2.0;
  params.stuck_timeout = 1.5;
  params.cmd_epsilon = 0.05;
  return params;
}

TEST(ProgressMonitor, NormalDrivingNeverFlags)
{
  ProgressMonitor monitor;
  monitor.configure(defaultParams());

  // 1.5 m/s，每拍 0.1 s 走 0.15 m：锚点持续被刷新。
  for (int i = 0; i <= 100; ++i) {
    monitor.update(Eigen::Vector2d(0.15 * i, 0.0), 1.5, 0.1);
    EXPECT_FALSE(monitor.noProgress()) << "tick " << i;
    EXPECT_FALSE(monitor.stuck()) << "tick " << i;
  }
}

TEST(ProgressMonitor, FlagsNoProgressWhenStationaryWithoutCommand)
{
  ProgressMonitor monitor;
  monitor.configure(defaultParams());

  const Eigen::Vector2d pos(1.0, 1.0);
  monitor.update(pos, 0.0, 0.1);  // 首拍建立锚点

  // 停在原地、且没有下发指令：应报无进展，但不应报卡住。
  for (int i = 0; i < 19; ++i) {
    monitor.update(pos, 0.0, 0.1);
  }
  EXPECT_FALSE(monitor.noProgress()) << "1.9 s 时还不该超时";

  monitor.update(pos, 0.0, 0.1);
  EXPECT_TRUE(monitor.noProgress());
  EXPECT_FALSE(monitor.stuck()) << "没有指令就不算卡住";
}

TEST(ProgressMonitor, FlagsStuckOnlyWhenCommanding)
{
  ProgressMonitor monitor;
  monitor.configure(defaultParams());

  const Eigen::Vector2d pos(2.0, -1.0);
  monitor.update(pos, 1.0, 0.1);  // 首拍建立锚点

  for (int i = 0; i < 14; ++i) {
    monitor.update(pos, 1.0, 0.1);
  }
  EXPECT_FALSE(monitor.stuck()) << "1.4 s 时还不该判卡住";

  monitor.update(pos, 1.0, 0.1);
  EXPECT_TRUE(monitor.stuck()) << "有指令、无位移，1.5 s 后判卡住";
}

TEST(ProgressMonitor, CommandDroppingToZeroClearsStuck)
{
  ProgressMonitor monitor;
  monitor.configure(defaultParams());

  const Eigen::Vector2d pos(0.0, 0.0);
  for (int i = 0; i < 20; ++i) {
    monitor.update(pos, 1.0, 0.1);
  }
  ASSERT_TRUE(monitor.stuck());

  // 指令归零后，卡住这一路清零；无进展仍然成立（位移确实没增长）。
  monitor.update(pos, 0.0, 0.1);
  EXPECT_FALSE(monitor.stuck());
  EXPECT_DOUBLE_EQ(monitor.commandedStagnantTime(), 0.0);
  EXPECT_TRUE(monitor.noProgress());
}

TEST(ProgressMonitor, SpeedBelowEpsilonDoesNotCountAsCommanding)
{
  ProgressMonitor monitor;
  monitor.configure(defaultParams());

  const Eigen::Vector2d pos(0.0, 0.0);
  // 0.02 m/s < cmd_epsilon(0.05)：视为没有下发指令。
  for (int i = 0; i < 30; ++i) {
    monitor.update(pos, 0.02, 0.1);
  }
  EXPECT_FALSE(monitor.stuck());
  EXPECT_TRUE(monitor.noProgress());
}

TEST(ProgressMonitor, RealMotionClearsBothTimers)
{
  ProgressMonitor monitor;
  monitor.configure(defaultParams());

  const Eigen::Vector2d stalled(0.0, 0.0);
  for (int i = 0; i < 18; ++i) {
    monitor.update(stalled, 1.0, 0.1);
  }
  ASSERT_TRUE(monitor.stuck());
  ASSERT_GT(monitor.stagnantTime(), 1.0);

  // 一次足够大的位移就把两路计时器都清掉。
  monitor.update(Eigen::Vector2d(0.2, 0.0), 1.0, 0.1);
  EXPECT_FALSE(monitor.stuck());
  EXPECT_FALSE(monitor.noProgress());
  EXPECT_DOUBLE_EQ(monitor.stagnantTime(), 0.0);
  EXPECT_DOUBLE_EQ(monitor.commandedStagnantTime(), 0.0);
}

TEST(ProgressMonitor, ResetClearsState)
{
  ProgressMonitor monitor;
  monitor.configure(defaultParams());

  const Eigen::Vector2d pos(5.0, 5.0);
  for (int i = 0; i < 25; ++i) {
    monitor.update(pos, 1.0, 0.1);
  }
  ASSERT_TRUE(monitor.noProgress());
  ASSERT_TRUE(monitor.stuck());

  monitor.reset();
  EXPECT_FALSE(monitor.noProgress());
  EXPECT_FALSE(monitor.stuck());
  EXPECT_DOUBLE_EQ(monitor.stagnantTime(), 0.0);

  // reset 后首拍只重建锚点，不该立刻又报。
  monitor.update(pos, 1.0, 0.1);
  EXPECT_FALSE(monitor.noProgress());
  EXPECT_FALSE(monitor.stuck());
}

TEST(ProgressMonitor, IgnoresNonFiniteAndNonPositiveDt)
{
  ProgressMonitor monitor;
  monitor.configure(defaultParams());

  const Eigen::Vector2d pos(0.0, 0.0);
  monitor.update(pos, 1.0, 0.1);

  // dt <= 0 不该推进计时器（时钟跳变时会出现）。
  for (int i = 0; i < 100; ++i) {
    monitor.update(pos, 1.0, 0.0);
  }
  EXPECT_DOUBLE_EQ(monitor.stagnantTime(), 0.0);
  EXPECT_FALSE(monitor.stuck());

  // NaN 位置直接忽略，不污染锚点。
  monitor.update(Eigen::Vector2d(std::nan(""), 0.0), 1.0, 0.1);
  EXPECT_DOUBLE_EQ(monitor.stagnantTime(), 0.0);
}

}  // namespace
}  // namespace navigation2::mpc

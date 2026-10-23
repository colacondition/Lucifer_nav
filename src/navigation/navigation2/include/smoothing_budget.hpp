#ifndef NAVIGATION2__SMOOTHING_BUDGET_HPP_
#define NAVIGATION2__SMOOTHING_BUDGET_HPP_

// 单次平滑调用的墙钟预算（header-only，无 ROS 依赖）。
//
// 为什么需要它：MINCO 平滑回调跑在 8 Hz（125 ms 周期）的重规划链路里，而一次
// 优化 + 有界修复的耗时随路径复杂度浮动 —— 修复循环每多一次尝试就多一次完整的
// L-BFGS，最坏情况（两次修复 + 两阶段都触发）能把单拍推到远超 125 ms，表现为
// 回调排队、/plan 延迟抖动，下游 MPC 拿到过期参考。预算给这条链路加一个墙钟
// 上限：超时就放弃剩余修复、保留当前可用结果或回退原始路径，把回调周期还回去。
//
// 设计纪律：
// - 时间源用 std::chrono::steady_clock（单调时钟），不受 ROS Time/仿真时间影响。
//   预算度量的是**实际 CPU 墙钟**，不是仿真时间 —— 8 Hz 回调的截止期也是墙钟。
// - 无 ROS 依赖、无锁、无线程：调用方（平滑节点）在 MutuallyExclusive 回调组里
//   串行使用，start()/expired() 只是读一个时间点。
// - budget_ms <= 0（含 NaN/Inf）表示关闭。关闭时 enabled() 恒 false、expired()
//   恒 false，所有检查退化为一次 double 比较，与不接预算的旧行为完全一致。
//
// 典型用法：
//   navigation2::SmoothingBudget budget(80.0);
//   budget.start();
//   ... 优化 / 修复 ...
//   if (budget.expired()) { /* 停止剩余重试 */ }
//
// 注意语义：预算从 start() 时刻开始算，与 perf_monitor 的 tick() 起点相互独立
// （后者可能包含回调内的其他工作）。本类只回答「从 start() 到现在是否超时」。

#include <chrono>
#include <cmath>

namespace navigation2
{

class SmoothingBudget
{
public:
  SmoothingBudget() = default;

  // budget_ms <= 0（或非有限值）时预算关闭。
  explicit SmoothingBudget(double budget_ms)
  : budget_ms_(normalize(budget_ms)) {}

  // 开始计时。每次进入一轮平滑（去重之后、优化/修复之前）调用一次。
  void start()
  {
    start_ = Clock::now();
    started_ = true;
  }

  bool enabled() const { return budget_ms_ > 0.0; }

  // 预算关闭时恒 false。elapsedMs() >= budget_ms 即视为超时。
  bool expired() const
  {
    if (!enabled()) {
      return false;
    }
    return elapsedMs() >= budget_ms_;
  }

  // 自 start() 起的墙钟毫秒数；未 start() 时为 0。下限钳到 0，避免时间源
  // 精度/回绕导致的负值把调用方的比较逻辑带偏。
  double elapsedMs() const
  {
    if (!started_) {
      return 0.0;
    }
    const double ms =
      std::chrono::duration<double, std::milli>(Clock::now() - start_).count();
    return ms > 0.0 ? ms : 0.0;
  }

  // 运行期改预算（例如参数热更新）。<= 0 恢复为关闭；不重置 start() 时间点，
  // 因此从「关闭」切到「开启」时，计时从本拍 start() 起算。
  void setBudgetMs(double budget_ms) { budget_ms_ = normalize(budget_ms); }

private:
  using Clock = std::chrono::steady_clock;

  static double normalize(double budget_ms)
  {
    return (std::isfinite(budget_ms) && budget_ms > 0.0) ? budget_ms : 0.0;
  }

  double budget_ms_{0.0};
  Clock::time_point start_{};
  bool started_{false};
};

}  // namespace navigation2

#endif  // NAVIGATION2__SMOOTHING_BUDGET_HPP_

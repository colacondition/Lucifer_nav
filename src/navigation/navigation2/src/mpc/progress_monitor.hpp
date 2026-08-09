#pragma once
// 失效检测：无进展与卡住。
//
// 这两件事是分开的判据，不能合并：
//   - noProgress：不管有没有下发指令，位移长时间不增长就算无进展。它捕捉的
//     是「MPC 自己一直在停车」这类循环（安全检查反复否决 → publishStop →
//     请求重规划 → 新路径仍不可跟随），此时速度指令本来就是零。
//   - stuck：有速度指令但位移不增长。它捕捉的是「发着速度车没动」——顶住
//     障碍、打滑、底盘不响应。没有指令时不该算卡住，所以这一路的计时器在
//     指令为零时清零。
//
// 判据基于**世界系位移**而不是弧长里程碑：A 的全局规划器每移动
// min_replan_distance(0.2 m) 就重规划一次，弧长坐标系原点随之改变，任何
// 基于弧长的累计计时都会被反复清零、永远到不了超时阈值。世界系位移与
// 重规划无关。
#include <Eigen/Dense>

namespace navigation2::mpc {

struct ProgressMonitorParams
{
  // 认为「确实动了」所需的最小位移（m）。
  double min_displacement = 0.15;
  // 位移不增长多久算无进展（s）。
  double no_progress_timeout = 2.0;
  // 有指令但位移不增长多久算卡住（s）。
  double stuck_timeout = 1.5;
  // 速率超过此值才算「正在下发指令」（m/s）。
  double cmd_epsilon = 0.05;
};

class ProgressMonitor
{
public:
  void configure(const ProgressMonitorParams & params) { params_ = params; }

  const ProgressMonitorParams & params() const noexcept { return params_; }

  // 进入恢复、换路径或到达目标时调用，清空计时器与锚点。
  void reset() noexcept;

  // 推进一拍。commanded_speed 是本拍实际下发的速率（m/s）。
  void update(const Eigen::Vector2d & pos, double commanded_speed, double dt);

  bool noProgress() const noexcept { return no_progress_; }

  bool stuck() const noexcept { return stuck_; }

  // 自上次确认位移以来的停滞时长（s），供日志与诊断。
  double stagnantTime() const noexcept { return stagnant_time_; }

  // 有指令但未位移的累计时长（s）。
  double commandedStagnantTime() const noexcept { return commanded_stagnant_time_; }

private:
  ProgressMonitorParams params_;

  Eigen::Vector2d anchor_{Eigen::Vector2d::Zero()};
  bool has_anchor_{false};
  double stagnant_time_{0.0};
  double commanded_stagnant_time_{0.0};
  bool no_progress_{false};
  bool stuck_{false};
};

}  // namespace navigation2::mpc

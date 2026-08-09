#pragma once
// 有向路径上的多假设弧长进度跟踪。
//
// 单帧最近点投影有两个已知缺陷：路径回绕时最近点会在两个弧长相差很远、
// 但空间距离接近的位置之间跳变；以及进度可以回退。台阶/里程碑一类基于
// 弧长的判定一旦拿到抖动的进度就会误触发。
//
// 这里把进度建模为有向路径上的时序状态 (s, s_dot)，用多帧观测推断：
//   - 维护多个竞争的弧长假设，每帧按似然加权、淘汰不合理分支；
//   - 似然同时看位置残差和「速度方向与路径切向的一致性」，后者是单帧
//     最近点无法区分「沿路径前进」还是「倒退」的关键；
//   - 对外报告的进度单调不减，即使内部最优假设发生切换。
//
// 速度由内部对连续位置做有限差分 + EMA 估计，不依赖外部里程计 twist 的
// 坐标系约定（ROS 里 Odometry::twist 是 child_frame 系，各 LIO 实现不一）。
#include <cstddef>
#include <vector>

#include <Eigen/Dense>

#include "mpc/path_reference.hpp"

namespace navigation2::mpc {

struct RouteTrackerParams
{
  // 初始假设沿路径的播撒间距（m）。
  double hypothesis_spacing = 1.0;
  // 每个假设每帧在预测弧长附近的精修半径（m）。
  double search_window = 1.5;
  // 精修时的粗扫步长（m）。
  double refine_step = 0.05;
  // 保留的假设上限。
  std::size_t max_hypotheses = 5;
  // 位置残差的似然尺度（m）。
  double position_sigma = 0.35;
  // 方向一致性在似然里的权重，0 表示完全不看方向。
  double direction_weight = 1.5;
  // 认为速度方向可信的最小速率（m/s），低于此值方向项取中性。
  double min_speed_for_direction = 0.15;
  // 归一化权重低于此值的假设被淘汰。
  double weight_floor = 0.02;
  // 弧长相距小于此值的假设合并（m）。
  double merge_distance = 0.25;
  // 播撒时偏向小弧长的先验尺度（m）。全局规划器总是从当前位姿起规划，
  // 所以 s≈0 才是正常起点；没有这个先验，U 形路径上「起点与终点空间接近」
  // 会让首帧（速度为零、方向项中性）有可能选中终点附近的假设。
  double seed_prior_length = 3.0;
  // 位置残差超过此值算跟踪异常（m）。
  double max_track_error = 0.5;
  // 残差持续超限多久才判定跟踪丢失（s）。
  double lost_grace_time = 0.5;
  // s_dot 的 EMA 系数，越小越平滑。
  double arc_rate_alpha = 0.3;
  // 速度估计的 EMA 系数。
  double velocity_alpha = 0.4;
};

class RouteTracker
{
public:
  void configure(const RouteTrackerParams & params) { params_ = params; }

  const RouteTrackerParams & params() const noexcept { return params_; }

  // 完全清零，包含速度估计。目标切换、失去路径时用。
  void reset() noexcept;

  // 路径被换掉时调用：规划器总是从当前位姿重新规划，弧长坐标系原点跟着移动，
  // 所以假设与已报告进度必须清零。但速度估计要保留 —— 它是靠连续两帧位置
  // 差分得来的，而 A 的规划器每前进 min_replan_distance(0.2 m) 就换一次路径，
  // 若跟着清零，方向似然项会有很大比例的时间处于「刚复位、尚未估出速度」的
  // 中性状态，多假设就退化成了纯位置最近点。
  void on_path_replaced() noexcept;

  // 推进一拍。dt <= 0 时只做播撒不积分。
  void update(const PathReference & ref, const Eigen::Vector2d & pos, double dt);

  // 是否已经有有效假设。
  bool valid() const noexcept { return !hypotheses_.empty(); }

  // 对外报告的进度，单调不减。
  double arc_length() const noexcept { return reported_s_; }

  // 当前最优假设的原始弧长（可回退，仅供诊断）。
  double best_arc_length() const noexcept { return best_s_; }

  // 最优假设的弧长变化率估计。
  double arc_rate() const noexcept { return best_s_dot_; }

  // 最优假设的位置残差。
  double residual() const noexcept { return best_residual_; }

  // 残差持续超限则判定跟踪丢失。
  bool lost() const noexcept { return lost_; }

  // 内部估计的世界系速度（仅供诊断）。
  const Eigen::Vector2d & velocity() const noexcept { return velocity_; }

  std::size_t hypothesis_count() const noexcept { return hypotheses_.size(); }

private:
  struct Hypothesis
  {
    double s{0.0};
    double s_dot{0.0};
    double weight{1.0};
    double residual{0.0};
    // 刚播撒、还没经历过一次精修的假设。它的 s 是任意播撒点，
    // 首帧不能用来算弧长变化率。
    bool fresh{true};
  };

  void seed(const PathReference & ref, const Eigen::Vector2d & pos);
  // 在 [center-window, center+window] 内找位置残差最小的弧长。
  double refine(
    const PathReference & ref, const Eigen::Vector2d & pos, double center,
    double & residual_out) const;
  void prune();

  RouteTrackerParams params_;
  std::vector<Hypothesis> hypotheses_;

  Eigen::Vector2d velocity_{Eigen::Vector2d::Zero()};
  Eigen::Vector2d last_pos_{Eigen::Vector2d::Zero()};
  bool has_last_pos_{false};

  double reported_s_{0.0};
  double best_s_{0.0};
  double best_s_dot_{0.0};
  double best_residual_{0.0};
  double over_error_time_{0.0};
  bool lost_{false};
};

}  // namespace navigation2::mpc

#pragma once
// 弧长域速度剖面：按曲率限侧向加速度，前/后向扫描保证加减速可达。
//
// 替代原来的 expected_speed 常数。常数速度让 MPC 在弯道处的参考轨迹不可跟踪：
// 过弯不减速 → 参考点跑到弯道外 → MPC 必须同时纠偏和降速 → 超调或切角。
// 速度剖面把「弯道减速」提前编码进参考，让 MPC 只负责跟踪。
//
// 算法三步：
//   1. 曲率限速：v_curve(s) = sqrt(max_lateral_accel / kappa(s))
//   2. 后向扫描：从终点倒推，保证在每处的速度不超过「以 max_tangential_decel
//      减速到下一处」所允许的最大值 ─ 让车能真正停在终点。
//   3. 前向扫描：从起点正推，限制加速度 max_tangential_accel。
#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Dense>

#include "mpc/path_reference.hpp"

namespace navigation2::mpc {

struct SpeedProfileParams
{
  // 侧向加速度上限（m/s²）。主要调参项 —— 值越小，弯道减速越早/越多。
  double max_lateral_accel = 2.0;
  // 切向加速度上限（m/s²）。
  double max_tangential_accel = 1.5;
  // 切向减速度上限（m/s²）。
  double max_tangential_decel = 2.5;
  // 速度下限（m/s）。防止极小曲率半径时速度降到零、参考轨迹近似停止。
  double min_speed = 0.3;
  // 采样间距（m）。越小精度越高，但重建代价也越高。
  double sample_spacing = 0.10;
  // 曲率平滑窗（样本数，双侧）。消除多段折线折点处的人工曲率尖峰。
  int curvature_window = 3;
  // 终点速度是否强制为零。
  bool stop_at_goal = true;
};

class SpeedProfile
{
public:
  void configure(const SpeedProfileParams & p) { params_ = p; }

  const SpeedProfileParams & params() const noexcept { return params_; }

  // 路径变化时调用。PathReference 的几何方法已是 public，可以直接使用。
  void rebuild(const PathReference & ref, double max_speed)
  {
    samples_.clear();
    speeds_.clear();

    if (!ref.valid()) {
      return;
    }

    const double total = ref.total_length();
    const double sp = std::max(params_.sample_spacing, 1e-4);
    const int n = std::max(2, static_cast<int>(std::ceil(total / sp)) + 1);

    samples_.resize(n);
    speeds_.resize(n);

    for (int i = 0; i < n; ++i) {
      samples_[i] = std::min(static_cast<double>(i) * sp, total);
    }
    samples_.back() = total;

    // 步骤 1：曲率估计 + 侧向加速度限速。
    const auto kappas = computeCurvatures(ref);
    for (int i = 0; i < n; ++i) {
      const double kappa = kappas[i];
      const double v_curve = (kappa > 1e-6)
        ? std::sqrt(std::max(params_.max_lateral_accel / kappa, 0.0))
        : max_speed;
      speeds_[i] = std::clamp(v_curve, params_.min_speed, max_speed);
    }

    // 步骤 2：终点强制为零（如果启用），再后向扫描保证减速可达。
    if (params_.stop_at_goal) {
      speeds_.back() = 0.0;
    }
    const double decel = std::max(params_.max_tangential_decel, 1e-4);
    for (int i = n - 2; i >= 0; --i) {
      const double ds = samples_[i + 1] - samples_[i];
      const double v_next = speeds_[i + 1];
      const double v_limit = std::sqrt(v_next * v_next + 2.0 * decel * ds);
      speeds_[i] = std::min(speeds_[i], v_limit);
    }

    // 步骤 3：前向扫描限制加速度。
    const double accel = std::max(params_.max_tangential_accel, 1e-4);
    for (int i = 1; i < n; ++i) {
      const double ds = samples_[i] - samples_[i - 1];
      const double v_prev = speeds_[i - 1];
      const double v_limit = std::sqrt(v_prev * v_prev + 2.0 * accel * ds);
      speeds_[i] = std::min(speeds_[i], v_limit);
    }
  }

  bool valid() const noexcept { return !speeds_.empty(); }

  // 按弧长线性插值速度。
  double speed_at(double s) const noexcept
  {
    if (speeds_.empty()) {
      return 0.0;
    }
    if (s <= samples_.front()) {
      return speeds_.front();
    }
    if (s >= samples_.back()) {
      return speeds_.back();
    }
    const auto it = std::lower_bound(samples_.begin(), samples_.end(), s);
    const int hi = std::clamp<int>(
      static_cast<int>(std::distance(samples_.begin(), it)), 1,
      static_cast<int>(samples_.size()) - 1);
    const int lo = hi - 1;
    const double ds = samples_[hi] - samples_[lo];
    if (ds < 1e-6) {
      return speeds_[hi];
    }
    const double alpha = (s - samples_[lo]) / ds;
    return speeds_[lo] * (1.0 - alpha) + speeds_[hi] * alpha;
  }

private:
  std::vector<double> computeCurvatures(const PathReference & ref) const
  {
    const int n = static_cast<int>(samples_.size());
    std::vector<double> raw(n, 0.0);
    const double sp = std::max(params_.sample_spacing, 1e-4);
    const double total = ref.total_length();

    for (int i = 1; i < n - 1; ++i) {
      const double s = samples_[i];
      const double s_prev = std::max(0.0, s - sp);
      const double s_next = std::min(total, s + sp);
      const Eigen::Vector2d d0 = ref.pos_by_arc(s) - ref.pos_by_arc(s_prev);
      const Eigen::Vector2d d1 = ref.pos_by_arc(s_next) - ref.pos_by_arc(s);
      const double len0 = d0.norm();
      const double len1 = d1.norm();
      if (len0 < 1e-6 || len1 < 1e-6) {
        continue;
      }
      const double cross_val = d0.x() * d1.y() - d0.y() * d1.x();
      const double dot_val = d0.dot(d1);
      const double angle = std::atan2(std::abs(cross_val), dot_val);
      const double arc = 0.5 * (len0 + len1);
      if (arc > 1e-6) {
        raw[i] = angle / arc;
      }
    }

    // 窗口均值平滑：消除折线折点处的人工尖峰。
    const int w = std::max(1, params_.curvature_window);
    std::vector<double> smooth(n, 0.0);
    for (int i = 0; i < n; ++i) {
      int cnt = 0;
      double sum = 0.0;
      for (int j = i - w; j <= i + w; ++j) {
        if (j >= 0 && j < n) {
          sum += raw[j];
          ++cnt;
        }
      }
      smooth[i] = (cnt > 0) ? sum / cnt : 0.0;
    }
    return smooth;
  }

  SpeedProfileParams params_;
  std::vector<double> samples_;
  std::vector<double> speeds_;
};

}  // namespace navigation2::mpc

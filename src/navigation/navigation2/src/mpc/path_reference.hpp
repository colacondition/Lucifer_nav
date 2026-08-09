#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <nav_msgs/msg/path.hpp>

namespace navigation2::mpc {

class PathReference
{
public:
  // 用路径生成参考序列。
  void set_path(const nav_msgs::msg::Path & path, double expected_speed)
  {
    pts_.clear();
    s_.clear();
    speed_ = std::max(expected_speed, 1e-3);
    pts_.reserve(path.poses.size());
    for (const auto & ps : path.poses) {
      pts_.emplace_back(ps.pose.position.x, ps.pose.position.y);
    }
    s_.resize(pts_.size(), 0.0);
    for (size_t i = 1; i < pts_.size(); ++i) {
      s_[i] = s_[i - 1] + (pts_[i] - pts_[i - 1]).norm();
    }
  }

  bool valid() const noexcept { return pts_.size() >= 2 && s_.back() > 1e-3; }

  double total_duration() const noexcept
  {
    return valid() ? s_.back() / speed_ : 0.0;
  }

  // 时间映射到弧长后取插值位置。
  Eigen::Vector2d pos_by_time(double t) const noexcept
  {
    return pos_by_arc(t * speed_);
  }

  // 速度前馈，末端直接置零。
  Eigen::Vector2d vel_by_time(double t) const noexcept
  {
    const double sq = t * speed_;
    if (!valid() || sq >= s_.back()) {
      return Eigen::Vector2d::Zero();
    }
    return tangent_by_arc(sq) * speed_;
  }

  // 找最近的参考时刻和偏差。
  std::pair<double, double> time_by_pos(const Eigen::Vector2d & pos, double sample_dt = 0.05)
    const noexcept
  {
    double best_t = 0.0;
    double min_dis = std::numeric_limits<double>::infinity();
    const double dur = total_duration();
    for (double t = 0.0; t <= dur; t += sample_dt) {
      double d = (pos_by_time(t) - pos).norm();
      if (d < min_dis) {
        min_dis = d;
        best_t = t;
      }
    }
    return {best_t, min_dis};
  }

  Eigen::Vector2d goal() const noexcept
  {
    return pts_.empty() ? Eigen::Vector2d::Zero() : pts_.back();
  }

  // 路径总弧长，弧长域采样和进度跟踪都以它为上界。
  double total_length() const noexcept
  {
    return s_.empty() ? 0.0 : s_.back();
  }

  double expected_speed() const noexcept { return speed_; }

  // 按弧长取插值位置（RouteTracker 复用，避免重复实现插值）。
  Eigen::Vector2d pos_by_arc(double sq) const noexcept
  {
    if (!valid() || sq <= 0.0) {
      return pts_.empty() ? Eigen::Vector2d::Zero() : pts_.front();
    }
    if (sq >= s_.back()) {
      return pts_.back();
    }
    auto it = std::lower_bound(s_.begin(), s_.end(), sq);
    int i = std::clamp<int>(std::distance(s_.begin(), it), 1, static_cast<int>(s_.size()) - 1);
    const double ds = s_[i] - s_[i - 1];
    if (ds < 1e-6) {
      return pts_[i];
    }
    const double a = (sq - s_[i - 1]) / ds;
    return pts_[i - 1] * (1.0 - a) + pts_[i] * a;
  }

  Eigen::Vector2d tangent_by_arc(double sq) const noexcept
  {
    auto it = std::lower_bound(s_.begin(), s_.end(), sq);
    int i = std::clamp<int>(std::distance(s_.begin(), it), 1, static_cast<int>(s_.size()) - 1);
    Eigen::Vector2d d = pts_[i] - pts_[i - 1];
    const double n = d.norm();
    if (n > 1e-6) {
      return d / n;
    }
    return Eigen::Vector2d::Zero();
  }

private:
  std::vector<Eigen::Vector2d> pts_;
  std::vector<double> s_;
  double speed_ = 1.0;
};

}  // namespace navigation2::mpc

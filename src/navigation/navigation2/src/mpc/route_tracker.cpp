#include "mpc/route_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace navigation2::mpc {

namespace {

constexpr double kEps = 1e-9;

}  // namespace

void RouteTracker::reset() noexcept
{
  hypotheses_.clear();
  velocity_.setZero();
  last_pos_.setZero();
  has_last_pos_ = false;
  reported_s_ = 0.0;
  best_s_ = 0.0;
  best_s_dot_ = 0.0;
  best_residual_ = 0.0;
  over_error_time_ = 0.0;
  lost_ = false;
}

void RouteTracker::on_path_replaced() noexcept
{
  // 路径换了，弧长坐标系的原点也换了，假设与报告进度都必须重来。
  // 但速度估计与上一帧位置是世界系量，与路径无关，保留下来可以让方向似然
  // 在换路后的第一帧就可用 —— A 的全局规划器每移动 min_replan_distance
  // (0.2 m) 就重规划一次，若每次都丢掉速度，方向项几乎永远处于「刚复位、
  // 还差一帧」的状态。
  hypotheses_.clear();
  reported_s_ = 0.0;
  best_s_ = 0.0;
  best_s_dot_ = 0.0;
  best_residual_ = 0.0;
  over_error_time_ = 0.0;
  lost_ = false;
}

void RouteTracker::seed(const PathReference & ref, const Eigen::Vector2d & pos)
{
  hypotheses_.clear();

  const double total = ref.total_length();
  const double spacing = std::max(params_.hypothesis_spacing, 1e-3);
  const double sigma = std::max(params_.position_sigma, 1e-3);
  const double prior_length = std::max(params_.seed_prior_length, 1e-3);

  std::vector<double> seeds;
  for (double s = 0.0; s < total - kEps; s += spacing) {
    seeds.push_back(s);
  }
  seeds.push_back(total);

  hypotheses_.reserve(seeds.size());
  for (const double s : seeds) {
    Hypothesis h;
    h.s = s;
    h.s_dot = 0.0;
    h.residual = (ref.pos_by_arc(s) - pos).norm();
    // 位置似然乘一个偏向小弧长的先验：A 的全局规划器总是从当前位姿起规划，
    // 所以 s≈0 才是正常的起点。没有这个先验时，U 形路径上「起点与终点空间
    // 接近」会让首帧（速度为零、方向项中性）有可能选中终点附近的假设。
    const double position_likelihood = std::exp(-0.5 * (h.residual * h.residual) / (sigma * sigma));
    h.weight = std::max(position_likelihood * std::exp(-s / prior_length), 1e-12);
    hypotheses_.push_back(h);
  }
}

double RouteTracker::refine(
  const PathReference & ref, const Eigen::Vector2d & pos, double center,
  double & residual_out) const
{
  const double total = ref.total_length();
  const double window = std::max(params_.search_window, 0.0);
  const double step = std::max(params_.refine_step, 1e-3);

  const double lo = std::clamp(center - window, 0.0, total);
  const double hi = std::clamp(center + window, 0.0, total);

  // 粗扫定位最小值所在的格。
  double best_s = lo;
  double best_d = std::numeric_limits<double>::infinity();
  for (double s = lo; s <= hi + kEps; s += step) {
    const double d = (ref.pos_by_arc(s) - pos).norm();
    if (d < best_d) {
      best_d = d;
      best_s = s;
    }
  }
  // 粗扫可能落不到 hi 上，单独补一次。
  {
    const double d = (ref.pos_by_arc(hi) - pos).norm();
    if (d < best_d) {
      best_d = d;
      best_s = hi;
    }
  }

  // 在最小值邻域做三分精修，避免受粗扫步长限制。
  double a = std::clamp(best_s - step, lo, hi);
  double b = std::clamp(best_s + step, lo, hi);
  for (int iteration = 0; iteration < 24 && (b - a) > 1e-4; ++iteration) {
    const double third = (b - a) / 3.0;
    const double m1 = a + third;
    const double m2 = b - third;
    if ((ref.pos_by_arc(m1) - pos).norm() <= (ref.pos_by_arc(m2) - pos).norm()) {
      b = m2;
    } else {
      a = m1;
    }
  }
  const double refined_s = 0.5 * (a + b);
  const double refined_d = (ref.pos_by_arc(refined_s) - pos).norm();
  if (refined_d < best_d) {
    best_d = refined_d;
    best_s = refined_s;
  }

  residual_out = best_d;
  return best_s;
}

void RouteTracker::prune()
{
  if (hypotheses_.empty()) {
    return;
  }

  auto normalize = [this]() {
    double sum = 0.0;
    for (const auto & h : hypotheses_) {
      sum += h.weight;
    }
    if (sum <= kEps) {
      const double uniform = 1.0 / static_cast<double>(hypotheses_.size());
      for (auto & h : hypotheses_) {
        h.weight = uniform;
      }
      return;
    }
    for (auto & h : hypotheses_) {
      h.weight /= sum;
    }
  };

  normalize();

  // 合并弧长过近的假设，避免所有分支收敛到同一处后退化成单假设。
  std::sort(
    hypotheses_.begin(), hypotheses_.end(),
    [](const Hypothesis & lhs, const Hypothesis & rhs) { return lhs.s < rhs.s; });

  const double merge_distance = std::max(params_.merge_distance, 0.0);
  std::vector<Hypothesis> merged;
  merged.reserve(hypotheses_.size());
  for (const auto & h : hypotheses_) {
    if (!merged.empty() && (h.s - merged.back().s) <= merge_distance) {
      auto & keep = merged.back();
      const double combined_weight = keep.weight + h.weight;
      // 保留权重更高那支的状态，权重相加。
      if (h.weight > keep.weight) {
        keep.s = h.s;
        keep.s_dot = h.s_dot;
        keep.residual = h.residual;
        keep.fresh = h.fresh;
      }
      keep.weight = combined_weight;
      continue;
    }
    merged.push_back(h);
  }
  hypotheses_ = std::move(merged);

  // 按权重排序后淘汰弱分支并截断。
  std::sort(
    hypotheses_.begin(), hypotheses_.end(),
    [](const Hypothesis & lhs, const Hypothesis & rhs) { return lhs.weight > rhs.weight; });

  const double floor_weight = std::clamp(params_.weight_floor, 0.0, 1.0);
  auto cut = std::find_if(
    hypotheses_.begin(), hypotheses_.end(),
    [floor_weight](const Hypothesis & h) { return h.weight < floor_weight; });
  // 至少保住最优的一支。
  if (cut == hypotheses_.begin()) {
    ++cut;
  }
  hypotheses_.erase(cut, hypotheses_.end());

  const std::size_t cap = std::max<std::size_t>(params_.max_hypotheses, 1);
  if (hypotheses_.size() > cap) {
    hypotheses_.resize(cap);
  }

  normalize();
}

void RouteTracker::update(const PathReference & ref, const Eigen::Vector2d & pos, double dt)
{
  if (!ref.valid()) {
    reset();
    return;
  }

  const double step_dt = std::max(dt, 0.0);

  // 内部估计世界系速度，绕开 Odometry::twist 的坐标系约定差异。
  if (has_last_pos_ && step_dt > 1e-6) {
    const Eigen::Vector2d raw_velocity = (pos - last_pos_) / step_dt;
    const double alpha = std::clamp(params_.velocity_alpha, 0.0, 1.0);
    velocity_ = alpha * raw_velocity + (1.0 - alpha) * velocity_;
  }
  last_pos_ = pos;
  has_last_pos_ = true;

  if (hypotheses_.empty()) {
    seed(ref, pos);
  }

  const double total = ref.total_length();
  const double sigma = std::max(params_.position_sigma, 1e-3);
  const double speed = velocity_.norm();
  const bool direction_usable =
    params_.direction_weight > 0.0 && speed >= std::max(params_.min_speed_for_direction, 0.0);
  const Eigen::Vector2d velocity_direction =
    direction_usable ? Eigen::Vector2d(velocity_ / speed) : Eigen::Vector2d::Zero();
  const double arc_alpha = std::clamp(params_.arc_rate_alpha, 0.0, 1.0);

  for (auto & h : hypotheses_) {
    const double predicted_s = std::clamp(h.s + h.s_dot * step_dt, 0.0, total);

    double residual = 0.0;
    const double refined_s = refine(ref, pos, predicted_s, residual);

    if (h.fresh) {
      // 刚播撒的假设，s 是任意的播撒点，不能用它算变化率。
      h.s_dot = 0.0;
      h.fresh = false;
    } else if (step_dt > 1e-6) {
      const double raw_rate = (refined_s - h.s) / step_dt;
      h.s_dot = arc_alpha * raw_rate + (1.0 - arc_alpha) * h.s_dot;
    }
    h.s = refined_s;
    h.residual = residual;

    double likelihood = std::exp(-0.5 * (residual * residual) / (sigma * sigma));
    if (direction_usable) {
      const Eigen::Vector2d tangent = ref.tangent_by_arc(refined_s);
      // 速度方向与路径切向的一致性：这是单帧最近点无法区分「前进/倒退」的关键。
      const double alignment = tangent.norm() > 1e-6 ? velocity_direction.dot(tangent) : 0.0;
      likelihood *= std::exp(params_.direction_weight * (alignment - 1.0));
    }
    h.weight *= std::max(likelihood, 1e-12);
  }

  prune();

  const auto best = std::max_element(
    hypotheses_.begin(), hypotheses_.end(),
    [](const Hypothesis & lhs, const Hypothesis & rhs) { return lhs.weight < rhs.weight; });
  if (best == hypotheses_.end()) {
    return;
  }

  best_s_ = best->s;
  best_s_dot_ = best->s_dot;
  best_residual_ = best->residual;
  // 对外报告单调不减：内部假设切换不应让下游看到进度回退。
  reported_s_ = std::max(reported_s_, best_s_);

  if (best_residual_ > params_.max_track_error) {
    over_error_time_ += step_dt;
    if (over_error_time_ >= std::max(params_.lost_grace_time, 0.0)) {
      lost_ = true;
    }
  } else {
    over_error_time_ = 0.0;
    lost_ = false;
  }
}

}  // namespace navigation2::mpc

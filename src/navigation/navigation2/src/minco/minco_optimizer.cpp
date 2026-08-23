#include "minco_optimizer.hpp"
#include "lbfgs.hpp"
#include <algorithm>
#include <cmath>
#include <rclcpp/logging.hpp>

namespace navigation2
{

struct MincoOptimizer::Impl
{
  Params params_;
  minco::MINCO_S3NU minco_;
  lbfgs::lbfgs_parameter_t lbfgs_params_;

  std::function<bool(const Eigen::Vector2d &, Eigen::Vector2d &)> tunnel_axis_query_;
  std::function<bool(const Eigen::Vector2d &, double &, Eigen::Vector2d &)> distance_query_;

  // 优化过程中的临时变量
  int piece_num_{0};
  Eigen::VectorXd in_times_;
  std::vector<Eigen::Vector2d> waypoints_;
  std::vector<int> opt_indices_;

  // Objective 热循环工作区：按 piece_num_ 在 optimize() 入口 resize 一次，cost() 只覆写
  // 和 setZero，不再为每次 line-search evaluation 创建多组 Eigen 堆对象。
  Eigen::VectorXd full_x_cache_;
  Eigen::Matrix2Xd in_ps_cache_;
  Eigen::Matrix2Xd energy_grad_cache_;
  Eigen::VectorXd energy_time_grad_cache_;
  Eigen::MatrixX2d partial_coeff_grad_cache_;
  Eigen::MatrixX2d adj_coeff_grad_cache_;
  Eigen::VectorXd partial_time_grad_cache_;
  Eigen::Matrix2Xd point_grad_cache_;
  Eigen::VectorXd full_grad_cache_;

  Impl()
  {
    lbfgs_params_.mem_size = 32;
    lbfgs_params_.past = 20;
    lbfgs_params_.min_step = 1e-32;
    lbfgs_params_.g_epsilon = 2.0e-7;
    lbfgs_params_.delta = 2e-7;
    lbfgs_params_.max_iterations = 4000;
    lbfgs_params_.max_linesearch = 32;
    lbfgs_params_.f_dec_coeff = 1e-4;
    lbfgs_params_.s_curv_coeff = 0.9;
  }

  // Kahan summation for numerical stability
  static inline double kahan_sum(double & sum, double & c, const double & val) noexcept
  {
    double y = val - c;
    double t = sum + y;
    c = (t - sum) - y;
    sum = t;
    return sum;
  }

  // 隧道轴线对齐软代价，逐段累加。段 k 从 point(k) 走到 point(k+1)，若该段落在
  // 隧道本体内，代价 = w * (1 - |cos θ|)，θ 是段走向与轴线的夹角。
  //
  // 为什么按段而不是按点：对齐约束的是「走向」，而走向是相邻两点之差，天然是段的
  // 属性。梯度落到段两端的可优化点上（边界点固定，梯度被丢弃）。
  //
  // 双向取 |·|：隧道正进倒进等价（见 SemanticMap::axisAlignment）。
  double attach_axis_functional(const Eigen::Matrix2Xd & in_ps, Eigen::Matrix2Xd & gradp) const
    noexcept
  {
    const double w = params_.tunnel_axis_weight;
    if (w <= 0.0 || !tunnel_axis_query_) {
      return 0.0;
    }
    const int M = piece_num_ + 1;  // 总点数（含首尾）
    if (M < 2) {
      return 0.0;
    }

    // 取第 k 个点：0 是首、M-1 是尾（均固定），其余是 in_ps 的列（可优化）。
    auto point_at = [&](int k) -> Eigen::Vector2d {
      if (k == 0) {
        return waypoints_.front();
      }
      if (k == M - 1) {
        return waypoints_.back();
      }
      return in_ps.col(k - 1);
    };

    double cost_val = 0.0;
    double c_cost = 0.0;

    for (int k = 0; k < M - 1; ++k) {
      const Eigen::Vector2d pa = point_at(k);
      const Eigen::Vector2d pb = point_at(k + 1);

      // 段落在洞里才算：查两端，任一端在本体内就用那端的轴线。直洞整条同轴，取哪端
      // 都一样；进/出洞的段只有一端在洞内，必须用洞内那端，否则洞口段不受约束。
      Eigen::Vector2d axis;
      if (!tunnel_axis_query_(pa, axis) && !tunnel_axis_query_(pb, axis)) {
        continue;
      }

      const Eigen::Vector2d d = pb - pa;
      const double n = d.norm();
      if (n < 1e-9) {
        continue;
      }
      const Eigen::Vector2d u = d / n;
      const double proj = u.dot(axis);       // = cos θ（axis 已归一化）
      const double s = (proj >= 0.0) ? 1.0 : -1.0;

      kahan_sum(cost_val, c_cost, w * (1.0 - std::abs(proj)));

      // dcost/dd = -w * (s/n) * (axis - proj * u)，见推导：|cos θ| 对段向量的梯度
      // 是轴线在垂直于走向方向上的分量。dd/dpb = +I，dd/dpa = -I。
      const Eigen::Vector2d dcost_dd = -w * (s / n) * (axis - proj * u);
      if (!dcost_dd.allFinite()) {
        continue;
      }
      // 只有内部点（k 或 k+1 落在 [1, M-2]）进 gradp；边界点固定。
      if (k >= 1 && k <= M - 2) {
        gradp.col(k - 1).noalias() += -dcost_dd;   // 对 pa
      }
      if (k + 1 >= 1 && k + 1 <= M - 2) {
        gradp.col(k).noalias() += dcost_dd;         // 对 pb
      }
    }

    return cost_val;
  }

  // 数据保持项：把每个内部点拉回原始路径点。
  double attach_penalty_functional(const Eigen::Matrix2Xd & in_ps, Eigen::Matrix2Xd & gradp) const noexcept
  {
    const int N = in_ps.cols();
    if (N <= 0) {
      return 0.0;
    }

    double cost_val = 0.0;
    double c_cost = 0.0;

    for (int i = 0; i < N; i++) {
      const Eigen::Vector2d & p0 = in_ps.col(i);

      // 数据项代价（保持接近原路径）
      const Eigen::Vector2d & original = waypoints_[i + 1];
      Eigen::Vector2d deviation = p0 - original;
      double data_cost = params_.data_weight * deviation.squaredNorm();
      kahan_sum(cost_val, c_cost, data_cost);
      gradp.col(i).noalias() += 2.0 * params_.data_weight * deviation;
    }

    return cost_val;
  }

  // 障碍 soft 代价：控制点处到最近障碍距离 d < safe_dist 时加
  // w*(safe_dist-d)²，梯度沿距离场远离障碍方向。与 sentry 的 ESDF 位置代价一致，
  // 但落在 Lucifer 已有的进程内距离场上（障碍外为正）。车本不该在障碍内；
  // 恢复倒车方向可读同一份 DistanceFieldRegistry，HAZARD 安全点仍是环采样。
  double attach_obstacle_functional(const Eigen::Matrix2Xd & in_ps, Eigen::Matrix2Xd & gradp) const noexcept
  {
    const double w = params_.obstacle_weight;
    const double safe = params_.safe_dist;
    if (w <= 0.0 || safe <= 0.0 || !distance_query_) {
      return 0.0;
    }
    const int M = piece_num_ + 1;  // 总点数（含首尾）
    const auto point_at = [&](int k) -> Eigen::Vector2d {
        if (k == 0) return waypoints_.front();
        if (k == M - 1) return waypoints_.back();
        return in_ps.col(k - 1);
      };
    double cost_val = 0.0;
    double c_cost = 0.0;

    for (int k = 0; k < M; ++k) {
      Eigen::Vector2d p;
      if (k == 0) {
        p = waypoints_.front();
      } else if (k == M - 1) {
        p = waypoints_.back();
      } else {
        p = in_ps.col(k - 1);
      }
      double d = 0.0;
      Eigen::Vector2d grad = Eigen::Vector2d::Zero();
      if (!distance_query_(p, d, grad)) {
        continue;
      }
      if (!std::isfinite(d) || d >= safe) {
        continue;
      }
      const double margin = safe - d;
      kahan_sum(cost_val, c_cost, w * margin * margin);
      // dcost/dp = -2w(safe-d) * grad。grad 指向远离障碍，距离越近代价越高，
      // 所以梯度沿 -grad 方向（把点往远离障碍推）。
      Eigen::Vector2d g = -2.0 * w * margin * grad;
      if (params_.obstacle_normal_only) {
        // 以相邻路标的几何方向近似该点轨迹切向；端点用单侧差分。仅投影 soft
        // 障碍梯度，距离值和发布前硬安全检查不变。
        Eigen::Vector2d tangent = Eigen::Vector2d::Zero();
        if (k == 0 && M > 1) tangent = point_at(1) - point_at(0);
        else if (k == M - 1 && M > 1) tangent = point_at(M - 1) - point_at(M - 2);
        else if (k > 0 && k + 1 < M) tangent = point_at(k + 1) - point_at(k - 1);
        const double tangent_norm = tangent.norm();
        if (tangent_norm > 1e-9) {
          tangent /= tangent_norm;
          g -= g.dot(tangent) * tangent;
        }
      }
      if (!g.allFinite()) {
        continue;
      }
      // 只有内部点进 gradp；边界点固定。
      if (k >= 1 && k <= M - 2) {
        gradp.col(k - 1).noalias() += g;
      }
    }
    return cost_val;
  }

  // L-BFGS 代价函数
  static double cost(void * ptr, const Eigen::VectorXd & x, Eigen::VectorXd & g) noexcept
  {
    auto * instance = static_cast<MincoOptimizer::Impl *>(ptr);

    const int ctrl_num = instance->piece_num_ - 1;
    const int opt_num = instance->opt_indices_.size();

    double cost_val = 0.0;

    auto & full_x = instance->full_x_cache_;

    for (int i = 0; i < ctrl_num; i++) {
      full_x(i) = instance->waypoints_[i + 1].x();
      full_x(i + ctrl_num) = instance->waypoints_[i + 1].y();
    }

    for (int k = 0; k < opt_num; ++k) {
      int i = instance->opt_indices_[k];
      full_x(i) = x(k);
      full_x(i + ctrl_num) = x(k + opt_num);
    }

    auto & in_ps = instance->in_ps_cache_;
    in_ps.row(0) = full_x.head(ctrl_num).transpose();
    in_ps.row(1) = full_x.segment(ctrl_num, ctrl_num).transpose();

    instance->minco_.setParameters(in_ps, instance->in_times_);

    double energy = 0.0;

    auto & energy_grad = instance->energy_grad_cache_;
    energy_grad.setZero();
    auto & energyT_grad = instance->energy_time_grad_cache_;
    energyT_grad.setZero();

    auto & partial_grad_by_coeffs = instance->partial_coeff_grad_cache_;
    auto & partial_grad_by_times = instance->partial_time_grad_cache_;

    instance->minco_.getEnergyPartialGradByCoeffs(partial_grad_by_coeffs);
    instance->minco_.getEnergyPartialGradByTimes(partial_grad_by_times);
    instance->minco_.getEnergy(energy);

    instance->minco_.propogateGrad(
      partial_grad_by_coeffs,
      partial_grad_by_times,
      energy_grad,
      energyT_grad,
      instance->adj_coeff_grad_cache_);

    auto & gradp = instance->point_grad_cache_;
    gradp = energy_grad;

    cost_val += energy;
    cost_val += instance->attach_penalty_functional(in_ps, gradp);
    cost_val += instance->attach_axis_functional(in_ps, gradp);
    cost_val += instance->attach_obstacle_functional(in_ps, gradp);

    auto & g_full = instance->full_grad_cache_;
    g_full.setZero();

    g_full.segment(0, ctrl_num) = gradp.row(0).transpose();
    g_full.segment(ctrl_num, ctrl_num) = gradp.row(1).transpose();

    g.resize(2 * opt_num);
    g.setZero();

    for (int k = 0; k < opt_num; ++k) {
      int i = instance->opt_indices_[k];
      g(k) = g_full(i);
      g(k + opt_num) = g_full(i + ctrl_num);
    }

    return cost_val;
  }

  std::vector<Piece<5, 2>> optimize(
    const std::vector<Eigen::Vector2d> & waypoints,
    const std::vector<double> & segment_times)
  {
    if (waypoints.size() < 2) {
      RCLCPP_ERROR(rclcpp::get_logger("minco_optimizer"), "waypoints too small");
      return {};
    }

    piece_num_ = static_cast<int>(waypoints.size()) - 1;
    if (segment_times.size() != static_cast<std::size_t>(piece_num_)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("minco_optimizer"),
        "segment_times size mismatch: expected %d, got %zu",
        piece_num_, segment_times.size());
      return {};
    }
    for (const double t : segment_times) {
      // 零/负/非有限段长时间会让 MINCO 的幂次矩阵与无主元带状 LU 产生
      // 除零或 NaN 轨迹，必须在这里拦掉而不是让 NaN 一路流到 MPC。
      if (!std::isfinite(t) || t <= 0.0) {
        RCLCPP_ERROR(
          rclcpp::get_logger("minco_optimizer"),
          "segment time must be finite and positive, got %.6f", t);
        return {};
      }
    }

    waypoints_ = waypoints;

    const auto w_smooth = params_.smooth_weight;

    Eigen::Matrix<double, 2, 3> head_state;
    head_state << waypoints.front(), Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero();

    Eigen::Matrix<double, 2, 3> tail_state;
    tail_state << waypoints.back(), Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero();

    minco_.setConditions(head_state, tail_state, piece_num_, Eigen::Vector2d(w_smooth, w_smooth));

    const int ctrl_num = piece_num_ - 1;
    // 一条 optimize 调用内维度固定；预先准备 objective 的所有动态矩阵。Eigen resize
    // 在尺寸不变时复用容量，第二阶段同路标数也直接复用第一阶段工作区。
    full_x_cache_.resize(2 * ctrl_num);
    in_ps_cache_.resize(2, ctrl_num);
    energy_grad_cache_.resize(2, ctrl_num);
    energy_time_grad_cache_.resize(ctrl_num + 1);
    partial_coeff_grad_cache_.resize(6 * piece_num_, 2);
    adj_coeff_grad_cache_.resize(6 * piece_num_, 2);
    partial_time_grad_cache_.resize(piece_num_);
    point_grad_cache_.resize(2, ctrl_num);
    full_grad_cache_.resize(2 * ctrl_num);

    opt_indices_.clear();
    for (int i = 0; i < ctrl_num; ++i) {
      opt_indices_.push_back(i);
    }
    const int opt_num = opt_indices_.size();

    Eigen::VectorXd x_opt(2 * opt_num);

    for (int k = 0; k < opt_num; ++k) {
      int i = opt_indices_[k];
      x_opt(k) = waypoints[i + 1].x();
      x_opt(k + opt_num) = waypoints[i + 1].y();
    }

    in_times_.resize(piece_num_);
    for (int i = 0; i < piece_num_; ++i) {
      in_times_(i) = segment_times[i];
    }

    double min_cost = 0.0;
    int ret = 0;

    if (opt_num > 0 && params_.enable) {
      ret = lbfgs::lbfgs_optimize(
        x_opt,
        min_cost,
        &MincoOptimizer::Impl::cost,
        nullptr,
        nullptr,
        this,
        lbfgs_params_);

      // 优化失败时返回空轨迹，让节点的“发布原始路径”回退真正生效。
      // 旧实现失败仍把可能已经发散的 x_opt 写回轨迹，NaN 路径会一路流到 MPC。
      if (!(ret >= 0 || ret == lbfgs::LBFGSERR_MAXIMUMLINESEARCH)) {
        RCLCPP_ERROR(
          rclcpp::get_logger("minco_optimizer"),
          "MINCO optimization failed: %s", lbfgs::lbfgs_strerror(ret));
        return {};
      }
    }

    Eigen::Matrix2Xd in_ps(2, ctrl_num);
    Eigen::VectorXd full_x(2 * ctrl_num);

    for (int i = 0; i < ctrl_num; ++i) {
      full_x(i) = waypoints[i + 1].x();
      full_x(i + ctrl_num) = waypoints[i + 1].y();
    }

    for (int k = 0; k < opt_num; ++k) {
      int i = opt_indices_[k];
      full_x(i) = x_opt(k);
      full_x(i + ctrl_num) = x_opt(k + opt_num);
    }

    in_ps.row(0) = full_x.head(ctrl_num).transpose();
    in_ps.row(1) = full_x.segment(ctrl_num, ctrl_num).transpose();

    minco_.setParameters(in_ps, in_times_);

    std::vector<Piece<5, 2>> final_traj;
    minco_.getPieces(final_traj);

    return final_traj;
  }
};

MincoOptimizer::MincoOptimizer()
: impl_(std::make_unique<Impl>())
{
}

MincoOptimizer::~MincoOptimizer() = default;

void MincoOptimizer::setParams(const Params & params)
{
  impl_->params_ = params;
  impl_->lbfgs_params_.mem_size = std::clamp(params.lbfgs_memory_size, 4, 128);
  impl_->lbfgs_params_.max_iterations = std::max(1, params.max_iterations);
}

void MincoOptimizer::setTunnelAxisQuery(
  std::function<bool(const Eigen::Vector2d &, Eigen::Vector2d &)> query_fn)
{
  impl_->tunnel_axis_query_ = query_fn;
}

void MincoOptimizer::setDistanceQuery(
  std::function<bool(const Eigen::Vector2d &, double &, Eigen::Vector2d &)> query_fn)
{
  impl_->distance_query_ = query_fn;
}

std::vector<Piece<5, 2>> MincoOptimizer::optimize(
  const std::vector<Eigen::Vector2d> & waypoints,
  const std::vector<double> & segment_times)
{
  return impl_->optimize(waypoints, segment_times);
}

}  // namespace navigation2

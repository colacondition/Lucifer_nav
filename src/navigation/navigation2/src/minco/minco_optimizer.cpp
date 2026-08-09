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

  std::function<bool(const Eigen::Vector2d &, double &, Eigen::Vector2d &)> esdf_query_;

  // 优化过程中的临时变量
  int piece_num_{0};
  Eigen::VectorXd in_times_;
  std::vector<Eigen::Vector2d> waypoints_;
  std::vector<int> opt_indices_;

  Impl()
  {
    lbfgs_params_.mem_size = 256;
    lbfgs_params_.past = 20;
    lbfgs_params_.min_step = 1e-32;
    lbfgs_params_.g_epsilon = 2.0e-7;
    lbfgs_params_.delta = 2e-7;
    lbfgs_params_.max_iterations = 4000;
    lbfgs_params_.max_linesearch = 32;
    lbfgs_params_.f_dec_coeff = 1e-4;
    lbfgs_params_.s_curv_coeff = 0.9;
  }

  // Smoothed L1 penalty function
  static inline bool smoothed_l1(
    const double & x, const double & mu, double & f, double & df) noexcept
  {
    if (x < 0.0) {
      return false;
    } else if (x > mu) {
      f = x - 0.5 * mu;
      df = 1.0;
      return true;
    } else {
      const double xdmu = x / mu;
      const double sqrxdmu = xdmu * xdmu;
      const double mumxd2 = mu - 0.5 * x;
      f = mumxd2 * sqrxdmu * xdmu;
      df = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);
      return true;
    }
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

  // 障碍物代价项
  Eigen::Vector2d obstacle_term(const Eigen::Vector2d & xcur, double & nearest_cost) const noexcept
  {
    nearest_cost = 0.0;
    Eigen::Vector2d grad = Eigen::Vector2d::Zero();

    const double R = params_.robot_radius;
    const double mu = params_.penalty_mu;

    double d;
    Eigen::Vector2d g;

    // 调用 ESDF 查询函数
    if (!esdf_query_ || !esdf_query_(xcur, d, g) || !std::isfinite(d)) {
      return grad;
    }

    if (d > R) {
      return grad;
    }

    double penetration = R - d;
    double cost_s1 = 0.0, dcost_s1 = 0.0;
    if (!smoothed_l1(penetration, mu, cost_s1, dcost_s1)) {
      return grad;
    }

    const auto w_obs = params_.obstacle_weight;
    nearest_cost = w_obs * cost_s1;
    Eigen::Vector2d dir = (g.norm() > 1e-6) ? g.normalized() : Eigen::Vector2d::Zero();
    grad = w_obs * dcost_s1 * (-dir);

    if (!grad.allFinite()) {
      grad.setZero();
    }

    return grad;
  }

  // 附加障碍物惩罚项到梯度
  double attach_penalty_functional(const Eigen::Matrix2Xd & in_ps, Eigen::Matrix2Xd & gradp) const noexcept
  {
    const int N = in_ps.cols();
    if (N < 2) {
      return 0.0;
    }

    double cost_val = 0.0;
    double c_cost = 0.0;

    for (int i = 0; i < N - 1; i++) {
      double nearest_cost = 0.0;
      const Eigen::Vector2d & p0 = in_ps.col(i);

      // 1. 障碍物代价
      Eigen::Vector2d obs_grad = obstacle_term(p0, nearest_cost);
      kahan_sum(cost_val, c_cost, nearest_cost);
      gradp.col(i).noalias() += obs_grad;

      // 2. 数据项代价（保持接近原路径）
      const Eigen::Vector2d & original = waypoints_[i + 1];
      Eigen::Vector2d deviation = p0 - original;
      double data_cost = params_.data_weight * deviation.squaredNorm();
      kahan_sum(cost_val, c_cost, data_cost);
      gradp.col(i).noalias() += 2.0 * params_.data_weight * deviation;
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

    Eigen::VectorXd full_x(2 * ctrl_num);

    for (int i = 0; i < ctrl_num; i++) {
      full_x(i) = instance->waypoints_[i + 1].x();
      full_x(i + ctrl_num) = instance->waypoints_[i + 1].y();
    }

    for (int k = 0; k < opt_num; ++k) {
      int i = instance->opt_indices_[k];
      full_x(i) = x(k);
      full_x(i + ctrl_num) = x(k + opt_num);
    }

    Eigen::Matrix2Xd in_ps(2, ctrl_num);
    in_ps.row(0) = full_x.head(ctrl_num).transpose();
    in_ps.row(1) = full_x.segment(ctrl_num, ctrl_num).transpose();

    instance->minco_.setParameters(in_ps, instance->in_times_);

    double energy = 0.0;

    Eigen::Matrix2Xd energy_grad = Eigen::Matrix2Xd::Zero(2, ctrl_num);
    Eigen::VectorXd energyT_grad = Eigen::VectorXd::Zero(ctrl_num + 1);

    Eigen::MatrixX2d partial_grad_by_coeffs;
    Eigen::VectorXd partial_grad_by_times;

    instance->minco_.getEnergyPartialGradByCoeffs(partial_grad_by_coeffs);
    instance->minco_.getEnergyPartialGradByTimes(partial_grad_by_times);
    instance->minco_.getEnergy(energy);

    instance->minco_.propogateGrad(
      partial_grad_by_coeffs,
      partial_grad_by_times,
      energy_grad,
      energyT_grad);

    Eigen::Matrix2Xd gradp = energy_grad;

    cost_val += energy;
    cost_val += instance->attach_penalty_functional(in_ps, gradp);

    Eigen::VectorXd g_full(2 * ctrl_num);
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

    waypoints_ = waypoints;
    piece_num_ = waypoints.size() - 1;

    const auto w_smooth = params_.smooth_weight;

    Eigen::Matrix<double, 2, 3> head_state;
    head_state << waypoints.front(), Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero();

    Eigen::Matrix<double, 2, 3> tail_state;
    tail_state << waypoints.back(), Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero();

    minco_.setConditions(head_state, tail_state, piece_num_, Eigen::Vector2d(w_smooth, w_smooth));

    const int ctrl_num = piece_num_ - 1;

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

    if (!(ret >= 0 || ret == lbfgs::LBFGSERR_MAXIMUMLINESEARCH)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("minco_optimizer"),
        "MINCO optimization failed: %s", lbfgs::lbfgs_strerror(ret));
    }

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
}

void MincoOptimizer::setEsdfQuery(
  std::function<bool(const Eigen::Vector2d &, double &, Eigen::Vector2d &)> query_fn)
{
  impl_->esdf_query_ = query_fn;
}

std::vector<Piece<5, 2>> MincoOptimizer::optimize(
  const std::vector<Eigen::Vector2d> & waypoints,
  const std::vector<double> & segment_times)
{
  return impl_->optimize(waypoints, segment_times);
}

}  // namespace navigation2

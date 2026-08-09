// 使用 OSQP 0.6.x C API 的全向跟踪 MPC。
//
// 决策向量 z = [x_0..x_{N-1}（每个 2 维），u_0..u_{N-1}（每个 2 维）]，N=steps。
// 代价：sum ||x_i - xref_i||_Q + ||u_i - uref_i||_R + ||du||_Rd
// 约束：x_0 = x_init；x_i = x_{i-1} + u_{i-1}*dt（动力学）；
//       |u_i| <= vmax；|u_i - u_{i-1}| <= amax*dt。
// 这里仅做轨迹跟踪 QP；避障由全局规划器和局部安全检查负责。
#pragma once
#include <algorithm>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <osqp.h>

namespace navigation2::mpc {

struct MpcParams
{
  int steps = 30;
  double dt = 0.1;
  double max_speed = 2.0;
  double max_accel = 1.0;
  double turtle_max_speed = 1.0;
  std::vector<double> Q{15.0, 15.0};   // 位置跟踪权重（x, y）
  std::vector<double> R{0.1, 0.1};     // 控制量大小权重（vx, vy）
  std::vector<double> Rd{1.0, 0.05};   // 控制平滑权重（dvx, dvy）
};

class MpcSolver
{
public:
  // 释放 OSQP 资源。
  ~MpcSolver() { cleanup(); }

  // 重新装配一组参数。
  void configure(const MpcParams & p)
  {
    params_ = p;
    cleanup();
    setup_done_ = false;
  }

  // 输入参考轨迹，输出每步的速度序列。
  std::vector<Eigen::Vector2d> solve(
    const Eigen::MatrixXd & xref, const Eigen::MatrixXd & uref,
    const Eigen::Vector2d & x_init, bool turtle)
  {
    const int steps = params_.steps;
    if (steps <= 0) {
      return {};
    }
    const int dimx = 2 * steps;
    const int dimu = 2 * steps;
    const int nx = dimx + dimu;

    buildHessian();                                   // P（固定稀疏结构）
    buildGradient(xref, uref);                        // q
    buildConstraintMatrix();                          // A（固定稀疏结构）
    buildBounds(x_init, turtle);                      // l, u

    if (!setup_done_) {
      if (!setup(nx)) {
        return {};
      }
    } else {
      osqp_update_lin_cost(work_, q_.data());
      osqp_update_bounds(work_, l_.data(), u_.data());
    }

    if (osqp_solve(work_) != 0 || work_->info->status_val <= 0) {
      return {};
    }

    // 复用输出缓冲区（避免每次分配）。
    if (output_cache_.size() != static_cast<size_t>(steps)) {
      output_cache_.resize(steps, Eigen::Vector2d::Zero());
    }
    for (int i = 0; i < steps; ++i) {
      const int ui = dimx + 2 * i;
      output_cache_[i] = Eigen::Vector2d(work_->solution->x[ui], work_->solution->x[ui + 1]);
    }
    return output_cache_;
  }

private:
  void buildHessian()
  {
    // 代价矩阵只在结构上固定,数值按参数重建。
    const int steps = params_.steps;
    const int dimx = 2 * steps;
    const int nx = dimx + 2 * steps;
    const auto & Q = params_.Q;
    const auto & R = params_.R;
    const auto & Rd = params_.Rd;

    // 预分配复用缓存。
    if (hessian_cols_cache_.size() != static_cast<size_t>(nx)) {
      hessian_cols_cache_.resize(nx);
    }
    for (auto & col : hessian_cols_cache_) {
      col.clear();
    }
    std::vector<std::vector<std::pair<int, double>>> & cols = hessian_cols_cache_;
    auto add = [&](int r, int c, double v) {
      if (r <= c) {
        cols[c].emplace_back(r, v);
      }
    };
    for (int i = 1; i < steps; ++i) {          // 状态代价（i==0 由等式约束固定）
      add(2 * i, 2 * i, 2.0 * Q[0]);
      add(2 * i + 1, 2 * i + 1, 2.0 * Q[1]);
    }
    for (int i = 0; i < steps; ++i) {          // 控制代价 + 平滑项对角部分
      const int ui = dimx + 2 * i;
      double w0 = R[0], w1 = R[1];
      if (i == 0 || i == steps - 1) { w0 += Rd[0]; w1 += Rd[1]; }
      else { w0 += 2.0 * Rd[0]; w1 += 2.0 * Rd[1]; }
      add(ui, ui, 2.0 * w0);
      add(ui + 1, ui + 1, 2.0 * w1);
    }
    for (int i = 1; i < steps; ++i) {          // 平滑项非对角部分（只写上三角）
      const int ui = dimx + 2 * i;
      const int up = dimx + 2 * (i - 1);
      add(up, ui, -2.0 * Rd[0]);
      add(up + 1, ui + 1, -2.0 * Rd[1]);
    }
    for (auto & col : cols) {                  // CSC 要求每一列内的行号升序
      std::sort(col.begin(), col.end());
    }
    toCsc(cols, nx, P_p_, P_i_, P_x_);
  }

  void buildGradient(const Eigen::MatrixXd & xref, const Eigen::MatrixXd & uref)
  {
    // 把参考轨迹写进线性项。
    const int steps = params_.steps;
    const int dimx = 2 * steps;
    const int nx = dimx + 2 * steps;
    const auto & Q = params_.Q;
    const auto & R = params_.R;
    q_.assign(nx, 0.0);
    for (int i = 0; i < steps; ++i) {
      q_[2 * i] = -2.0 * Q[0] * xref(0, i);
      q_[2 * i + 1] = -2.0 * Q[1] * xref(1, i);
    }
    for (int i = 0; i < steps; ++i) {
      const int ui = dimx + 2 * i;
      q_[ui] = -2.0 * R[0] * uref(0, i);
      q_[ui + 1] = -2.0 * R[1] * uref(1, i);
    }
  }

  void buildConstraintMatrix()
  {
    // 动力学、速度上界和加速度上界都在这里。
    const int steps = params_.steps;
    const int dimx = 2 * steps;
    const int nx = dimx + 2 * steps;
    // 行分布：[动力学 2*steps][速度边界 2*steps][加速度平滑 2*(steps-1)]
    const int dyn = 2 * steps;
    const int veloff = dyn;
    const int accoff = dyn + 2 * steps;
    const double dt = params_.dt;

    // 预分配复用缓存。
    if (constraint_cols_cache_.size() != static_cast<size_t>(nx)) {
      constraint_cols_cache_.resize(nx);
    }
    for (auto & col : constraint_cols_cache_) {
      col.clear();
    }
    std::vector<std::vector<std::pair<int, double>>> & cols = constraint_cols_cache_;
    auto add = [&](int r, int c, double v) { cols[c].emplace_back(r, v); };

    // 初始状态：x_0 对应第 0、1 行。
    add(0, 0, 1.0);
    add(1, 1, 1.0);
    // 动力学：x_i - x_{i-1} - u_{i-1}*dt = 0，对应第 2*i 行。
    for (int i = 1; i < steps; ++i) {
      const int r = 2 * i;
      const int last = 2 * (i - 1);
      const int uc = dimx + 2 * (i - 1);
      add(r, r, 1.0); add(r, last, -1.0); add(r, uc, -dt);
      add(r + 1, r + 1, 1.0); add(r + 1, last + 1, -1.0); add(r + 1, uc + 1, -dt);
    }
    // 每个 u_i 的速度边界。
    for (int i = 0; i < steps; ++i) {
      const int r = veloff + 2 * i;
      const int c = dimx + 2 * i;
      add(r, c, 1.0); add(r + 1, c + 1, 1.0);
    }
    // 加速度平滑：u_i - u_{i-1}。
    for (int i = 1; i < steps; ++i) {
      const int r = accoff + 2 * (i - 1);
      const int u = dimx + 2 * i;
      const int up = dimx + 2 * (i - 1);
      add(r, u, 1.0); add(r, up, -1.0);
      add(r + 1, u + 1, 1.0); add(r + 1, up + 1, -1.0);
    }
    ncon_ = accoff + 2 * (steps - 1);
    // CSC 要求每列中的行号按升序排列。
    for (auto & col : cols) {
      std::sort(col.begin(), col.end());
    }
    toCsc(cols, nx, A_p_, A_i_, A_x_);
  }

  void buildBounds(const Eigen::Vector2d & x_init, bool turtle)
  {
    // 初值、限速和限加速度。
    const int steps = params_.steps;
    const int dyn = 2 * steps;
    const int veloff = dyn;
    const int accoff = dyn + 2 * steps;
    const double vmax = turtle ? params_.turtle_max_speed : params_.max_speed;
    const double dv = params_.max_accel * params_.dt;

    l_.assign(ncon_, -1e30);
    u_.assign(ncon_, 1e30);
    // 初始状态等式约束。
    l_[0] = u_[0] = x_init.x();
    l_[1] = u_[1] = x_init.y();
    // 动力学等式 = 0。
    for (int i = 1; i < steps; ++i) {
      l_[2 * i] = u_[2 * i] = 0.0;
      l_[2 * i + 1] = u_[2 * i + 1] = 0.0;
    }
    // 速度边界。
    for (int i = 0; i < steps; ++i) {
      const int r = veloff + 2 * i;
      l_[r] = -vmax; u_[r] = vmax;
      l_[r + 1] = -vmax; u_[r + 1] = vmax;
    }
    // 加速度差分边界。
    for (int i = 1; i < steps; ++i) {
      const int r = accoff + 2 * (i - 1);
      l_[r] = -dv; u_[r] = dv;
      l_[r + 1] = -dv; u_[r + 1] = dv;
    }
  }

  bool setup(int nx)
  {
    // 第一次求解时创建 OSQP workspace。
    P_csc_ = csc_matrix(nx, nx, static_cast<c_int>(P_x_.size()), P_x_.data(), P_i_.data(),
        P_p_.data());
    A_csc_ = csc_matrix(ncon_, nx, static_cast<c_int>(A_x_.size()), A_x_.data(), A_i_.data(),
        A_p_.data());
    data_ = static_cast<OSQPData *>(c_malloc(sizeof(OSQPData)));
    data_->n = nx;
    data_->m = ncon_;
    data_->P = P_csc_;
    data_->A = A_csc_;
    data_->q = q_.data();
    data_->l = l_.data();
    data_->u = u_.data();

    settings_ = static_cast<OSQPSettings *>(c_malloc(sizeof(OSQPSettings)));
    osqp_set_default_settings(settings_);
    settings_->warm_start = 1;
    settings_->verbose = 0;
    settings_->max_iter = 2000;
    settings_->eps_abs = 1e-4;
    settings_->eps_rel = 1e-4;

    if (osqp_setup(&work_, data_, settings_) != 0) {
      work_ = nullptr;
      return false;
    }
    setup_done_ = true;
    return true;
  }

  void cleanup()
  {
    // 释放 OSQP 及稀疏矩阵缓存。
    if (work_) { osqp_cleanup(work_); work_ = nullptr; }
    if (data_) {
      // P_csc_/A_csc_ 由 csc_matrix 分配（只有结构体本体，数组仍归我们管理）。
      if (P_csc_) { c_free(P_csc_); P_csc_ = nullptr; }
      if (A_csc_) { c_free(A_csc_); A_csc_ = nullptr; }
      c_free(data_); data_ = nullptr;
    }
    if (settings_) { c_free(settings_); settings_ = nullptr; }
    setup_done_ = false;
  }

  static void toCsc(
    const std::vector<std::vector<std::pair<int, double>>> & cols, int ncol,
    std::vector<c_int> & p, std::vector<c_int> & i, std::vector<c_float> & x)
  {
    // 按 CSC 顺序展开列缓存。
    p.assign(ncol + 1, 0);
    i.clear();
    x.clear();
    for (int c = 0; c < ncol; ++c) {
      p[c + 1] = p[c] + static_cast<c_int>(cols[c].size());
      for (const auto & [row, val] : cols[c]) {
        i.push_back(row);
        x.push_back(val);
      }
    }
  }

  MpcParams params_;
  bool setup_done_ = false;
  int ncon_ = 0;

  std::vector<c_int> P_p_, P_i_, A_p_, A_i_;
  std::vector<c_float> P_x_, A_x_, q_, l_, u_;
  csc * P_csc_ = nullptr;
  csc * A_csc_ = nullptr;
  OSQPData * data_ = nullptr;
  OSQPSettings * settings_ = nullptr;
  OSQPWorkspace * work_ = nullptr;

  // 预分配复用：避免每次 solve() 都分配临时 vector。
  std::vector<std::vector<std::pair<int, double>>> hessian_cols_cache_;
  std::vector<std::vector<std::pair<int, double>>> constraint_cols_cache_;
  std::vector<Eigen::Vector2d> output_cache_;
};

}  // namespace navigation2::mpc

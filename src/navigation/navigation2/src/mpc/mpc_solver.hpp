// 使用 OSQP 0.6.x C API 的全向跟踪 MPC。
//
// 决策向量 z = [x_0..x_{N-1}（每个 2 维），u_0..u_{N-1}（每个 2 维）]，N=steps。
// 代价：sum ||x_i - xref_i||_Q + ||u_i - uref_i||_R + ||du||_Rd
// 约束：x_0 = x_init；x_i = x_{i-1} + u_{i-1}*dt（动力学）；
//       |u_i| <= vmax；|u_i - u_{i-1}| <= amax*dt。
// 这里仅做轨迹跟踪 QP；避障由全局规划器和局部安全检查负责。
//
// 速度上界的两种语义（enforce_speed_norm）：
//   false（历史行为）：逐分量 box |u_x|,|u_y| <= vmax。这是 L∞ 球，对角方向
//     实际能跑到 sqrt(2)*vmax —— 声明 2.0 m/s 的机器人斜 45° 会下发 2.83 m/s。
//   true：内接正八边形（L2 圆的 8 边线性近似）。轴向 4 个法向（0/45/90/135°）
//     的支撑距离统一取 vmax*cos(22.5°)，顶点恰好落在半径 vmax 的圆上 —— 既
//     保证 ||u||_2 <= vmax 处处成立，又把轴向速度只压到 0.924*vmax。八边形
//     内接于圆而不是外切，所以是「收紧到声明值」而不是「放大到对角值」。
//     实现上复用原有的 2 行速度 box（只是把界从 vmax 换成 vmax*cos(22.5°)），
//     再追加 2 行对角方向，净增 2*steps 行约束。
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <osqp.h>

namespace navigation2::mpc {

// 内接正八边形的支撑距离系数：半径 r 的圆内接八边形，各边法向到圆心的距离是
// r*cos(pi/8)。轴向速度因此从 vmax 压到 0.924*vmax，但对角方向从 1.414*vmax
// 压回 vmax —— 顶点正好落在圆上，是能保证 ||u||_2 <= vmax 的最紧 8 边线性近似。
constexpr double kOctagonSupport = 0.92387953251128673848;  // cos(22.5°)

struct MpcParams
{
  int steps = 30;
  double dt = 0.1;
  double max_speed = 2.0;
  double max_accel = 1.0;
  double turtle_max_speed = 1.0;
  std::vector<double> Q{15.0, 15.0};   // 兼容模式的位置跟踪权重（世界 x, y）
  bool path_frame_weighting = false;   // true: 按轨迹切/法向旋转位置权重
  double tangential_weight = 6.0;
  double lateral_weight = 24.0;
  double min_tangent_speed = 0.05;
  std::vector<double> R{0.1, 0.1};     // 控制量大小权重（vx, vy）
  std::vector<double> Rd{1.0, 0.05};   // 控制平滑权重（dvx, dvy）
  // OSQP 单次求解时限（秒）。0 = 不限（默认，行为与历史一致）。
  // 设正值可封住病态 QP 最坏耗时，但可能拿到 SOLVED_INACCURATE 解；
  // 是否接受由调用方看 inaccurateSolves() 计数自行取舍。
  double time_limit = 0.0;
  // 速度上界按 L2 模长收紧（内接八边形）。默认开：max_speed 声明的是速度上限，
  // 逐分量 box 让对角实际跑到 sqrt(2)*max_speed 是缺陷。轴向速度因此为
  // 0.924*max_speed；要保留对角速度就把 max_speed 提到 1/cos(22.5°) 倍。
  bool enforce_speed_norm = true;
};

// 参数合法性检查。MPC 的稀疏结构与工作区尺寸都由这些值决定，非法值会在
// configure() 里变成负尺寸 resize / 数组越界（一条 ros2 param set 即可击穿
// 控制节点），所以这里作为库级入口统一拒绝。
inline bool paramsAreValid(const MpcParams & p)
{
  return p.steps > 0 && p.dt > 0.0 && std::isfinite(p.dt) &&
         p.max_speed >= 0.0 && p.max_accel >= 0.0 && p.turtle_max_speed >= 0.0 &&
         p.tangential_weight >= 0.0 && p.lateral_weight >= 0.0 &&
         std::isfinite(p.tangential_weight) && std::isfinite(p.lateral_weight) &&
         p.min_tangent_speed >= 0.0 && std::isfinite(p.min_tangent_speed) &&
         p.Q.size() >= 2 && p.R.size() >= 2 && p.Rd.size() >= 2 &&
         p.time_limit >= 0.0 && std::isfinite(p.time_limit);
  // 注意：enforce_speed_norm 下 max_speed==0 是合法的（八边形退化成「必须停」，
  // 是强制停车的一种配置），不要在这里拒绝它 —— 拒绝会触发调用方的「回退默认
  // 参数」，反而让机器人按 2.0 m/s 跑起来。
}

class MpcSolver
{
public:
  // 释放 OSQP 资源。
  ~MpcSolver() { cleanup(); }

  // 重新装配一组参数。P/A 的稀疏结构与数值只依赖参数（与 xref/x_init 无关），
  // 在这里一次性建好；solve() 热路径只更新线性项与边界。
  void configure(const MpcParams & p)
  {
    if (!paramsAreValid(p)) {
      throw std::invalid_argument(
        "MpcSolver::configure: invalid parameters "
        "(require steps>0, dt>0, max_speed/max_accel/turtle_max_speed>=0, "
        "|Q|,|R|,|Rd| >= 2)");
    }
    params_ = p;
    cleanup();
    setup_done_ = false;
    buildHessian();                                   // P
    buildConstraintMatrix();                          // A、ncon_
  }

  // 输入参考轨迹，输出每步的速度序列。返回成员缓冲区的只读引用：
  // 失败时缓冲区被清空（empty 即失败），成功时直接复用，热路径零拷贝。
  const std::vector<Eigen::Vector2d> & solve(
    const Eigen::MatrixXd & xref, const Eigen::MatrixXd & uref,
    const Eigen::Vector2d & x_init, bool turtle)
  {
    const int steps = params_.steps;
    if (steps <= 0) {
      output_cache_.clear();
      return output_cache_;
    }
    const int dimx = 2 * steps;
    const int dimu = 2 * steps;
    const int nx = dimx + dimu;

    buildPathFrameWeights(xref, uref);
    updateStateHessian();                              // 路径切/法向 Q（结构固定，只改数值）
    buildGradient(xref, uref);                        // q
    buildBounds(x_init, turtle);                      // l, u

    if (!setup_done_) {
      if (!setup(nx)) {
        output_cache_.clear();
        return output_cache_;
      }
    } else {
      if (params_.path_frame_weighting) {
        osqp_update_P(work_, P_x_.data(), nullptr, static_cast<c_int>(P_x_.size()));
      }
      osqp_update_lin_cost(work_, q_.data());
      osqp_update_bounds(work_, l_.data(), u_.data());
    }

    const int solve_ret = osqp_solve(work_);
    const c_int status_val =
      (work_ != nullptr && work_->info != nullptr) ? work_->info->status_val : -1;
    if (solve_ret != 0 || status_val <= 0) {
      output_cache_.clear();
      return output_cache_;
    }
    // OSQP_SOLVED_INACCURATE 也是可用解：丢弃会让控制突然断拍，故沿用。
    // 但必须留痕——跟踪质量劣化排查时这是唯一线索。
    if (status_val == 2 /* OSQP_SOLVED_INACCURATE */) {
      ++inaccurate_solves_;
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

  // SOLVED_INACCURATE 累计次数。非零即说明 QP 在数值上吃紧，
  // 结合 PerformanceMonitor 的耗时一起看是否需要调权重或开 time_limit。
  uint64_t inaccurateSolves() const { return inaccurate_solves_; }

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
      // 路径坐标权重开启时预留 xy 非零结构；初值可为0，但结构必须在OSQP setup前存在。
      if (params_.path_frame_weighting) {
        add(2 * i, 2 * i + 1, 0.0);
      }
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
    state_p_xx_.assign(steps, -1);
    state_p_xy_.assign(steps, -1);
    state_p_yy_.assign(steps, -1);
    for (int col = 0; col < dimx; ++col) {
      for (c_int k = P_p_[col]; k < P_p_[col + 1]; ++k) {
        const int row = P_i_[k];
        const int step = col / 2;
        if (col == 2 * step && row == col) state_p_xx_[step] = k;
        if (col == 2 * step + 1 && row == 2 * step) state_p_xy_[step] = k;
        if (col == 2 * step + 1 && row == col) state_p_yy_[step] = k;
      }
    }
  }

  void buildPathFrameWeights(const Eigen::MatrixXd & xref, const Eigen::MatrixXd & uref)
  {
    const int steps = params_.steps;
    if (state_weights_.size() != static_cast<std::size_t>(steps)) {
      state_weights_.resize(steps, Eigen::Matrix2d::Zero());
    }
    Eigen::Vector2d last_tangent(1.0, 0.0);
    for (int i = 0; i < steps; ++i) {
      if (!params_.path_frame_weighting) {
        state_weights_[i] = Eigen::Vector2d(params_.Q[0], params_.Q[1]).asDiagonal();
        continue;
      }
      Eigen::Vector2d tangent = uref.col(i);
      if (tangent.norm() < params_.min_tangent_speed) {
        if (i + 1 < steps) tangent = xref.col(i + 1) - xref.col(i);
        if (tangent.norm() < 1e-9 && i > 0) tangent = xref.col(i) - xref.col(i - 1);
      }
      if (tangent.norm() >= 1e-9 && tangent.allFinite()) {
        last_tangent = tangent.normalized();
      }
      const Eigen::Vector2d normal(-last_tangent.y(), last_tangent.x());
      state_weights_[i] =
        params_.tangential_weight * (last_tangent * last_tangent.transpose()) +
        params_.lateral_weight * (normal * normal.transpose());
    }
  }

  void updateStateHessian()
  {
    for (int i = 1; i < params_.steps; ++i) {
      const auto & weight = state_weights_[i];
      if (state_p_xx_[i] >= 0) P_x_[state_p_xx_[i]] = 2.0 * weight(0, 0);
      if (state_p_xy_[i] >= 0) P_x_[state_p_xy_[i]] = 2.0 * weight(0, 1);
      if (state_p_yy_[i] >= 0) P_x_[state_p_yy_[i]] = 2.0 * weight(1, 1);
    }
  }

  void buildGradient(const Eigen::MatrixXd & xref, const Eigen::MatrixXd & uref)
  {
    // 把参考轨迹写进线性项。
    const int steps = params_.steps;
    const int dimx = 2 * steps;
    const int nx = dimx + 2 * steps;
    const auto & R = params_.R;
    q_.assign(nx, 0.0);
    for (int i = 0; i < steps; ++i) {
      const Eigen::Vector2d linear = -2.0 * state_weights_[i] * xref.col(i);
      q_[2 * i] = linear.x();
      q_[2 * i + 1] = linear.y();
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
    //         [模长对角 2*steps（仅 enforce_speed_norm）]
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
    // 模长收紧时的对角方向（±45°）。轴向 0°/90° 已由速度 box 行覆盖，这里补齐
    // 另外两个法向，凑成完整的内接八边形。结构在 configure() 一次建好，
    // 热路径只改 l/u。
    if (params_.enforce_speed_norm) {
      const int normoff = ncon_;
      constexpr double kInvSqrt2 = 0.70710678118654752440;
      for (int i = 0; i < steps; ++i) {
        const int r = normoff + 2 * i;
        const int c = dimx + 2 * i;
        add(r, c, kInvSqrt2); add(r, c + 1, kInvSqrt2);       // (ux+vy)/sqrt2
        add(r + 1, c, kInvSqrt2); add(r + 1, c + 1, -kInvSqrt2);  // (ux-vy)/sqrt2
      }
      ncon_ = normoff + 2 * steps;
    }
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
    const double vmax_raw = turtle ? params_.turtle_max_speed : params_.max_speed;
    // enforce_speed_norm 时轴向界收紧到内接八边形的支撑距离，使顶点恰好落在
    // 半径 vmax 的圆上；对角行用同一个界。见文件头注释的推导。
    const double vmax = params_.enforce_speed_norm ?
      vmax_raw * kOctagonSupport : vmax_raw;
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
    if (params_.enforce_speed_norm) {
      const int normoff = accoff + 2 * (steps - 1);
      for (int i = 0; i < steps; ++i) {
        const int r = normoff + 2 * i;
        l_[r] = -vmax; u_[r] = vmax;
        l_[r + 1] = -vmax; u_[r + 1] = vmax;
      }
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
    // 求解质量上调（用户授权用空闲 CPU 换解的最优性）：实测整机 busy<8%，
    // 30Hz QP 有的是余量。inaccurateSolves() 观察器与 PerformanceMonitor
    // 兜底，退化可观测、可回退。
    settings_->max_iter = 6000;
    settings_->eps_abs = 3e-5;
    settings_->eps_rel = 3e-5;
    if (params_.time_limit > 0.0) {
      settings_->time_limit = static_cast<c_float>(params_.time_limit);
    }

    if (osqp_setup(&work_, data_, settings_) != 0) {
      // osqp_setup 失败：OSQP 自行清理半成品 workspace，但 data_/settings_ 和
      // 两个 csc 结构体是我们分配的，必须在这里释放——否则每次重试（configure
      // 后再次 solve）都会泄漏两份内存。
      if (P_csc_) { c_free(P_csc_); P_csc_ = nullptr; }
      if (A_csc_) { c_free(A_csc_); A_csc_ = nullptr; }
      if (data_) { c_free(data_); data_ = nullptr; }
      if (settings_) { c_free(settings_); settings_ = nullptr; }
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
  std::vector<Eigen::Matrix2d> state_weights_;
  std::vector<c_int> state_p_xx_, state_p_xy_, state_p_yy_;
  std::vector<Eigen::Vector2d> output_cache_;
  uint64_t inaccurate_solves_ = 0;
};

}  // namespace navigation2::mpc

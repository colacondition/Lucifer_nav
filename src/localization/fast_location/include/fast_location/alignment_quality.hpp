#pragma once
// 定位结果质量评估。
//
// B（HWSentryNav26）的 odom_localizer 用五道验收门：收敛性、误差、重叠率、
// 信息矩阵正定性与条件数。A 原来只有收敛性 + 内点比（前三道）。
// 这里补上「条件数检查」（可观测性）：
//
//   在长走廊场景中，点云沿走廊方向几乎没有匹配约束，GICP 会给出一个在该
//   方向上不可信的平移修正。条件数 = 对齐点云 2D 分布的最大特征值 / 最小
//   特征值；走廊场景下最小特征值趋近于零，条件数极大，说明对准结果在某个
//   方向上是退化的，应当拒绝。
//
// 信息来源：直接取 fast_gicp 收敛时的 6×6 Hessian（`getFinalHessian()`），
// 其平移子块 H.block<3,3>(3,3) 就是平移方向上的信息矩阵。这比用点云空间
// 分布做代理更准确——空间分布只反映「点长在哪」，信息矩阵反映的是「匹配
// 约束在哪个方向真正存在」，两者在有面法向分布不均的场景下差别很大。
//
// 退化时的处理：不整拍丢弃，而是把修正量投影到可观测子空间（见
// projectToObservableSubspace）。fast_location 是 Lucifer 唯一的 map→odom
// 来源，整拍拒绝会让 TF 冻结、精度完全交给 LIO 开环漂移；部分更新则保留
// 了强约束方向上的修正，只在退化方向信任 LIO 推算。
#include <algorithm>
#include <cstdint>
#include <cmath>

#include <Eigen/Dense>
#include <pcl/search/kdtree.h>

#include "fast_location/global_search.hpp"

namespace fast_location
{

// ========== 6DoF 可观测性：平移 / 旋转 Schur 谱 ==========
//
// 2D 条件数只看 H_tt 的 XY 子块，丢了旋转与平移的耦合，也看不到 roll/pitch。
// 这里对 fast_gicp 的 6×6 Hessian（状态序 [rot(0:3), trans(3:6)]）做分块 Schur
// 补：
//
//   S_tt = H_tt - H_tr H_rr^{-1} H_rt   （扣掉旋转后，平移还剩下多少信息）
//   S_rr = H_rr - H_rt H_tt^{-1} H_tr   （扣掉平移后，旋转还剩下多少信息）
//
// Schur 补就是"边缘化另一组变量之后的信息矩阵"，其特征谱直接回答"哪个自由度
// 真的被观测约束住了"：走廊里沿走廊平移 λ→0，而 yaw 仍被两侧墙面约束；某个
// 方向完全退化的扫描则连旋转 Schur 也掉秩。rank 用相对阈值 λ_i > ratio·λ_max
// 判定，避免依赖点数量纲（绝对特征值随点数/体素变化，相对谱不随）。
struct SixDofObservability
{
  bool valid{false};
  // 平移到平移、旋转到旋转的 Schur 信息矩阵（3×3，已对称化）。
  Eigen::Matrix3d translation_schur{Eigen::Matrix3d::Zero()};
  Eigen::Matrix3d rotation_schur{Eigen::Matrix3d::Zero()};
  // 特征值升序（[0]=λ_min，[2]=λ_max）。数值上 -1e-15 级的负特征值已夹到 0。
  Eigen::Vector3d translation_eigenvalues{Eigen::Vector3d::Zero()};
  Eigen::Vector3d rotation_eigenvalues{Eigen::Vector3d::Zero()};
  // 弱方向 = λ_min 对应的单位特征向量（退化方向）。平移分量为 (x, y, z)，
  // 旋转分量为 (roll, pitch, yaw)（与 fast_gicp 状态序一致）。
  Eigen::Vector3d translation_weak_direction{Eigen::Vector3d::UnitX()};
  Eigen::Vector3d rotation_weak_direction{Eigen::Vector3d::UnitX()};
  // λ_max / λ_min。λ_max<=0（该块完全没有信息）时为 -1；λ_min 触底封顶 1e12。
  double translation_condition{-1.0};
  double rotation_condition{-1.0};
  // 观测秩：λ_i > relative_eigen_threshold · λ_max（且 λ_max > 0）。
  int translation_rank{0};
  int rotation_rank{0};
  // yaw（fast_gicp 旋转状态 index 2）方向的 Schur 后信息量及其是否可观测。
  double yaw_information{0.0};
  bool yaw_observable{false};
  double relative_eigen_threshold{1e-3};
};

namespace detail
{
// 3×3 对称矩阵求逆：LDLT + 相对对角正则（尺度 × 1e-9）。
// 尺度为零（该子块完全没有信息）或正则后仍非正定（数值奇异）时返回 false，
// 由调用方把整体标记为 invalid —— 不抛异常，定位线程里不能因为退化炸掉。
inline bool invertSymmetric3(
  const Eigen::Matrix3d & matrix, Eigen::Matrix3d & inverse,
  double relative_regularization = 1e-9)
{
  inverse.setZero();
  if (!matrix.allFinite()) {
    return false;
  }
  const Eigen::Matrix3d symmetric = 0.5 * (matrix + matrix.transpose());
  // 用全元素最大幅值做尺度（对角元可能全为零而耦合项非零）。
  const double scale = symmetric.cwiseAbs().maxCoeff();
  if (!(scale > 0.0) || !std::isfinite(scale)) {
    return false;  // 全零/非有限：Schur 补无定义。
  }
  Eigen::Matrix3d regularized = symmetric;
  regularized.diagonal().array() += scale * relative_regularization;
  const Eigen::LDLT<Eigen::Matrix3d> ldlt(regularized);
  if (ldlt.info() != Eigen::Success || !ldlt.vectorD().allFinite()) {
    return false;
  }
  // 正则后仍有非正主元 => 矩阵远非正定（强 indefiniteness），不硬算。
  if ((ldlt.vectorD().array() <= 0.0).any()) {
    return false;
  }
  inverse = ldlt.solve(Eigen::Matrix3d::Identity());
  return inverse.allFinite();
}

// 对单个 Schur 块做特征分解，填特征值（升序）、弱方向、条件数、观测秩。
inline void analyzeSchurBlock(
  const Eigen::Matrix3d & schur,
  double relative_eigen_threshold,
  Eigen::Vector3d & eigenvalues,
  Eigen::Vector3d & weak_direction,
  double & condition,
  int & rank)
{
  eigenvalues.setZero();
  weak_direction = Eigen::Vector3d::UnitX();
  condition = -1.0;
  rank = 0;
  if (!schur.allFinite()) {
    return;
  }
  const Eigen::Matrix3d symmetric = 0.5 * (schur + schur.transpose());
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(symmetric);
  if (solver.info() != Eigen::Success) {
    return;
  }
  eigenvalues = solver.eigenvalues().cwiseMax(0.0);
  const Eigen::Vector3d eigenvector =
    solver.eigenvectors().col(0).normalized().eval();
  if (eigenvector.allFinite()) {
    weak_direction = eigenvector;
  }

  const double lambda_max = eigenvalues(2);
  if (!(lambda_max > 0.0) || !std::isfinite(lambda_max)) {
    return;  // 整块没有信息：condition/rank 保持 -1/0。
  }
  const double lambda_min = eigenvalues(0);
  condition = lambda_max / std::max(lambda_min, lambda_max * 1e-12);
  condition = std::min(condition, 1e12);

  const double threshold = relative_eigen_threshold * lambda_max;
  for (int i = 0; i < 3; ++i) {
    if (eigenvalues(i) > threshold) {
      ++rank;
    }
  }
}
}  // namespace detail

// 分析 6×6 Hessian 的 3D 平移/旋转可观测性。
// 返回 valid=false 表示 Hessian 非有限、子块求逆失败或 Schur 补非有限。
inline SixDofObservability analyzeSixDofObservability(
  const Eigen::Matrix<double, 6, 6> & hessian,
  double relative_eigen_threshold = 1e-3)
{
  SixDofObservability out;
  if (!hessian.allFinite()) {
    return out;
  }
  if (!(relative_eigen_threshold > 0.0) || !std::isfinite(relative_eigen_threshold)) {
    relative_eigen_threshold = 1e-3;
  }
  out.relative_eigen_threshold = relative_eigen_threshold;

  const Eigen::Matrix<double, 6, 6> h = 0.5 * (hessian + hessian.transpose());
  // FastGICP 状态序 [rot(0:3), trans(3:6)]。
  const Eigen::Matrix3d h_rr = h.block<3, 3>(0, 0);
  const Eigen::Matrix3d h_rt = h.block<3, 3>(0, 3);
  const Eigen::Matrix3d h_tr = h.block<3, 3>(3, 0);
  const Eigen::Matrix3d h_tt = h.block<3, 3>(3, 3);

  Eigen::Matrix3d h_rr_inv;
  Eigen::Matrix3d h_tt_inv;
  if (!detail::invertSymmetric3(h_rr, h_rr_inv) ||
    !detail::invertSymmetric3(h_tt, h_tt_inv))
  {
    return out;
  }

  out.translation_schur = h_tt - h_tr * h_rr_inv * h_rt;
  out.rotation_schur = h_rr - h_rt * h_tt_inv * h_tr;
  out.translation_schur =
    0.5 * (out.translation_schur + out.translation_schur.transpose());
  out.rotation_schur =
    0.5 * (out.rotation_schur + out.rotation_schur.transpose());
  if (!out.translation_schur.allFinite() || !out.rotation_schur.allFinite()) {
    return out;
  }

  detail::analyzeSchurBlock(
    out.translation_schur, relative_eigen_threshold,
    out.translation_eigenvalues, out.translation_weak_direction,
    out.translation_condition, out.translation_rank);
  detail::analyzeSchurBlock(
    out.rotation_schur, relative_eigen_threshold,
    out.rotation_eigenvalues, out.rotation_weak_direction,
    out.rotation_condition, out.rotation_rank);

  out.valid = true;
  // yaw = fast_gicp 旋转状态 index 2（roll, pitch, yaw）。
  out.yaw_information = out.rotation_schur(2, 2);
  const double rotation_lambda_max = out.rotation_eigenvalues(2);
  out.yaw_observable = rotation_lambda_max > 0.0 &&
    out.yaw_information > relative_eigen_threshold * rotation_lambda_max;
  return out;
}

// 首配/全局重定位位姿门：平移秩≥2（至少约束住平面内两个方向）且 yaw 可观测。
// 只有平移 rank≥3 才叫严格 6DoF 完整；平面机器人放宽到 2（z 不参与）。
inline bool isFullPoseObservable(
  const SixDofObservability & observability, int min_translation_rank = 2)
{
  if (!observability.valid) {
    return false;
  }
  const int required_rank = std::max(2, std::min(3, min_translation_rank));
  return observability.translation_rank >= required_rank &&
         observability.yaw_observable;
}

struct AlignmentAssessment
{
  bool solver_converged{false};
  bool usable{false};
  float inlier_ratio{0.0f};
  // 平移信息矩阵 XY 子块的条件数 = λ_max / λ_min。值越大越退化。
  // 无法计算时（点数不足 / 无 Hessian）设为 -1。
  float condition_number{-1.0f};
  // 是否在某个方向退化（条件数 > 阈值）。
  bool is_degenerate{false};
  // 退化方向的单位向量（XY 平面内，对应 λ_min 的特征向量）。
  // 仅当 is_degenerate 为真时有意义。
  Eigen::Vector2f degenerate_direction{Eigen::Vector2f::Zero()};
  // 可观测方向的单位向量（对应 λ_max），与 degenerate_direction 正交。
  Eigen::Vector2f observable_direction{Eigen::Vector2f::Zero()};
  // 6DoF Schur 可观测性。只有拿到 Hessian 时才 valid；无 Hessian 的兜底路径
  // 仍只靠上面的 2D 字段，six_dof.valid=false。
  SixDofObservability six_dof;
  // 首配/全局重定位位姿门：平移秩≥2 且 yaw 可观测（见 isFullPoseObservable）。
  bool full_pose_observable{false};
};

// 2×2 对称矩阵的特征分解（解析解）。返回 {λ_min, λ_max, v_min, v_max}。
struct Eigen2D
{
  double lambda_min{0.0};
  double lambda_max{0.0};
  Eigen::Vector2f v_min{Eigen::Vector2f::UnitX()};
  Eigen::Vector2f v_max{Eigen::Vector2f::UnitY()};
};

inline Eigen2D decompose2D(double cxx, double cxy, double cyy)
{
  Eigen2D result;
  const double trace = cxx + cyy;
  const double det = cxx * cyy - cxy * cxy;
  const double disc = std::sqrt(std::max(0.0, trace * trace * 0.25 - det));
  result.lambda_max = trace * 0.5 + disc;
  result.lambda_min = trace * 0.5 - disc;

  // 特征向量：对 (A - λI)v = 0，用 [cxy, λ-cxx] 或 [λ-cyy, cxy]，取模长大的那个
  // 以避开 cxy≈0（矩阵已对角化）时的退化表达。
  const auto eigenvector = [cxx, cxy, cyy](double lambda) {
    Eigen::Vector2d v1(cxy, lambda - cxx);
    Eigen::Vector2d v2(lambda - cyy, cxy);
    Eigen::Vector2d v = (v1.squaredNorm() >= v2.squaredNorm()) ? v1 : v2;
    if (v.squaredNorm() < 1e-20) {
      // 各向同性矩阵，任意正交基都成立。
      return Eigen::Vector2d(1.0, 0.0);
    }
    return v.normalized().eval();
  };

  const Eigen::Vector2d vmax = eigenvector(result.lambda_max);
  result.v_max = vmax.cast<float>();
  // 2D 对称矩阵的两个特征向量必然正交，直接旋转 90° 比再解一次稳。
  result.v_min = Eigen::Vector2f(-result.v_max.y(), result.v_max.x());
  return result;
}

// 从 GICP 的 6×6 Hessian 提取 XY 平移方向的可观测性。
// fast_gicp 的状态排序是 [rot(0:3), trans(3:6)]，所以平移信息矩阵是
// H.block<3,3>(3,3)，再取其 XY 子块（平面机器人只关心 XY 退化）。
//
// 特征值量纲是「信息」（1/方差），λ_min 小意味着该方向的位置估计方差大，
// 即约束弱。返回条件数 λ_max/λ_min，并输出两个主方向。
inline float computeHessianConditionNumber(
  const Eigen::Matrix<double, 6, 6> & hessian,
  Eigen::Vector2f & observable_direction,
  Eigen::Vector2f & degenerate_direction)
{
  if (!hessian.allFinite()) {
    return -1.0f;
  }

  // 对称化，消掉数值累加带来的不对称。
  const Eigen::Matrix<double, 6, 6> h_sym = 0.5 * (hessian + hessian.transpose());
  const double cxx = h_sym(3, 3);
  const double cxy = h_sym(3, 4);
  const double cyy = h_sym(4, 4);

  // 信息矩阵对角元必须为正；退化到零说明这个方向完全没有约束。
  if (!(cxx > 0.0) || !(cyy > 0.0)) {
    return -1.0f;
  }

  const Eigen2D decomp = decompose2D(cxx, cxy, cyy);
  observable_direction = decomp.v_max;
  degenerate_direction = decomp.v_min;

  // 用相对阈值：λ_min 相对 λ_max 太小就当作完全退化，避免绝对量纲依赖
  // 点数和体素尺寸。
  if (decomp.lambda_min <= 0.0 || decomp.lambda_min < decomp.lambda_max * 1e-6) {
    return 1e6f;
  }
  return static_cast<float>(decomp.lambda_max / decomp.lambda_min);
}

// 计算对齐点云 XY 平面 2D 协方差矩阵的条件数。
// 退化方向（长走廊）会让最小特征值接近零，条件数显著增大。
//
// 这是 Hessian 不可用时的兜底路径（例如 GICP 未产出有效 Hessian）。
//
// 关键：它的方向语义与信息矩阵**相反**。走廊沿 x 延伸时，点云在 x 方向铺得
// 最开（分布 λ_max 沿 x），但沿 x 平移并不改变匹配残差，所以 x 恰是不可观测
// 方向。因此分布的 v_max → 退化方向，v_min → 可观测方向；而信息矩阵是
// v_max → 可观测，v_min → 退化。条件数的大小关系两者一致（走廊下都偏大），
// 所以同一个阈值仍然适用，但方向向量必须对调。
inline float compute2DConditionNumber(
  const PointCloudXYZI::ConstPtr & cloud,
  Eigen::Vector2f * observable_direction = nullptr,
  Eigen::Vector2f * degenerate_direction = nullptr)
{
  if (!cloud || cloud->size() < 10) {
    return -1.0f;
  }

  // 只统计有效点，并用有效点数做归一化——用 cloud->size() 会在存在 NaN 点
  // 时把均值和协方差同时压小。
  double mx = 0.0, my = 0.0;
  std::size_t valid = 0;
  for (const auto & p : cloud->points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
      continue;
    }
    mx += p.x;
    my += p.y;
    ++valid;
  }
  if (valid < 10) {
    return -1.0f;
  }
  mx /= static_cast<double>(valid);
  my /= static_cast<double>(valid);

  double cxx = 0.0, cxy = 0.0, cyy = 0.0;
  for (const auto & p : cloud->points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
      continue;
    }
    const double dx = p.x - mx;
    const double dy = p.y - my;
    cxx += dx * dx;
    cxy += dx * dy;
    cyy += dy * dy;
  }
  cxx /= static_cast<double>(valid);
  cxy /= static_cast<double>(valid);
  cyy /= static_cast<double>(valid);

  const Eigen2D decomp = decompose2D(cxx, cxy, cyy);
  // 方向语义对调：分布最开的方向 (v_max) 是不可观测的那个。
  if (observable_direction) {
    *observable_direction = decomp.v_min;
  }
  if (degenerate_direction) {
    *degenerate_direction = decomp.v_max;
  }

  if (decomp.lambda_min < 1e-9) {
    return 1e6f;    // 最小特征值为零 → 完全退化
  }
  return static_cast<float>(decomp.lambda_max / decomp.lambda_min);
}

// hessian 为 nullptr 时退回点云分布近似（见 compute2DConditionNumber）。
// relative_eigen_threshold 是 6DoF 观测秩的相对特征值阈值（λ_i > ratio·λ_max）。
inline AlignmentAssessment assessAlignment(
  bool solver_converged,
  const PointCloudXYZI::ConstPtr & aligned,
  const PointCloudXYZI::ConstPtr & target,
  float max_correspondence_distance,
  float degenerate_condition_threshold = 100.0f,
  const Eigen::Matrix<double, 6, 6> * hessian = nullptr,
  double relative_eigen_threshold = 1e-3)
{
  AlignmentAssessment assessment;
  assessment.solver_converged = solver_converged;
  if (!aligned || aligned->empty() || !target || target->empty() ||
    !std::isfinite(max_correspondence_distance) || max_correspondence_distance <= 0.0f)
  {
    return assessment;
  }

  pcl::search::KdTree<Point> kdtree;
  kdtree.setInputCloud(target);
  const float threshold_sq = max_correspondence_distance * max_correspondence_distance;
  std::vector<int> indices(1);
  std::vector<float> distances_sq(1);
  std::size_t inlier_count = 0;

  for (const auto & point : aligned->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    if (kdtree.nearestKSearch(point, 1, indices, distances_sq) > 0 &&
      distances_sq[0] < threshold_sq)
    {
      ++inlier_count;
    }
  }

  assessment.usable = true;
  assessment.inlier_ratio =
    static_cast<float>(inlier_count) / static_cast<float>(aligned->size());

  // 可观测性检查：优先用 GICP 的真实信息矩阵，取不到再退回点云分布。
  if (hessian) {
    assessment.condition_number = computeHessianConditionNumber(
      *hessian, assessment.observable_direction, assessment.degenerate_direction);
    // 6DoF Schur 谱与首配/重定位位姿门的数据来源。与上面的 2D 条件数并行，
    // 不改变 is_degenerate / 投影逻辑的语义。
    assessment.six_dof = analyzeSixDofObservability(*hessian, relative_eigen_threshold);
    assessment.full_pose_observable = isFullPoseObservable(assessment.six_dof);
  }
  if (assessment.condition_number <= 0.0f) {
    assessment.condition_number = compute2DConditionNumber(
      aligned, &assessment.observable_direction, &assessment.degenerate_direction);
  }
  if (assessment.condition_number > 0.0f) {
    assessment.is_degenerate = (assessment.condition_number > degenerate_condition_threshold);
  }

  return assessment;
}

// 把 GICP 的修正量投影到可观测子空间。
//
// 退化时（长走廊）GICP 在退化方向上给出的平移修正是噪声，但垂直方向的修正
// 仍然可信。整拍拒绝会让 map→odom 冻结；这里只保留可观测方向的平移分量，
// 退化方向沿用初值（即信任 LIO 推算）。
//
// 旋转分量保持原样：走廊场景退化的是沿墙平移，yaw 由两侧墙面法向约束得很好。
// 若条件数极端（> 1e5，近乎完全无约束），旋转也不可信，此时应整拍拒绝，
// 由调用方通过 reject_threshold 判定。
inline Eigen::Matrix4f projectToObservableSubspace(
  const Eigen::Matrix4f & corrected,
  const Eigen::Matrix4f & initial_guess,
  const Eigen::Vector2f & observable_direction)
{
  if (observable_direction.squaredNorm() < 1e-12f) {
    return initial_guess;
  }
  const Eigen::Vector2f u = observable_direction.normalized();

  // XY 平面内的修正增量。
  const Eigen::Vector2f delta_xy =
    corrected.block<2, 1>(0, 3) - initial_guess.block<2, 1>(0, 3);
  // 只保留 u 方向上的投影分量。
  const Eigen::Vector2f delta_projected = u * (u.dot(delta_xy));

  Eigen::Matrix4f result = corrected;
  result.block<2, 1>(0, 3) = initial_guess.block<2, 1>(0, 3) + delta_projected;
  // z 同样交回初值：退化通常伴随高度约束不足，且平面机器人不需要 GICP 修 z。
  result(2, 3) = initial_guess(2, 3);
  return result;
}

// 完整性对外两态。只告诉下游当前 map→odom 锁还能不能信。
// 本拍能不能写 TF、走廊投影、NIS 连败，都在定位节点内部做完再贴 reason。
// 不改 LIO 进程 / 线程预算。
enum class IntegrityState : std::uint8_t
{
  OK = 0,
  LOST = 1,
};

inline const char * integrityStateName(IntegrityState state)
{
  switch (state) {
    case IntegrityState::OK: return "OK";
    case IntegrityState::LOST: return "LOST";
  }
  return "UNKNOWN";
}

// ========== χ² 上尾概率（自实现，不引 boost） ==========
//
// NIS 判拒需要"在 H0（观测与 map→odom 一致）下，看到这么大 NIS 的概率"。
// 卡方分布上尾 Q(k/2, x/2) 由正则化上不完全 Gamma 函数给出：
//
//   Q(a, x) = Γ(a, x) / Γ(a)
//   级数展开求 P(a, x)（x < a+1 时收敛快），连分式求 Q(a, x)（x >= a+1），
//   两者互补，避免大 x 时 1-P 的灾难性相消。dof∈[1,6] 足够（平移 XY + yaw 最多
//   3 维，Schur 分块分析用到 3 维；留一点余量）。
namespace detail
{
// 正则化下不完全 Gamma P(a, x) 的级数展开（Numerical Recipes gser）。
inline double regularizedGammaPSeries(double a, double x)
{
  constexpr int kMaxIterations = 500;
  constexpr double kEpsilon = 1e-14;
  double ap = a;
  double sum = 1.0 / a;
  double del = sum;
  for (int n = 0; n < kMaxIterations; ++n) {
    ++ap;
    del *= x / ap;
    sum += del;
    if (std::fabs(del) < std::fabs(sum) * kEpsilon) {
      break;
    }
  }
  return sum * std::exp(-x + a * std::log(x) - std::lgamma(a));
}

// 正则化上不完全 Gamma Q(a, x) 的连分式（Numerical Recipes gcf）。
inline double regularizedGammaQContinuedFraction(double a, double x)
{
  constexpr int kMaxIterations = 500;
  constexpr double kEpsilon = 1e-14;
  constexpr double kTiny = 1e-300;
  const double gln = std::lgamma(a);
  double b = x + 1.0 - a;
  double c = 1.0 / kTiny;
  double d = 1.0 / b;
  double h = d;
  for (int i = 1; i <= kMaxIterations; ++i) {
    const double an = -static_cast<double>(i) * (static_cast<double>(i) - a);
    b += 2.0;
    d = an * d + b;
    if (std::fabs(d) < kTiny) {
      d = kTiny;
    }
    c = b + an / c;
    if (std::fabs(c) < kTiny) {
      c = kTiny;
    }
    d = 1.0 / d;
    const double delta = d * c;
    h *= delta;
    if (std::fabs(delta - 1.0) < kEpsilon) {
      break;
    }
  }
  return std::exp(-x + a * std::log(x) - gln) * h;
}
}  // namespace detail

// 正则化上不完全 Gamma Q(a, x)，a>0、x>=0。返回 [0, 1]。
inline double regularizedUpperIncompleteGamma(double a, double x)
{
  if (!(a > 0.0) || !std::isfinite(a) || !std::isfinite(x)) {
    return -1.0;
  }
  if (x <= 0.0) {
    return 1.0;
  }
  double q = 0.0;
  if (x < a + 1.0) {
    q = 1.0 - detail::regularizedGammaPSeries(a, x);
  } else {
    q = detail::regularizedGammaQContinuedFraction(a, x);
  }
  if (!std::isfinite(q)) {
    return -1.0;
  }
  return std::clamp(q, 0.0, 1.0);
}

// 卡方分布上尾概率 P(χ²_dof > x) = Q(dof/2, x/2)。
// x<=0 返回 1；dof<=0 或非有限输入返回 -1（调用方视为"未计算"）。
inline double chiSquareUpperTail(double x, int dof)
{
  if (dof <= 0 || dof > 6 || !std::isfinite(x)) {
    return -1.0;
  }
  if (x <= 0.0) {
    return 1.0;
  }
  return regularizedUpperIncompleteGamma(0.5 * static_cast<double>(dof), 0.5 * x);
}

// 反解 χ² 阈值：返回 x 使上尾概率约等于 tail_probability。
// 单调下降，用倍增找上界 + 二分；dof<=0 或 tail_probability∉(0,1) 返回 -1。
inline double chiSquareThreshold(int dof, double tail_probability)
{
  if (dof <= 0 || dof > 6 || !(tail_probability > 0.0) || !(tail_probability < 1.0)) {
    return -1.0;
  }
  double low = 0.0;
  double high = std::max(1.0, 2.0 * static_cast<double>(dof));
  int expansions = 0;
  while (chiSquareUpperTail(high, dof) > tail_probability && high < 1e8 &&
    expansions < 60)
  {
    high *= 2.0;
    ++expansions;
  }
  for (int i = 0; i < 200; ++i) {
    const double mid = 0.5 * (low + high);
    const double tail = chiSquareUpperTail(mid, dof);
    if (!(tail >= 0.0)) {
      return -1.0;
    }
    if (tail > tail_probability) {
      low = mid;
    } else {
      high = mid;
    }
    if (high - low <= 1e-10 * std::max(1.0, high)) {
      break;
    }
  }
  return 0.5 * (low + high);
}

struct ScanToMapNis
{
  bool valid{false};
  double value{-1.0};
  Eigen::Vector3d innovation{Eigen::Vector3d::Zero()};  // dx, dy, dyaw
};

// 投影/白化 NIS 的结果：dof = 观测秩，upper_tail = χ² 上尾概率。
struct ProjectedScanToMapNis
{
  bool valid{false};
  double value{-1.0};
  // 可观测子空间的维度（= 参与判拒的自由度），退化方向不计入。
  int dof{0};
  int observable_rank{0};
  // P(χ²_dof > value)，落在 [0,1]；值越小越不像一致观测。
  double upper_tail{1.0};
  Eigen::Vector3d innovation{Eigen::Vector3d::Zero()};  // dx, dy, dyaw
};

inline double wrapAnglePi(double angle)
{
  constexpr double kPi = 3.14159265358979323846;
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

// 观测一致性（NIS）。Hessian 只能说明“这次观测有没有信息”，
// 不能说明“观测和当前 map→odom 是否统计一致”。
//
// 创新 ν = (Δx, Δy, Δyaw)；测量协方差 R = H3^{-1}（FastGICP 6×6 的
// [yaw, tx, ty] 子块）；当前位姿先验 P 取对角。S = R + P，NIS = ν^T S^{-1} ν。
// 3 自由度 χ² 99% 约 11.34，默认门限 12。
inline ScanToMapNis computeScanToMapNis(
  const Eigen::Matrix4f & pose_guess,
  const Eigen::Matrix4f & pose_measured,
  const Eigen::Matrix<double, 6, 6> & hessian,
  double prior_xy_std,
  double prior_yaw_std)
{
  ScanToMapNis out;
  if (!hessian.allFinite() || !pose_guess.allFinite() || !pose_measured.allFinite() ||
    !(prior_xy_std > 0.0) || !(prior_yaw_std > 0.0))
  {
    return out;
  }

  const Eigen::Matrix<double, 6, 6> H = 0.5 * (hessian + hessian.transpose());
  // FastGICP 状态序 [rot(0:3), trans(3:6)]；yaw 近似 rz = index 2。
  Eigen::Matrix3d H3;
  H3 << H(3, 3), H(3, 4), H(3, 2),
        H(4, 3), H(4, 4), H(4, 2),
        H(2, 3), H(2, 4), H(2, 2);
  if (!H3.allFinite()) {
    return out;
  }

  const Eigen::LDLT<Eigen::Matrix3d> ldlt(H3);
  if (ldlt.info() != Eigen::Success || (ldlt.vectorD().array() <= 1e-12).any()) {
    return out;
  }
  const Eigen::Matrix3d R = ldlt.solve(Eigen::Matrix3d::Identity());
  if (!R.allFinite()) {
    return out;
  }

  Eigen::Matrix3d P = Eigen::Matrix3d::Zero();
  P(0, 0) = prior_xy_std * prior_xy_std;
  P(1, 1) = prior_xy_std * prior_xy_std;
  P(2, 2) = prior_yaw_std * prior_yaw_std;
  const Eigen::Matrix3d S = R + P;
  const Eigen::LDLT<Eigen::Matrix3d> s_ldlt(S);
  if (s_ldlt.info() != Eigen::Success || (s_ldlt.vectorD().array() <= 1e-18).any()) {
    return out;
  }

  Eigen::Vector3d nu;
  nu << static_cast<double>(pose_measured(0, 3) - pose_guess(0, 3)),
        static_cast<double>(pose_measured(1, 3) - pose_guess(1, 3)),
        wrapAnglePi(
          std::atan2(static_cast<double>(pose_measured(1, 0)), static_cast<double>(pose_measured(0, 0))) -
          std::atan2(static_cast<double>(pose_guess(1, 0)), static_cast<double>(pose_guess(0, 0))));
  out.innovation = nu;
  out.value = nu.dot(s_ldlt.solve(nu));
  out.valid = std::isfinite(out.value);
  return out;
}

// 投影/白化 NIS：退化时不再整拍跳过一致性检验。
//
// 旧 computeScanToMapNis 用 H3 的完整逆当观测协方差；H3 在退化方向近乎奇异，
// 逆会爆炸，于是旧逻辑只能在 is_degenerate 时跳过 NIS —— 走廊里等于放弃了一致性
// 判据。这里改为：
//
//   1) H3（[dx, dy, dyaw]）做特征分解，只保留 λ_i > observable_ratio·λ_max 的
//      可观测方向（R = Σ 1/λ_i u_i u_iᵀ 的截断伪逆）；
//   2) 创新 ν 与先验 P 都投影到同一可观测子空间，S = R + P，NIS = νᵀ S⁻¹ ν；
//   3) dof = 可观测秩，按 χ² 上尾概率判拒（退化方向既不参与白化也不参与判拒）。
//
// 这样走廊里 yaw 与垂直走廊的平移仍被检验，只有沿走廊的平移创新被忽略——
// 那正是 GICP 本身不可信的方向。
inline ProjectedScanToMapNis computeScanToMapNisProjected(
  const Eigen::Matrix4f & pose_guess,
  const Eigen::Matrix4f & pose_measured,
  const Eigen::Matrix<double, 6, 6> & hessian,
  double prior_xy_std,
  double prior_yaw_std,
  double observable_ratio = 1e-3)
{
  ProjectedScanToMapNis out;
  if (!hessian.allFinite() || !pose_guess.allFinite() || !pose_measured.allFinite() ||
    !(prior_xy_std > 0.0) || !(prior_yaw_std > 0.0) ||
    !(observable_ratio > 0.0) || !(observable_ratio < 1.0))
  {
    return out;
  }

  const Eigen::Matrix<double, 6, 6> H = 0.5 * (hessian + hessian.transpose());
  // FastGICP 状态序 [rot(0:3), trans(3:6)]；H3 的行列序是 [dx, dy, dyaw]，
  // 对应 Hessian 的 [3, 4, 2]（与原 computeScanToMapNis 完全一致）。
  Eigen::Matrix3d H3;
  H3 << H(3, 3), H(3, 4), H(3, 2),
        H(4, 3), H(4, 4), H(4, 2),
        H(2, 3), H(2, 4), H(2, 2);
  if (!H3.allFinite()) {
    return out;
  }

  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(H3);
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite() ||
    !solver.eigenvectors().allFinite())
  {
    return out;
  }
  const Eigen::Vector3d eigenvalues = solver.eigenvalues();
  const Eigen::Matrix3d eigenvectors = solver.eigenvectors();
  const double lambda_max = eigenvalues(2);
  if (!(lambda_max > 0.0) || !std::isfinite(lambda_max)) {
    return out;  // 三个方向都没有约束：没有可检验的子空间。
  }

  Eigen::Matrix3d P = Eigen::Matrix3d::Zero();
  P(0, 0) = prior_xy_std * prior_xy_std;
  P(1, 1) = prior_xy_std * prior_xy_std;
  P(2, 2) = prior_yaw_std * prior_yaw_std;

  Eigen::Vector3d nu;
  nu << static_cast<double>(pose_measured(0, 3) - pose_guess(0, 3)),
        static_cast<double>(pose_measured(1, 3) - pose_guess(1, 3)),
        wrapAnglePi(
          std::atan2(static_cast<double>(pose_measured(1, 0)), static_cast<double>(pose_measured(0, 0))) -
          std::atan2(static_cast<double>(pose_guess(1, 0)), static_cast<double>(pose_guess(0, 0))));
  out.innovation = nu;

  // 换到 H3 的特征基：此时 R 是对角 1/λ_i、P 是 UᵀPU。
  Eigen::Vector3d nu_eigen = eigenvectors.transpose() * nu;
  Eigen::Matrix3d S = eigenvectors.transpose() * P * eigenvectors;

  const double threshold = observable_ratio * lambda_max;
  int rank = 0;
  for (int i = 0; i < 3; ++i) {
    if (eigenvalues(i) <= threshold || !(eigenvalues(i) > 0.0)) {
      // 退化方向：从检验里剔除，同时把该轴创新清零（不参与判拒）。
      S.row(i).setZero();
      S.col(i).setZero();
      S(i, i) = 1.0;
      nu_eigen(i) = 0.0;
      continue;
    }
    S(i, i) += 1.0 / eigenvalues(i);  // R_i = 1/λ_i（观测协方差）
    ++rank;
  }
  if (rank <= 0) {
    return out;  // 连一个可观测方向都没有，退化为"不判拒"。
  }

  const Eigen::LDLT<Eigen::Matrix3d> s_ldlt(S);
  if (s_ldlt.info() != Eigen::Success ||
    (s_ldlt.vectorD().array() <= 1e-18).any())
  {
    return out;
  }
  out.value = nu_eigen.dot(s_ldlt.solve(nu_eigen));
  if (!std::isfinite(out.value) || out.value < 0.0) {
    return out;
  }
  out.observable_rank = rank;
  out.dof = rank;
  out.upper_tail = chiSquareUpperTail(out.value, rank);
  out.valid = std::isfinite(out.upper_tail) && out.upper_tail >= 0.0;
  return out;
}

}  // namespace fast_location

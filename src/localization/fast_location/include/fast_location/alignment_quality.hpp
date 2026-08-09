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
#include <cmath>

#include <Eigen/Dense>
#include <pcl/search/kdtree.h>

#include "fast_location/global_search.hpp"

namespace fast_location
{

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
inline AlignmentAssessment assessAlignment(
  bool solver_converged,
  const PointCloudXYZI::ConstPtr & aligned,
  const PointCloudXYZI::ConstPtr & target,
  float max_correspondence_distance,
  float degenerate_condition_threshold = 100.0f,
  const Eigen::Matrix<double, 6, 6> * hessian = nullptr)
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

}  // namespace fast_location

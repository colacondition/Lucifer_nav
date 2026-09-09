#pragma once

#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include <Eigen/Core>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace fast_location
{

using Point = pcl::PointXYZI;
using PointCloudXYZI = pcl::PointCloud<Point>;

// 用来描述全局搜索可覆盖的平面范围。
struct PlanarBounds
{
  float min_x{0.0f};
  float max_x{0.0f};
  float min_y{0.0f};
  float max_y{0.0f};
};

// 全局搜索的采样和打分参数。
struct GlobalSearchConfig
{
  float xy_step{2.0f};
  float yaw_step{0.5235988f};
  // 候选数量硬上限：地图有飞点/边界异常时，xy 网格 × yaw 网格会让候选爆炸，
  // 一次全局重定位就可能 OOM 或阻塞数分钟。超限时自动倍增 xy_step 降采样。
  std::size_t max_candidates{2000000};
  float score_distance{0.45f};
  std::size_t score_stride{4};
  std::size_t top_k{6};
  float minimum_score{0.20f};
  float minimum_score_margin{0.03f};
  float minimum_candidate_separation{1.0f};
  float minimum_candidate_yaw_separation{0.35f};
  // 两候选命中率之差小于该值时,视为"打平",改用平均残差(mean_error)决胜。
  // 对称场地(如 RMUL)两侧命中率几乎相同,残差是唯一的区分信号。
  float score_tie_epsilon{0.02f};
  // 只在 top-k 入选候选上再做一遍精细打分:stride=1、距离更严,几乎零额外开销。
  // <=0 表示沿用粗搜的 score_distance。
  float refine_score_distance{0.20f};
  // 精细打分的采样步长(通常设为 1,让候选之间的排序更可靠)。
  std::size_t refine_score_stride{1};
};

// 一个候选位姿及其评分。
struct GlobalSearchCandidate
{
  Eigen::Matrix4f pcd_from_odom{Eigen::Matrix4f::Identity()};
  float score{0.0f};
  // inlier 的平均残差(平方距离)。分数接近时用它做次级排序:
  // 命中率相当的对称两侧,残差更小的一侧更可能是真值。
  float mean_error{std::numeric_limits<float>::max()};
};

// 记录连续失败次数，够阈值后触发重定位。
class TrackingRecovery
{
public:
  // failure_threshold=0 时等于关闭统计。
  explicit TrackingRecovery(std::size_t failure_threshold)
  : failure_threshold_(failure_threshold)
  {
  }

  // 记录一次失败，返回是否已经到达阈值。
  bool recordFailure()
  {
    ++failure_count_;
    return failure_threshold_ > 0 && failure_count_ >= failure_threshold_;
  }

  // 成功后清零失败计数。
  void recordSuccess()
  {
    failure_count_ = 0;
  }

  // 当前累计失败次数。
  std::size_t failureCount() const
  {
    return failure_count_;
  }

private:
  std::size_t failure_threshold_{0};
  std::size_t failure_count_{0};
};

// 计算全局点云在 XY 平面的包围盒。
PlanarBounds computePlanarBounds(const PointCloudXYZI & cloud);

// 在包围盒内按步长生成候选位姿。
std::vector<Eigen::Matrix4f> generatePlanarCandidates(
  const PlanarBounds & bounds,
  const Eigen::Matrix4f & odom_from_base,
  const GlobalSearchConfig & config);

// 上一拍与候选 map→odom 的 XY 平移距离。LOST 仍全场搜，
// 但跳变超过门限（RMUL 对面角 ≈ 11m）就是锁错角。max_xy_jump<=0 关门。
// 注意：该判据的前提是"上一拍锚点大体正确"，前端漂移级故障下前提不成立，
// 由 jumpGateIsAdvisory 兜底放行。
inline float mapOdomXyJump(
  const Eigen::Matrix4f & previous, const Eigen::Matrix4f & candidate)
{
  const float dx = candidate(0, 3) - previous(0, 3);
  const float dy = candidate(1, 3) - previous(1, 3);
  return std::hypot(dx, dy);
}

inline bool mapOdomJumpExceeds(
  const Eigen::Matrix4f & previous,
  const Eigen::Matrix4f & candidate,
  float max_xy_jump)
{
  if (!(max_xy_jump > 0.0f)) {
    return false;
  }
  return mapOdomXyJump(previous, candidate) > max_xy_jump;
}

// 跳变门超时放行（lost_escape.*）：连续 LOST 超过 timeout_sec 后，跳变门不再
// 拦候选。判据本身是「候选 vs 上一拍锚点」，而上一拍锚点在前端失效级漂移时
// 就是错的 —— 此时门会把唯一正确的候选永久丢掉（实测：全场搜稳定给出
// fitness≈1.0 的真值，只因离错锚点 6.9m 被逐次拒绝，整车停在 LOST 不动）。
// 放行后候选仍须过 margin / GICP 精化 fitness / 时序校验。
inline bool jumpGateIsAdvisory(
  bool escape_enabled, bool lost_active, double lost_sec, double timeout_sec)
{
  if (!escape_enabled || !(timeout_sec > 0.0) || !lost_active) {
    return false;
  }
  return lost_sec >= timeout_sec;
}

// 给每个候选位姿打分，分数越高越像当前扫描。kdtree 由调用方传入并复用
// （节点已持有全局图的 kd-tree，每次调用重建是 O(N log N) 的浪费）。
std::vector<GlobalSearchCandidate> scoreGlobalCandidates(
  const pcl::KdTreeFLANN<Point> & kdtree,
  const PointCloudXYZI::ConstPtr & scan,
  const std::vector<Eigen::Matrix4f> & candidates,
  const GlobalSearchConfig & config);

// 只保留彼此间隔足够大的候选点。
std::vector<GlobalSearchCandidate> selectSeparatedCandidates(
  const std::vector<GlobalSearchCandidate> & ranked,
  const GlobalSearchConfig & config);

// 去重后的第一、第二名分差小于 margin 时视为歧义，整次拒绝。
bool candidatesAreAmbiguous(
  const std::vector<GlobalSearchCandidate> & selected,
  const GlobalSearchConfig & config);

// 对已经入选的少量候选做一次精细打分(stride=1、距离更严),用于打破粗搜的粒度。
// 只作用在 ~top_k 个候选上,几乎不增加总耗时。
std::vector<GlobalSearchCandidate> refineCandidateScores(
  const pcl::KdTreeFLANN<Point> & kdtree,
  const PointCloudXYZI::ConstPtr & scan,
  const std::vector<GlobalSearchCandidate> & selected,
  const GlobalSearchConfig & config);

}  // namespace fast_location

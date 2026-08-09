#pragma once

#include <cstddef>
#include <limits>
#include <vector>

#include <Eigen/Core>
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

// 给每个候选位姿打分，分数越高越像当前扫描。
std::vector<GlobalSearchCandidate> scoreGlobalCandidates(
  const PointCloudXYZI::ConstPtr & global_map,
  const PointCloudXYZI::ConstPtr & scan,
  const std::vector<Eigen::Matrix4f> & candidates,
  const GlobalSearchConfig & config);

// 只保留彼此间隔足够大的候选点。
std::vector<GlobalSearchCandidate> selectSeparatedCandidates(
  const std::vector<GlobalSearchCandidate> & ranked,
  const GlobalSearchConfig & config);

// 对已经入选的少量候选做一次精细打分(stride=1、距离更严),用于打破粗搜的粒度。
// 只作用在 ~top_k 个候选上,几乎不增加总耗时。
std::vector<GlobalSearchCandidate> refineCandidateScores(
  const PointCloudXYZI::ConstPtr & global_map,
  const PointCloudXYZI::ConstPtr & scan,
  const std::vector<GlobalSearchCandidate> & selected,
  const GlobalSearchConfig & config);

}  // namespace fast_location

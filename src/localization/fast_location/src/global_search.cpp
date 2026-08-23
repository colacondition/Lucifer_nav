#include "fast_location/global_search.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <pcl/kdtree/kdtree_flann.h>

#include "fast_location/frame_transforms.hpp"

namespace fast_location
{
namespace
{

constexpr float kPi = 3.14159265358979323846f;

// 把角度压到 [-pi, pi]。
float normalizeAngle(float angle)
{
  while (angle > kPi) {
    angle -= 2.0f * kPi;
  }
  while (angle <= -kPi) {
    angle += 2.0f * kPi;
  }
  return angle;
}

// 从齐次矩阵里取 yaw。
float yawOf(const Eigen::Matrix4f & transform)
{
  return std::atan2(transform(1, 0), transform(0, 0));
}

}  // namespace

PlanarBounds computePlanarBounds(const PointCloudXYZI & cloud)
{
  // 只看平面范围，后面按这个范围撒候选点。
  PlanarBounds bounds;
  bounds.min_x = std::numeric_limits<float>::infinity();
  bounds.max_x = -std::numeric_limits<float>::infinity();
  bounds.min_y = std::numeric_limits<float>::infinity();
  bounds.max_y = -std::numeric_limits<float>::infinity();

  for (const auto & point : cloud.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      continue;
    }
    bounds.min_x = std::min(bounds.min_x, point.x);
    bounds.max_x = std::max(bounds.max_x, point.x);
    bounds.min_y = std::min(bounds.min_y, point.y);
    bounds.max_y = std::max(bounds.max_y, point.y);
  }

  if (!std::isfinite(bounds.min_x) || !std::isfinite(bounds.min_y)) {
    throw std::invalid_argument("global map has no finite planar points");
  }
  return bounds;
}

std::vector<Eigen::Matrix4f> generatePlanarCandidates(
  const PlanarBounds & bounds,
  const Eigen::Matrix4f & odom_from_base,
  const GlobalSearchConfig & config)
{
  // 按 XY 网格和 yaw 网格撒一批粗候选。
  if (!(config.xy_step > 0.0f) || !(config.yaw_step > 0.0f)) {
    throw std::invalid_argument("global search steps must be positive");
  }
  if (bounds.max_x < bounds.min_x || bounds.max_y < bounds.min_y) {
    throw std::invalid_argument("global search bounds are invalid");
  }

  const auto base_from_odom = odom_from_base.inverse();
  const auto yaw_count = std::max(
    1, static_cast<int>(std::ceil((2.0f * kPi) / config.yaw_step)));
  const float actual_yaw_step = (2.0f * kPi) / static_cast<float>(yaw_count);

  // 候选数上限保护：地图出现飞点/异常边界时，xy 网格 × yaw 网格会让候选数量
  // 爆炸（500m×500m、0.5m 步长、30° yaw ≈ 1200 万），评分阶段每个候选都要做
  // KD 树查询，一次重定位可能 OOM 或阻塞数分钟。这里预先估算候选数，超限则
  // 倍增 xy_step 直到估算值回到上限以内，牺牲分辨率保住实时性与内存安全。
  const std::size_t max_candidates = std::max<std::size_t>(config.max_candidates, 1);
  float xy_step = config.xy_step;
  while (true) {
    const double nx = std::ceil(
      static_cast<double>(bounds.max_x - bounds.min_x) / xy_step) + 1.0;
    const double ny = std::ceil(
      static_cast<double>(bounds.max_y - bounds.min_y) / xy_step) + 1.0;
    const double estimated = nx * ny * static_cast<double>(yaw_count);
    if (estimated <= static_cast<double>(max_candidates)) {
      break;
    }
    // 候选数与步长平方成反比：乘以 sqrt(estimated/max) 一步压回上限附近。
    const double scale = std::sqrt(estimated / static_cast<double>(max_candidates));
    const float grown = xy_step * static_cast<float>(std::max(scale, 1.05));
    if (!(grown > xy_step) || !std::isfinite(grown)) {
      break;  // 浮点饱和兜底，避免死循环。
    }
    xy_step = grown;
  }

  const float epsilon = xy_step * 1e-4f;
  std::vector<Eigen::Matrix4f> candidates;
  candidates.reserve(
    std::min<std::size_t>(max_candidates, static_cast<std::size_t>(1) << 24));

  for (float x = bounds.min_x; x <= bounds.max_x + epsilon; x += xy_step) {
    for (float y = bounds.min_y; y <= bounds.max_y + epsilon; y += xy_step) {
      for (int yaw_index = 0; yaw_index < yaw_count; ++yaw_index) {
        const float yaw = -kPi + actual_yaw_step * static_cast<float>(yaw_index);
        const auto pcd_from_base = planarPoseMatrix({x, y, yaw});
        candidates.push_back(pcd_from_base * base_from_odom);
      }
    }
  }
  return candidates;
}

namespace
{

// 对单个位姿在给定 kdtree 上打分。score = 命中率,mean_error = 命中点平均平方残差。
void scoreSingleCandidate(
  const pcl::KdTreeFLANN<Point> & kdtree,
  const PointCloudXYZI & scan,
  const Eigen::Matrix4f & transform,
  std::size_t stride,
  float maximum_distance_sq,
  float & score,
  float & mean_error)
{
  std::size_t considered = 0;
  std::size_t inliers = 0;
  float error_sum = 0.0f;
  std::vector<int> indices(1);
  std::vector<float> squared_distances(1);

  for (std::size_t point_index = 0; point_index < scan.size(); point_index += stride) {
    const auto & source = scan.points[point_index];
    if (!std::isfinite(source.x) || !std::isfinite(source.y) || !std::isfinite(source.z)) {
      continue;
    }
    const Eigen::Vector4f transformed =
      transform * Eigen::Vector4f(source.x, source.y, source.z, 1.0f);
    Point query;
    query.x = transformed.x();
    query.y = transformed.y();
    query.z = transformed.z();
    ++considered;
    if (kdtree.nearestKSearch(query, 1, indices, squared_distances) > 0 &&
      squared_distances.front() <= maximum_distance_sq)
    {
      ++inliers;
      error_sum += squared_distances.front();
    }
  }

  score = considered == 0 ? 0.0f :
    static_cast<float>(inliers) / static_cast<float>(considered);
  mean_error = inliers == 0 ?
    std::numeric_limits<float>::max() :
    error_sum / static_cast<float>(inliers);
}

void sortByScoreWithTieBreak(
  std::vector<GlobalSearchCandidate> & ranked, float tie_epsilon)
{
  std::stable_sort(
    ranked.begin(), ranked.end(),
    [tie_epsilon](const GlobalSearchCandidate & left, const GlobalSearchCandidate & right) {
      // 命中率差距明显时按命中率排;差距在 tie_epsilon 内视为打平,改比平均残差(小者优先)。
      if (std::fabs(left.score - right.score) > tie_epsilon) {
        return left.score > right.score;
      }
      return left.mean_error < right.mean_error;
    });
}

}  // namespace

std::vector<GlobalSearchCandidate> scoreGlobalCandidates(
  const pcl::KdTreeFLANN<Point> & kdtree,
  const PointCloudXYZI::ConstPtr & scan,
  const std::vector<Eigen::Matrix4f> & candidates,
  const GlobalSearchConfig & config)
{
  // 用最近邻命中率给候选位姿打分。
  if (!scan || scan->empty()) {
    return {};
  }
  if (!(config.score_distance > 0.0f)) {
    throw std::invalid_argument("global search score distance must be positive");
  }
  // 采样稀疏一点，减小搜索代价。
  const std::size_t stride = std::max<std::size_t>(1, config.score_stride);
  const float maximum_distance_sq = config.score_distance * config.score_distance;
  std::vector<GlobalSearchCandidate> ranked;
  ranked.reserve(candidates.size());

  for (const auto & transform : candidates) {
    float score = 0.0f;
    float mean_error = std::numeric_limits<float>::max();
    scoreSingleCandidate(
      kdtree, *scan, transform, stride, maximum_distance_sq, score, mean_error);
    ranked.push_back({transform, score, mean_error});
  }

  sortByScoreWithTieBreak(ranked, std::max(0.0f, config.score_tie_epsilon));
  return ranked;
}

std::vector<GlobalSearchCandidate> selectSeparatedCandidates(
  const std::vector<GlobalSearchCandidate> & ranked,
  const GlobalSearchConfig & config)
{
  // 过滤掉彼此太近的候选，保留不同的局部极值。
  std::vector<GlobalSearchCandidate> selected;
  selected.reserve(std::min(config.top_k, ranked.size()));

  for (const auto & candidate : ranked) {
    bool separated = true;
    for (const auto & accepted : selected) {
      const float dx = candidate.pcd_from_odom(0, 3) - accepted.pcd_from_odom(0, 3);
      const float dy = candidate.pcd_from_odom(1, 3) - accepted.pcd_from_odom(1, 3);
      const float translation = std::hypot(dx, dy);
      const float yaw = std::abs(normalizeAngle(
          yawOf(candidate.pcd_from_odom) - yawOf(accepted.pcd_from_odom)));
      if (translation < config.minimum_candidate_separation &&
        yaw < config.minimum_candidate_yaw_separation)
      {
        separated = false;
        break;
      }
    }

    if (separated) {
      selected.push_back(candidate);
      if (selected.size() >= config.top_k) {
        break;
      }
    }
  }
  return selected;
}

bool candidatesAreAmbiguous(
  const std::vector<GlobalSearchCandidate> & selected,
  const GlobalSearchConfig & config)
{
  if (selected.size() < 2) {
    return false;
  }
  return selected.front().score - selected[1].score < config.minimum_score_margin;
}

std::vector<GlobalSearchCandidate> refineCandidateScores(
  const pcl::KdTreeFLANN<Point> & kdtree,
  const PointCloudXYZI::ConstPtr & scan,
  const std::vector<GlobalSearchCandidate> & selected,
  const GlobalSearchConfig & config)
{
  // 对少量入选候选用更严的 score_distance 和 stride 再打一次分,打破粗搜的量化粒度。
  if (selected.empty() || !scan || scan->empty()) {
    return selected;
  }

  const float refine_distance = config.refine_score_distance > 0.0f ?
    config.refine_score_distance : config.score_distance;
  const std::size_t refine_stride = std::max<std::size_t>(1, config.refine_score_stride);
  const float maximum_distance_sq = refine_distance * refine_distance;

  std::vector<GlobalSearchCandidate> refined;
  refined.reserve(selected.size());
  for (const auto & candidate : selected) {
    float score = 0.0f;
    float mean_error = std::numeric_limits<float>::max();
    scoreSingleCandidate(
      kdtree, *scan, candidate.pcd_from_odom, refine_stride, maximum_distance_sq,
      score, mean_error);
    refined.push_back({candidate.pcd_from_odom, score, mean_error});
  }

  sortByScoreWithTieBreak(refined, std::max(0.0f, config.score_tie_epsilon));
  return refined;
}

}  // namespace fast_location

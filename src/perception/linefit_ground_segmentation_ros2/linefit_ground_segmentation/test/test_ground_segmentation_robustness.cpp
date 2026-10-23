// linefit 地面分割的健壮性回归测试。
//
// 存在的理由：线搜索/台阶识别那套算法逻辑已有 test_step_detection.cpp 锁住，
// 这里只锁「输入边界」——即 2026-10 审计里发现的三类静默问题：
//   1. insertionThread 把 (range-r_min)/bin_step 直接转 unsigned int，浮点舍入
//      落到 n_bins 时就是 segments_[seg][n_bins] 越界写（UB，不一定当场崩）；
//   2. NaN/Inf 点会污染 bin 的最小高度（z=NaN 时 z<min_z 恒假，但 has_point_ 已
//      被置 true，bin 里留下一个 min_z=DBL_MAX 的假点），可能带坏整条地面线；
//   3. 非有限点最终 label=0，会被当成障碍发布给代价地图。
// 修复原则：合法输入的分割结果必须逐点不变（第 3 条测试直接对比修复前后等价
// 基线），只让非法输入不再制造 UB / 污染。
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "ground_segmentation/bin.h"
#include "ground_segmentation/ground_segmentation.h"

namespace
{

constexpr double kSensorHeight = 0.275;  // 与 common/segmentation.yaml 同量级
constexpr double kPointSpacing = 0.05;   // 径向采样步长

GroundSegmentationParams makeParams(double line_search_angle = 0.8)
{
  GroundSegmentationParams params;
  params.n_threads = 4;
  params.r_min_square = 0.2 * 0.2;
  params.r_max_square = 10.0 * 10.0;
  params.n_bins = 120;
  params.n_segments = 360;
  params.max_dist_to_line = 0.1;
  params.min_slope = -0.4;
  params.max_slope = 0.4;
  params.max_error_square = 0.05 * 0.05;
  params.long_threshold = 1.0;
  params.max_long_height = 0.1;
  params.max_start_height = 0.1;
  params.sensor_height = kSensorHeight;
  params.line_search_angle = line_search_angle;
  params.visualize = false;
  return params;
}

// 传感器系（base_link）下的平地：z = -sensor_height。
pcl::PointCloud<pcl::PointXYZ> buildFlatGround(
  double max_d = 9.9, double az_step_deg = 1.0, double skip_az_lo = 999.0,
  double skip_az_hi = 999.0)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (double az = -180.0; az < 180.0; az += az_step_deg) {
    if (az > skip_az_lo && az < skip_az_hi) {
      continue;
    }
    const double angle = az * M_PI / 180.0;
    for (double d = 0.25; d < max_d; d += kPointSpacing) {
      pcl::PointXYZ point;
      point.x = static_cast<float>(std::cos(angle) * d);
      point.y = static_cast<float>(std::sin(angle) * d);
      point.z = static_cast<float>(-kSensorHeight);
      cloud.push_back(point);
    }
  }
  return cloud;
}

}  // namespace

// ---------------------------------------------------------------------------
// 1) 径向 bin 索引：边界 clamp
// ---------------------------------------------------------------------------
TEST(Robustness, BinIndexIsClampedAtTheFloatingPointBoundary)
{
  // 病态参数：n_bins 不整除 r_max，使 nextafter(r_max,0)/bin_step 四舍五入到
  // 正好 n_bins。旧实现 static_cast<unsigned int>(q) == n_bins → 越界写。
  const int n_bins = 7;
  const double r_min = 0.0;
  const double r_max = 1.0;
  const double bin_step = (r_max - r_min) / n_bins;
  const double near_max = std::nextafter(r_max, 0.0);
  ASSERT_EQ(static_cast<int>(near_max / bin_step), n_bins)
    << "前提失效：这个浮点病态用例不再产生 q==n_bins，需重新找用例";

  EXPECT_EQ(computeBinIndex(near_max, r_min, r_max, bin_step, n_bins), n_bins - 1);
  EXPECT_EQ(computeBinIndex(r_min, r_min, r_max, bin_step, n_bins), 0);
  // 区间端点：r_max 属于「不参与分割」（旧实现同样落到 else）。
  EXPECT_EQ(computeBinIndex(r_max, r_min, r_max, bin_step, n_bins), -1);
  EXPECT_EQ(computeBinIndex(0.0, 0.1, r_max, bin_step, n_bins), -1);
  EXPECT_EQ(computeBinIndex(-1.0, r_min, r_max, bin_step, n_bins), -1);
}

TEST(Robustness, BinIndexNeverLeavesValidRange)
{
  const int n_bins = 120;
  const double r_min = 0.2;
  const double r_max = 10.0;
  const double bin_step = (r_max - r_min) / n_bins;
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> dist(-1.0, 12.0);
  for (int i = 0; i < 100000; ++i) {
    const double range = dist(rng);
    const int index = computeBinIndex(range, r_min, r_max, bin_step, n_bins);
    ASSERT_TRUE(index == -1 || (index >= 0 && index < n_bins))
      << "range=" << range << " index=" << index;
    if (range >= r_min && range < r_max) {
      ASSERT_NE(index, -1) << "range=" << range;
    }
  }
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_EQ(computeBinIndex(nan, r_min, r_max, bin_step, n_bins), -1);
  EXPECT_EQ(computeBinIndex(inf, r_min, r_max, bin_step, n_bins), -1);
  EXPECT_EQ(computeBinIndex(-inf, r_min, r_max, bin_step, n_bins), -1);
}

// ---------------------------------------------------------------------------
// 2) Bin：非有限点不得进入最小高度
// ---------------------------------------------------------------------------
TEST(Robustness, BinRejectsNonFinitePoints)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  Bin bin;
  bin.addPoint(1.0, nan);  // 旧实现：has_point_=true，min_z 仍是 DBL_MAX
  EXPECT_FALSE(bin.hasPoint());
  bin.addPoint(inf, -0.3);
  EXPECT_FALSE(bin.hasPoint());
  bin.addPoint(nan, -0.3);
  EXPECT_FALSE(bin.hasPoint());

  bin.addPoint(1.0, -0.3);
  ASSERT_TRUE(bin.hasPoint());
  Bin::MinZPoint min = bin.getMinZPoint();
  EXPECT_DOUBLE_EQ(min.d, 1.0);
  EXPECT_DOUBLE_EQ(min.z, -0.3);

  // 后续 NaN 点不能覆盖已有的合法最小点。
  bin.addPoint(2.0, nan);
  min = bin.getMinZPoint();
  EXPECT_DOUBLE_EQ(min.d, 1.0);
  EXPECT_DOUBLE_EQ(min.z, -0.3);

  bin.reset();
  EXPECT_FALSE(bin.hasPoint());
}

// ---------------------------------------------------------------------------
// 3) 点云里的 NaN/Inf：不崩、不被当地面、且不改变合法点的分类结果
// ---------------------------------------------------------------------------
TEST(Robustness, NonFinitePointsDoNotChangeFiniteResults)
{
  const pcl::PointCloud<pcl::PointXYZ> baseline = buildFlatGround();

  GroundSegmentation segmenter(makeParams());
  std::vector<int> baseline_labels;
  segmenter.segment(baseline, &baseline_labels);
  ASSERT_EQ(baseline_labels.size(), baseline.size());

  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  pcl::PointCloud<pcl::PointXYZ> cloud = baseline;
  const std::vector<pcl::PointXYZ> invalid = {
    {static_cast<float>(nan), 0.0f, static_cast<float>(-kSensorHeight)},
    {0.0f, static_cast<float>(nan), static_cast<float>(-kSensorHeight)},
    {5.0f, 5.0f, static_cast<float>(nan)},
    {static_cast<float>(inf), 0.0f, static_cast<float>(-kSensorHeight)},
    {0.0f, static_cast<float>(-inf), static_cast<float>(-kSensorHeight)},
  };
  const size_t first_invalid = cloud.size();
  for (const auto & point : invalid) {
    cloud.push_back(point);
  }

  std::vector<int> labels;
  segmenter.segment(cloud, &labels);
  ASSERT_EQ(labels.size(), cloud.size());

  // 契约 1：合法点逐个不变（这就是「低风险修复不改变正常场地结果」）。
  for (size_t i = 0; i < baseline.size(); ++i) {
    ASSERT_EQ(labels[i], baseline_labels[i]) << "point " << i;
  }
  // 契约 2：非有限点永远不是地面（旧实现同样如此，输出层已额外剔除，
  // 见 ground_segmentation_node.cc 的出口过滤）。
  for (size_t i = first_invalid; i < labels.size(); ++i) {
    EXPECT_NE(labels[i], 1) << "非有限点被判成地面";
  }
}

// ---------------------------------------------------------------------------
// 4) 空点云 / r_min、r_max 边界点：不崩，标签只有 0/1
// ---------------------------------------------------------------------------
TEST(Robustness, EmptyAndBoundaryCloudsAreSafe)
{
  GroundSegmentation segmenter(makeParams());
  std::vector<int> labels;
  pcl::PointCloud<pcl::PointXYZ> empty;
  segmenter.segment(empty, &labels);
  EXPECT_TRUE(labels.empty());

  // 一圈平地上再撒 r_min/r_max 附近的点（含正好等于端点、0、极小值）。
  pcl::PointCloud<pcl::PointXYZ> cloud = buildFlatGround();
  const std::vector<double> radii = {0.0, 0.199, 0.2,   0.2000001, 9.8,
                                     9.9, 9.99,  9.999, 10.0,      10.000001};
  for (double az = -180.0; az < 180.0; az += 7.0) {
    const double angle = az * M_PI / 180.0;
    for (double d : radii) {
      pcl::PointXYZ point;
      point.x = static_cast<float>(std::cos(angle) * d);
      point.y = static_cast<float>(std::sin(angle) * d);
      point.z = static_cast<float>(-kSensorHeight);
      cloud.push_back(point);
    }
  }
  segmenter.segment(cloud, &labels);
  ASSERT_EQ(labels.size(), cloud.size());
  // 新 label 约定：0=障碍、1=地面、2=忽略（r_min/r_max 外或非有限点）。
  // 距离边界 1cm 内的点允许落在任意 label（float 舍入决定归属，不是契约）；
  // 明确在范围外的点必须是 2（否则会被 ROS 节点当障碍发布），明确在内的是 0/1。
  for (size_t i = 0; i < labels.size(); ++i) {
    const float range_square = cloud[i].x * cloud[i].x + cloud[i].y * cloud[i].y;
    const float range = std::sqrt(range_square);
    const bool near_boundary =
      std::fabs(range - 0.2f) <= 0.01f || std::fabs(range - 10.0f) <= 0.01f;
    if (!near_boundary) {
      if (range <= 0.2f || range >= 10.0f) {
        ASSERT_EQ(labels[i], 2) << "out-of-range point " << i;
      } else {
        ASSERT_TRUE(labels[i] == 0 || labels[i] == 1) << "point " << i;
      }
    }
  }
  // 边界内（r < r_max 且 r > r_min）的平地必须基本判成地面；这里只做宽松上界，
  // 避免把「线端点 ±0.1 margin」这类算法细节写死。
  int in_range = 0, non_ground = 0;
  for (size_t i = 0; i < cloud.size(); ++i) {
    const double d = std::hypot(cloud[i].x, cloud[i].y);
    if (d < 9.5 && d > 0.25) {
      ++in_range;
      if (labels[i] != 1) ++non_ground;
    }
  }
  ASSERT_GT(in_range, 0);
  EXPECT_LT(100.0 * non_ground / in_range, 1.0)
    << "平地边界点出现整圈误判，边界防护可能把点扔出了分割";
}

// ---------------------------------------------------------------------------
// 5) line_search_angle：空扇区靠邻居线补地面，0.8 rad 必须能跨越 ~10°
// ---------------------------------------------------------------------------
TEST(Robustness, LineSearchAngleBridgesMissingAngularSector)
{
  // ±10° 的扇区不落点（模拟遮挡/欠采样），探针点贴着地面放在扇区中心。
  const pcl::PointCloud<pcl::PointXYZ> cloud = buildFlatGround(9.9, 1.0, -10.0, 10.0);
  pcl::PointCloud<pcl::PointXYZ> scene = cloud;
  pcl::PointXYZ probe;
  probe.x = static_cast<float>(5.0 * std::cos(0.0));
  probe.y = static_cast<float>(5.0 * std::sin(0.0));
  probe.z = static_cast<float>(-kSensorHeight);
  scene.push_back(probe);  // 方位 0°，正落在空扇区中心

  std::vector<int> labels_wide;
  GroundSegmentation wide(makeParams(0.8));  // 配置值 0.8 rad ≈ 46°
  wide.segment(scene, &labels_wide);
  EXPECT_EQ(labels_wide.back(), 1) << "line_search_angle=0.8 没能跨过 10° 空扇区，邻居线搜索失效";

  std::vector<int> labels_off;
  GroundSegmentation off(makeParams(0.0));  // 关闭邻居搜索
  off.segment(scene, &labels_off);
  EXPECT_EQ(labels_off.back(), 0)
    << "关掉 line_search_angle 后探针仍是地面，说明本用例没走到邻居搜索分支";
}

// 台阶识别的回归测试。
//
// 存在的理由：RMUL 场地里 20cm 高台一度在代价地图上完全不可见，排查了半天最后
// 落在 max_start_height 这一个参数上。它的名字("起始高度上限")完全看不出来跟
// 台阶识别有关，注释里写的是 "maximum difference to estimated ground height to
// start a new line"，很容易被当成无害的容差往大调。而一旦调回 0.5，高台就静默
// 消失 —— 代价地图上没有任何报错，只有车撞上去才知道。
//
// 机理(segment.cc:76-77)：地面线长到台阶前，第一个台面点让拟合误差爆掉，整条线
// 被拒、只留最后一个点；下一个点落进 size<2 的 else 分支，判据是
//   |上一点的 z - 已提交地面线末端高度| < max_start_height
// 该判据成立时台面点成为新线的起点，两三个点就拟合出一条贴着台面的「地面线」，
// 整片台面随即被 verticalDistanceToLine 判成地面。所以这个值的真实语义是
// 「能识别的最小台阶高度」，必须小于场地里最矮的那级台阶。
#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "ground_segmentation/ground_segmentation.h"

namespace {

constexpr double kSensorHeight = 0.275;   // 与 segmentation_sim.yaml 一致
constexpr double kPointSpacing = 0.06;    // 上游 cpp_lidar_filter 的体素尺寸
constexpr double kRayStepDeg = 0.25;      // 每个 segment 落 4 条射线
constexpr double kPlatformNear = 2.0;
constexpr double kPlatformFar = 4.0;
constexpr double kSectorHalfDeg = 30.0;

// 配置文件里的实际取值，改配置就要改这里，测试即契约。
constexpr int kConfigNBins = 120;
constexpr double kConfigMaxStartHeight = 0.1;
constexpr double kConfigMaxDistToLine = 0.1;
constexpr double kConfigMaxFitError = 0.05;
constexpr double kConfigMaxSlope = 0.4;

struct Scene {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  std::vector<bool> is_platform_top;   // 期望判成障碍
  std::vector<bool> is_ground;         // 期望判成地面
};

// 传感器系下的场景：一片地面(可带整体坡度)，正前方 ±30 度、径向 2~4 m 处
// 抬高 step_height 形成高台，并补上高台近端的竖直侧面。
Scene buildScene(double step_height, double ground_slope) {
  Scene scene;
  const bool has_platform = step_height > 0.0;
  for (double az = -180.0; az < 180.0; az += kRayStepDeg) {
    const double angle = az * M_PI / 180.0;
    const double cos_a = std::cos(angle);
    const double sin_a = std::sin(angle);
    const bool in_sector =
        has_platform && az >= -kSectorHalfDeg && az <= kSectorHalfDeg;

    for (double d = 0.25; d < 9.9; d += kPointSpacing) {
      const double ground_z = -kSensorHeight + ground_slope * d;
      const bool on_top =
          in_sector && d >= kPlatformNear && d <= kPlatformFar;
      pcl::PointXYZ point;
      point.x = static_cast<float>(cos_a * d);
      point.y = static_cast<float>(sin_a * d);
      point.z = static_cast<float>(on_top ? ground_z + step_height : ground_z);
      scene.cloud.push_back(point);
      scene.is_platform_top.push_back(on_top);
      scene.is_ground.push_back(!on_top);
    }

    // 高台近端的竖直面：激光唯一能直接看到的落差证据，最低点仍在地面高度上。
    if (in_sector) {
      const double base_z = -kSensorHeight + ground_slope * kPlatformNear;
      for (double z = base_z; z < base_z + step_height; z += kPointSpacing) {
        pcl::PointXYZ point;
        point.x = static_cast<float>(cos_a * kPlatformNear);
        point.y = static_cast<float>(sin_a * kPlatformNear);
        point.z = static_cast<float>(z);
        scene.cloud.push_back(point);
        scene.is_platform_top.push_back(false);
        scene.is_ground.push_back(false);   // 侧面点不参与两类统计
      }
    }
  }
  return scene;
}

GroundSegmentationParams makeParams(int n_bins, double max_start_height) {
  GroundSegmentationParams params;
  params.n_threads = 4;
  params.r_min_square = 0.2 * 0.2;
  params.r_max_square = 10.0 * 10.0;
  params.n_bins = n_bins;
  params.n_segments = 360;
  params.max_dist_to_line = kConfigMaxDistToLine;
  params.min_slope = -kConfigMaxSlope;
  params.max_slope = kConfigMaxSlope;
  // 注意：GroundSegmentationParams 存的是平方值，节点里传 max_fit_error^2。
  params.max_error_square = kConfigMaxFitError * kConfigMaxFitError;
  params.long_threshold = 1.0;
  params.max_long_height = 0.1;
  params.max_start_height = max_start_height;
  params.sensor_height = kSensorHeight;
  params.line_search_angle = 0.8;
  params.visualize = false;
  return params;
}

struct Rates {
  double top_missed_pct;      // 台面点被判成地面的比例
  double ground_false_pct;    // 地面点被判成障碍的比例
};

Rates classify(const GroundSegmentationParams& params, const Scene& scene) {
  GroundSegmentation segmenter(params);
  std::vector<int> labels;
  segmenter.segment(scene.cloud, &labels);

  int top_total = 0, top_as_ground = 0;
  int ground_total = 0, ground_as_obstacle = 0;
  for (size_t i = 0; i < labels.size(); ++i) {
    const bool labelled_ground = labels[i] == 1;
    if (scene.is_platform_top[i]) {
      ++top_total;
      if (labelled_ground) ++top_as_ground;
    }
    if (scene.is_ground[i]) {
      ++ground_total;
      if (!labelled_ground) ++ground_as_obstacle;
    }
  }
  return {top_total ? 100.0 * top_as_ground / top_total : 0.0,
          ground_total ? 100.0 * ground_as_obstacle / ground_total : 0.0};
}

}  // namespace

// 核心契约：配置文件里的这组参数必须能看见 20cm 高台。
TEST(StepDetection, ConfiguredParamsSeeTwentyCentimetrePlatform) {
  const Scene scene = buildScene(0.20, 0.0);
  const Rates rates =
      classify(makeParams(kConfigNBins, kConfigMaxStartHeight), scene);
  EXPECT_LT(rates.top_missed_pct, 1.0)
      << "20cm 高台的台面被判成地面，代价地图上会完全看不到它。"
         "先检查 segmentation_{sim,real}.yaml 的 max_start_height "
         "是否 >= 台阶高度。";
  EXPECT_LT(rates.ground_false_pct, 1.0) << "平地被误判成障碍。";
}

// 反向锁定：把 max_start_height 放回 0.5 必须让高台消失。这条测试保证上面那条
// 不是碰巧通过 —— 它证明高台可见性真的由这个参数控制。
TEST(StepDetection, LargeMaxStartHeightSwallowsPlatform) {
  const Scene scene = buildScene(0.20, 0.0);
  const Rates rates = classify(makeParams(kConfigNBins, 0.5), scene);
  EXPECT_GT(rates.top_missed_pct, 50.0)
      << "max_start_height=0.5 竟然还能看见 20cm 高台，说明上游算法已经改过，"
         "本测试对参数含义的假设需要重新确认。";
}

// max_start_height 的语义 = 能识别的最小台阶高度：只有高于它的台阶才看得见。
TEST(StepDetection, MaxStartHeightBoundsSmallestVisibleStep) {
  for (const double step : {0.10, 0.15, 0.20, 0.30}) {
    const Scene scene = buildScene(step, 0.0);
    const Rates rates =
        classify(makeParams(kConfigNBins, kConfigMaxStartHeight), scene);
    EXPECT_LT(rates.top_missed_pct, 1.0)
        << "台阶高度 " << step << " m 高于 max_start_height="
        << kConfigMaxStartHeight << "，本该看得见。";
  }
}

// 收紧 max_start_height 不会把带坡度的地面误标成障碍 —— 这是当初不敢往下调的
// 主要顾虑，实测到 15% 坡度都没有虚警。
TEST(StepDetection, SlopedGroundDoesNotBecomeObstacle) {
  for (const double slope : {0.0, 0.05, 0.10, 0.15}) {
    const Scene scene = buildScene(0.0, slope);
    const Rates rates =
        classify(makeParams(kConfigNBins, kConfigMaxStartHeight), scene);
    EXPECT_LT(rates.ground_false_pct, 1.0)
        << "坡度 " << slope * 100 << "% 的地面被误判成障碍，"
        << "车会被一圈虚假障碍围住。";
  }
}

// n_bins 与高台识别无关，别再为了「提高分辨率」去动它（改了只是白烧 CPU）。
TEST(StepDetection, RadialResolutionDoesNotAffectStepDetection) {
  const Scene scene = buildScene(0.20, 0.0);
  for (const int n_bins : {80, 120, 200, 300}) {
    const Rates rates = classify(makeParams(n_bins, kConfigMaxStartHeight), scene);
    EXPECT_LT(rates.top_missed_pct, 1.0) << "n_bins=" << n_bins;
  }
}

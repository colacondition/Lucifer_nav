#include <cmath>
#include <vector>

#include <gtest/gtest.h>
#include <pcl/common/transforms.h>

#include "fast_location/alignment_quality.hpp"
#include "fast_location/frame_transforms.hpp"
#include "fast_location/global_search.hpp"

namespace
{
using fast_location::GlobalSearchCandidate;
using fast_location::GlobalSearchConfig;
using fast_location::PointCloudXYZI;

PointCloudXYZI::Ptr makeAsymmetricMap()
{
  auto cloud = std::make_shared<PointCloudXYZI>();
  for (int index = 0; index < 12; ++index) {
    fast_location::Point point;
    point.x = static_cast<float>(index) * 0.25f;
    point.y = index < 7 ? 0.0f : 1.0f + static_cast<float>(index - 7) * 0.2f;
    point.z = index % 3 == 0 ? 0.2f : 0.0f;
    cloud->push_back(point);
  }
  return cloud;
}
}  // namespace

TEST(GlobalSearchTest, CoversMapBoundsAndFullYawRange)
{
  fast_location::PlanarBounds bounds{-2.0f, 2.0f, -1.0f, 3.0f};
  GlobalSearchConfig config;
  config.xy_step = 2.0f;
  config.yaw_step = static_cast<float>(M_PI_2);

  const auto candidates = fast_location::generatePlanarCandidates(
    bounds, Eigen::Matrix4f::Identity(), config);

  EXPECT_EQ(candidates.size(), 36u);
}

TEST(GlobalSearchTest, RanksKnownSyntheticPoseFirst)
{
  const auto global_map = makeAsymmetricMap();
  const auto expected = fast_location::planarPoseMatrix({2.0, -1.0, 0.0});
  auto scan = std::make_shared<PointCloudXYZI>();
  pcl::transformPointCloud(*global_map, *scan, expected.inverse());

  GlobalSearchConfig config;
  config.score_distance = 0.05f;
  config.score_stride = 1;
  std::vector<Eigen::Matrix4f> candidates{
    fast_location::planarPoseMatrix({0.0, 0.0, 0.0}),
    expected,
    fast_location::planarPoseMatrix({-1.0, 2.0, M_PI_2}),
  };

  const auto ranked = fast_location::scoreGlobalCandidates(
    global_map, scan, candidates, config);

  ASSERT_EQ(ranked.size(), candidates.size());
  EXPECT_TRUE(ranked.front().pcd_from_odom.isApprox(expected, 1e-6f));
  EXPECT_NEAR(ranked.front().score, 1.0f, 1e-6f);
}

TEST(GlobalSearchTest, BreaksScoreTiesByMeanResidual)
{
  // 构造两个候选:命中率完全相同,但一个残差略大。
  // tie-break 应让残差更小的候选排在前面。
  const auto global_map = makeAsymmetricMap();

  // 真值位姿:扫描正好对齐地图,残差近似 0。
  const auto exact = fast_location::planarPoseMatrix({2.0, -1.0, 0.0});
  auto scan = std::make_shared<PointCloudXYZI>();
  pcl::transformPointCloud(*global_map, *scan, exact.inverse());

  // 偏移一点点(小于 score_distance)的位姿:仍全部命中,但残差更大。
  const auto shifted = fast_location::planarPoseMatrix({2.0 + 0.03, -1.0, 0.0});

  GlobalSearchConfig config;
  config.score_distance = 0.2f;  // 足够大,两个候选都全命中
  config.score_stride = 1;
  config.score_tie_epsilon = 0.05f;  // 命中率相同 -> 触发残差决胜
  std::vector<Eigen::Matrix4f> candidates{shifted, exact};

  const auto ranked = fast_location::scoreGlobalCandidates(
    global_map, scan, candidates, config);

  ASSERT_EQ(ranked.size(), candidates.size());
  EXPECT_NEAR(ranked.front().score, ranked.back().score, 1e-6f);
  EXPECT_TRUE(ranked.front().pcd_from_odom.isApprox(exact, 1e-6f));
  EXPECT_LT(ranked.front().mean_error, ranked.back().mean_error);
}

TEST(GlobalSearchTest, RefinesTopCandidatesWithTighterMetric)
{
  // 精细打分:偏移量比 refine_score_distance 大的候选应被降到后面。
  const auto global_map = makeAsymmetricMap();
  const auto exact = fast_location::planarPoseMatrix({2.0, -1.0, 0.0});
  auto scan = std::make_shared<PointCloudXYZI>();
  pcl::transformPointCloud(*global_map, *scan, exact.inverse());

  // shifted 与真值差 0.15,超过 refine 距离 0.05,精化时应大幅降分。
  const auto shifted = fast_location::planarPoseMatrix({2.0 + 0.15, -1.0, 0.0});

  GlobalSearchConfig config;
  // 粗搜距离比较松,两者都近似满分。
  config.score_distance = 0.5f;
  config.score_stride = 1;
  config.refine_score_distance = 0.05f;
  config.refine_score_stride = 1;
  config.score_tie_epsilon = 0.0f;
  std::vector<GlobalSearchCandidate> selected{
    {shifted, 1.0f, 0.0f},
    {exact, 1.0f, 0.0f},
  };

  const auto refined = fast_location::refineCandidateScores(
    global_map, scan, selected, config);

  ASSERT_EQ(refined.size(), 2u);
  EXPECT_TRUE(refined.front().pcd_from_odom.isApprox(exact, 1e-6f));
  EXPECT_GT(refined.front().score, refined.back().score);
}

TEST(GlobalSearchTest, SelectsSeparatedTopCandidates)
{
  GlobalSearchConfig config;
  config.top_k = 2;
  config.minimum_candidate_separation = 1.0f;
  config.minimum_candidate_yaw_separation = 0.3f;
  std::vector<GlobalSearchCandidate> ranked{
    {fast_location::planarPoseMatrix({0.0, 0.0, 0.0}), 0.90f},
    {fast_location::planarPoseMatrix({0.2, 0.1, 0.05}), 0.85f},
    {fast_location::planarPoseMatrix({3.0, 0.0, 0.0}), 0.80f},
  };

  const auto selected = fast_location::selectSeparatedCandidates(ranked, config);

  ASSERT_EQ(selected.size(), 2u);
  EXPECT_NEAR(selected[0].score, 0.90f, 1e-6f);
  EXPECT_NEAR(selected[1].score, 0.80f, 1e-6f);
}

TEST(GlobalSearchTest, RequestsGlobalSearchAfterConfiguredTrackingFailures)
{
  fast_location::TrackingRecovery recovery(3);

  EXPECT_FALSE(recovery.recordFailure());
  EXPECT_FALSE(recovery.recordFailure());
  EXPECT_TRUE(recovery.recordFailure());
  EXPECT_EQ(recovery.failureCount(), 3u);

  recovery.recordSuccess();
  EXPECT_EQ(recovery.failureCount(), 0u);
}

TEST(AlignmentQualityTest, ScoresValidAlignmentWhenSolverDoesNotReportConvergence)
{
  const auto cloud = makeAsymmetricMap();

  const auto assessment = fast_location::assessAlignment(
    false, cloud, cloud, 0.05f);

  EXPECT_FALSE(assessment.solver_converged);
  EXPECT_TRUE(assessment.usable);
  EXPECT_NEAR(assessment.inlier_ratio, 1.0f, 1e-6f);
}

// 构造一个「沿 x 方向强约束、沿 y 方向弱约束」的平移信息矩阵。
// 对应现实场景：走廊沿 y 延伸，只有两侧墙面提供 x 方向约束。
Eigen::Matrix<double, 6, 6> makeCorridorHessian(double info_x, double info_y)
{
  Eigen::Matrix<double, 6, 6> hessian = Eigen::Matrix<double, 6, 6>::Identity();
  // fast_gicp 排序为 [rot(0:3), trans(3:6)]，平移块在 (3,3)。
  hessian(3, 3) = info_x;
  hessian(4, 4) = info_y;
  hessian(5, 5) = 1.0;
  return hessian;
}

TEST(AlignmentQualityTest, HessianIdentifiesObservableAxisInCorridor)
{
  // x 方向信息量是 y 的 1000 倍 → y 是退化方向。
  const auto hessian = makeCorridorHessian(1000.0, 1.0);
  Eigen::Vector2f observable = Eigen::Vector2f::Zero();
  Eigen::Vector2f degenerate = Eigen::Vector2f::Zero();

  const float condition =
    fast_location::computeHessianConditionNumber(hessian, observable, degenerate);

  EXPECT_NEAR(condition, 1000.0f, 1.0f);
  // 可观测方向应为 ±x，退化方向应为 ±y。
  EXPECT_NEAR(std::abs(observable.x()), 1.0f, 1e-4f);
  EXPECT_NEAR(std::abs(observable.y()), 0.0f, 1e-4f);
  EXPECT_NEAR(std::abs(degenerate.y()), 1.0f, 1e-4f);
  // 两个方向必须正交。
  EXPECT_NEAR(observable.dot(degenerate), 0.0f, 1e-4f);
}

TEST(AlignmentQualityTest, HessianReportsWellConstrainedSceneAsNonDegenerate)
{
  const auto hessian = makeCorridorHessian(100.0, 80.0);
  Eigen::Vector2f observable = Eigen::Vector2f::Zero();
  Eigen::Vector2f degenerate = Eigen::Vector2f::Zero();

  const float condition =
    fast_location::computeHessianConditionNumber(hessian, observable, degenerate);

  EXPECT_NEAR(condition, 1.25f, 0.01f);
  EXPECT_LT(condition, 80.0f);  // 低于生产阈值，不该触发退化处理
}

TEST(AlignmentQualityTest, HessianRejectsNonFiniteAndNonPositiveDiagonal)
{
  Eigen::Vector2f observable = Eigen::Vector2f::Zero();
  Eigen::Vector2f degenerate = Eigen::Vector2f::Zero();

  auto nan_hessian = makeCorridorHessian(100.0, 100.0);
  nan_hessian(3, 4) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_LT(
    fast_location::computeHessianConditionNumber(nan_hessian, observable, degenerate), 0.0f);

  // 信息矩阵对角元为零 → 该方向完全无约束，应报告不可用而非算出条件数。
  const auto zero_hessian = makeCorridorHessian(100.0, 0.0);
  EXPECT_LT(
    fast_location::computeHessianConditionNumber(zero_hessian, observable, degenerate), 0.0f);
}

TEST(AlignmentQualityTest, ProjectionKeepsObservableCorrectionAndDropsDegenerateOne)
{
  const Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f corrected = Eigen::Matrix4f::Identity();
  // GICP 想在 x 上修 0.10 m（可信），在 y 上修 0.80 m（走廊方向，是噪声）。
  corrected(0, 3) = 0.10f;
  corrected(1, 3) = 0.80f;
  corrected(2, 3) = 0.05f;

  const Eigen::Vector2f observable(1.0f, 0.0f);
  const auto projected =
    fast_location::projectToObservableSubspace(corrected, guess, observable);

  EXPECT_NEAR(projected(0, 3), 0.10f, 1e-6f);   // 可观测方向的修正保留
  EXPECT_NEAR(projected(1, 3), 0.0f, 1e-6f);    // 退化方向丢弃，回到初值
  EXPECT_NEAR(projected(2, 3), 0.0f, 1e-6f);    // z 交回初值
}

TEST(AlignmentQualityTest, ProjectionHandlesDiagonalObservableAxis)
{
  Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
  guess(0, 3) = 1.0f;
  guess(1, 3) = 2.0f;

  Eigen::Matrix4f corrected = guess;
  // 修正量 (0.4, 0.0)，可观测方向是 45°。
  corrected(0, 3) = 1.4f;

  const Eigen::Vector2f observable(
    static_cast<float>(M_SQRT1_2), static_cast<float>(M_SQRT1_2));
  const auto projected =
    fast_location::projectToObservableSubspace(corrected, guess, observable);

  // (0.4,0) 在 45° 上的投影长度是 0.4/√2，分量各为 0.2。
  EXPECT_NEAR(projected(0, 3), 1.2f, 1e-5f);
  EXPECT_NEAR(projected(1, 3), 2.2f, 1e-5f);

  // 投影后的增量必须与退化方向正交。
  const Eigen::Vector2f delta = projected.block<2, 1>(0, 3) - guess.block<2, 1>(0, 3);
  const Eigen::Vector2f degenerate(-observable.y(), observable.x());
  EXPECT_NEAR(delta.dot(degenerate), 0.0f, 1e-5f);
}

TEST(AlignmentQualityTest, ProjectionFallsBackToGuessWhenAxisUnknown)
{
  Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
  guess(0, 3) = 3.0f;
  Eigen::Matrix4f corrected = guess;
  corrected(0, 3) = 9.0f;

  // 零向量表示方向未知；此时不应采纳任何修正。
  const auto projected = fast_location::projectToObservableSubspace(
    corrected, guess, Eigen::Vector2f::Zero());

  EXPECT_NEAR(projected(0, 3), 3.0f, 1e-6f);
}

TEST(AlignmentQualityTest, PointCloudFallbackInvertsDirectionSemantics)
{
  // 沿 x 铺开的一条线：点云分布 λ_max 在 x，但 x 恰是不可观测方向。
  auto cloud = std::make_shared<PointCloudXYZI>();
  for (int index = 0; index < 60; ++index) {
    fast_location::Point point;
    point.x = static_cast<float>(index) * 0.5f;
    point.y = (index % 2 == 0) ? 0.01f : -0.01f;
    point.z = 0.0f;
    cloud->push_back(point);
  }

  Eigen::Vector2f observable = Eigen::Vector2f::Zero();
  Eigen::Vector2f degenerate = Eigen::Vector2f::Zero();
  const float condition =
    fast_location::compute2DConditionNumber(cloud, &observable, &degenerate);

  EXPECT_GT(condition, 100.0f);
  // 退化方向应为 x（点铺开的方向），可观测方向为 y。
  EXPECT_NEAR(std::abs(degenerate.x()), 1.0f, 1e-3f);
  EXPECT_NEAR(std::abs(observable.y()), 1.0f, 1e-3f);
}

TEST(AlignmentQualityTest, AssessAlignmentPrefersHessianOverPointDistribution)
{
  const auto cloud = makeAsymmetricMap();
  // Hessian 说场景约束良好，尽管点云分布本身可能偏窄。
  const auto hessian = makeCorridorHessian(100.0, 90.0);

  const auto assessment = fast_location::assessAlignment(
    true, cloud, cloud, 0.05f, 80.0f, &hessian);

  EXPECT_TRUE(assessment.usable);
  EXPECT_FALSE(assessment.is_degenerate);
  EXPECT_NEAR(assessment.condition_number, 100.0f / 90.0f, 0.01f);
}

TEST(AlignmentQualityTest, AssessAlignmentFallsBackWhenHessianUnusable)
{
  const auto cloud = makeAsymmetricMap();
  // 对角元为零 → Hessian 路径不可用，应自动退回点云分布。
  const auto bad_hessian = makeCorridorHessian(0.0, 0.0);

  const auto assessment = fast_location::assessAlignment(
    true, cloud, cloud, 0.05f, 80.0f, &bad_hessian);

  EXPECT_TRUE(assessment.usable);
  // 退回路径必须算出了一个有效条件数，而不是留在 -1。
  EXPECT_GT(assessment.condition_number, 0.0f);
}

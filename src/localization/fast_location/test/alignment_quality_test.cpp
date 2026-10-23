#include <gtest/gtest.h>

#include <limits>

#include "fast_location/alignment_quality.hpp"

TEST(AlignmentQualityTest, WrapAngleKeepsDeltaInsidePi)
{
  EXPECT_NEAR(fast_location::wrapAnglePi(3.5), 3.5 - 2.0 * 3.14159265358979323846, 1e-9);
  EXPECT_NEAR(fast_location::wrapAnglePi(-3.5), -3.5 + 2.0 * 3.14159265358979323846, 1e-9);
  EXPECT_NEAR(fast_location::wrapAnglePi(0.2), 0.2, 1e-12);
}

TEST(AlignmentQualityTest, IdenticalPosesGiveNearZeroNis)
{
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  Eigen::Matrix<double, 6, 6> hessian = Eigen::Matrix<double, 6, 6>::Identity();
  hessian *= 100.0;

  const auto nis = fast_location::computeScanToMapNis(pose, pose, hessian, 0.20, 0.14);
  ASSERT_TRUE(nis.valid);
  EXPECT_NEAR(nis.value, 0.0, 1e-9);
  EXPECT_NEAR(nis.innovation.norm(), 0.0, 1e-12);
}

TEST(AlignmentQualityTest, LargeXyInnovationExceedsDefaultGate)
{
  Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f measured = Eigen::Matrix4f::Identity();
  measured(0, 3) = 1.0f;
  Eigen::Matrix<double, 6, 6> hessian = Eigen::Matrix<double, 6, 6>::Identity();
  hessian *= 100.0;

  const auto nis = fast_location::computeScanToMapNis(guess, measured, hessian, 0.20, 0.14);
  ASSERT_TRUE(nis.valid);
  EXPECT_GT(nis.value, 12.0);
  EXPECT_NEAR(nis.innovation.x(), 1.0, 1e-6);
}

TEST(AlignmentQualityTest, DegenerateHessianIsInvalid)
{
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  Eigen::Matrix<double, 6, 6> hessian = Eigen::Matrix<double, 6, 6>::Zero();

  const auto nis = fast_location::computeScanToMapNis(pose, pose, hessian, 0.20, 0.14);
  EXPECT_FALSE(nis.valid);
}

TEST(AlignmentQualityTest, IntegrityStateNamesAreStable)
{
  EXPECT_STREQ(fast_location::integrityStateName(fast_location::IntegrityState::OK), "OK");
  EXPECT_STREQ(fast_location::integrityStateName(fast_location::IntegrityState::LOST), "LOST");
}

namespace
{
// 合成 6×6 Hessian（状态序 [rot(0:3), trans(3:6)]）：对角块各向同性。
Eigen::Matrix<double, 6, 6> makeBlockDiagonalHessian(
  const Eigen::Vector3d & rotation_diag, const Eigen::Vector3d & translation_diag)
{
  Eigen::Matrix<double, 6, 6> hessian = Eigen::Matrix<double, 6, 6>::Zero();
  hessian.block<3, 3>(0, 0) = rotation_diag.asDiagonal();
  hessian.block<3, 3>(3, 3) = translation_diag.asDiagonal();
  return hessian;
}
}  // namespace

TEST(AlignmentQualityTest, WellConditionedSixDofHessianIsFullRank)
{
  const auto hessian = makeBlockDiagonalHessian(
    Eigen::Vector3d(1000.0, 1000.0, 1000.0),
    Eigen::Vector3d(500.0, 500.0, 500.0));
  const auto observability = fast_location::analyzeSixDofObservability(hessian, 1e-3);

  ASSERT_TRUE(observability.valid);
  EXPECT_EQ(observability.translation_rank, 3);
  EXPECT_EQ(observability.rotation_rank, 3);
  // 特征值升序输出。
  EXPECT_NEAR(observability.translation_eigenvalues(0), 500.0, 1e-6);
  EXPECT_NEAR(observability.translation_eigenvalues(2), 500.0, 1e-6);
  EXPECT_NEAR(observability.translation_condition, 1.0, 1e-9);
  EXPECT_NEAR(observability.rotation_condition, 1.0, 1e-9);
  EXPECT_TRUE(observability.yaw_observable);
  EXPECT_GT(observability.yaw_information, 0.0);
  EXPECT_TRUE(fast_location::isFullPoseObservable(observability));
}

TEST(AlignmentQualityTest, CorridorDegeneracyDropsTranslationRankToTwo)
{
  // 走廊沿 x：x 方向信息被压到远小于 y。λ_x/λ_max = 0.4/800 < 1e-3 → 降秩。
  const auto hessian = makeBlockDiagonalHessian(
    Eigen::Vector3d(1000.0, 1000.0, 800.0),
    Eigen::Vector3d(0.4, 500.0, 500.0));
  const auto observability = fast_location::analyzeSixDofObservability(hessian, 1e-3);

  ASSERT_TRUE(observability.valid);
  EXPECT_EQ(observability.translation_rank, 2);
  EXPECT_EQ(observability.rotation_rank, 3);
  // 弱方向应基本是 x 轴。
  EXPECT_GT(std::abs(observability.translation_weak_direction.x()), 0.9);
  EXPECT_GT(observability.translation_condition, 1e3);
  EXPECT_TRUE(observability.yaw_observable);
  // 平移秩 2 且 yaw 可观测：平面机器人视为完整位姿（z 不参与）。
  EXPECT_TRUE(fast_location::isFullPoseObservable(observability));
}

TEST(AlignmentQualityTest, SingleDirectionTranslationLosesFullPoseGate)
{
  // 只有一个平移方向有信息（rank=1）→ 首配完整位姿门必须拒绝。
  const auto hessian = makeBlockDiagonalHessian(
    Eigen::Vector3d(1000.0, 1000.0, 1000.0),
    Eigen::Vector3d(500.0, 0.0, 0.0));
  const auto observability = fast_location::analyzeSixDofObservability(hessian, 1e-3);

  ASSERT_TRUE(observability.valid);
  EXPECT_EQ(observability.translation_rank, 1);
  EXPECT_GT(observability.translation_condition, 1e6);
  EXPECT_FALSE(fast_location::isFullPoseObservable(observability));
}

TEST(AlignmentQualityTest, RotationDegeneracyMakesYawUnobservable)
{
  // 旋转块里 yaw（index 2）完全没有信息：平移满秩也不能放行首配。
  const auto hessian = makeBlockDiagonalHessian(
    Eigen::Vector3d(1000.0, 1000.0, 0.0),
    Eigen::Vector3d(500.0, 500.0, 500.0));
  const auto observability = fast_location::analyzeSixDofObservability(hessian, 1e-3);

  ASSERT_TRUE(observability.valid);
  EXPECT_EQ(observability.translation_rank, 3);
  EXPECT_EQ(observability.rotation_rank, 2);
  EXPECT_FALSE(observability.yaw_observable);
  EXPECT_FALSE(fast_location::isFullPoseObservable(observability));
}

TEST(AlignmentQualityTest, ChiSquareUpperTailMatchesReferenceValues)
{
  // 3 自由度：χ²=11.34 → 上尾≈0.01；χ²=16.27 → 上尾≈1e-3。
  EXPECT_NEAR(fast_location::chiSquareUpperTail(11.34, 3), 1.0e-2, 2e-3);
  EXPECT_NEAR(fast_location::chiSquareUpperTail(16.27, 3), 1.0e-3, 2e-3);
  // 1 自由度 95% 分位 3.841；2 自由度 95% 分位 5.991。
  EXPECT_NEAR(fast_location::chiSquareUpperTail(3.841, 1), 0.05, 1e-3);
  EXPECT_NEAR(fast_location::chiSquareUpperTail(5.991, 2), 0.05, 1e-3);
  // x<=0 表示统计量在分布左端，上尾概率为 1；dof 非法返回 -1（不抛异常）。
  EXPECT_NEAR(fast_location::chiSquareUpperTail(-1.0, 3), 1.0, 1e-12);
  EXPECT_LT(fast_location::chiSquareUpperTail(1.0, 0), 0.0);
  EXPECT_LT(fast_location::chiSquareUpperTail(std::numeric_limits<double>::quiet_NaN(), 3), 0.0);
}

TEST(AlignmentQualityTest, ChiSquareThresholdInvertsUpperTail)
{
  const double threshold_3dof = fast_location::chiSquareThreshold(3, 1e-3);
  EXPECT_NEAR(threshold_3dof, 16.27, 0.1);
  EXPECT_NEAR(fast_location::chiSquareUpperTail(threshold_3dof, 3), 1e-3, 1e-6);

  const double threshold_2dof = fast_location::chiSquareThreshold(2, 1e-3);
  EXPECT_NEAR(threshold_2dof, 13.82, 0.1);
  EXPECT_NEAR(fast_location::chiSquareUpperTail(threshold_2dof, 2), 1e-3, 1e-6);
}

TEST(AlignmentQualityTest, ProjectedNisIgnoresWeakDirectionInnovation)
{
  // H3 顺序为 [dx, dy, dyaw] = Hessian 的 [3, 4, 2]。x 方向退化（0.4/800 < 1e-3）。
  Eigen::Matrix<double, 6, 6> hessian = Eigen::Matrix<double, 6, 6>::Zero();
  hessian(3, 3) = 0.4;    // dx 弱
  hessian(4, 4) = 500.0;  // dy
  hessian(2, 2) = 800.0;  // dyaw

  Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f measured = Eigen::Matrix4f::Identity();
  measured(0, 3) = 1.0f;  // 沿退化方向（x）1m 的创新

  const auto nis = fast_location::computeScanToMapNisProjected(
    guess, measured, hessian, 0.20, 0.14, 1e-3);
  ASSERT_TRUE(nis.valid);
  EXPECT_EQ(nis.dof, 2);           // x 被剔除，剩 dy + dyaw
  EXPECT_LT(nis.value, 1e-6);      // 弱方向不参与判拒
  EXPECT_GT(nis.upper_tail, 0.5);  // 完全不像不一致观测
}

TEST(AlignmentQualityTest, ProjectedNisRejectsStrongDirectionInnovation)
{
  Eigen::Matrix<double, 6, 6> hessian = Eigen::Matrix<double, 6, 6>::Zero();
  hessian(3, 3) = 0.4;    // dx 弱（退化方向）
  hessian(4, 4) = 500.0;  // dy 强观测
  hessian(2, 2) = 800.0;  // dyaw 强观测

  Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f measured = Eigen::Matrix4f::Identity();
  measured(1, 3) = 1.0f;  // 沿强观测方向（y）1m 的创新

  const auto nis = fast_location::computeScanToMapNisProjected(
    guess, measured, hessian, 0.20, 0.14, 1e-3);
  ASSERT_TRUE(nis.valid);
  EXPECT_EQ(nis.dof, 2);
  // S = R + P ≈ 1/500 + 0.04 = 0.042，1m 创新 NIS≈23.8；2DoF 的 1e-3 门限≈13.82。
  EXPECT_GT(nis.value, fast_location::chiSquareThreshold(2, 1e-3));
  EXPECT_LT(nis.upper_tail, 1e-3);
}

TEST(AlignmentQualityTest, ProjectedNisAllDegenerateIsInvalid)
{
  // 6×6 Hessian 全零：没有任何可观测方向，不应判拒（valid=false）。
  const Eigen::Matrix<double, 6, 6> hessian = Eigen::Matrix<double, 6, 6>::Zero();
  const Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  const auto nis = fast_location::computeScanToMapNisProjected(
    pose, pose, hessian, 0.20, 0.14, 1e-3);
  EXPECT_FALSE(nis.valid);
}

TEST(AlignmentQualityTest, AssessmentDefaultHasNoFullPoseClaim)
{
  // 没有 Hessian 的兜底路径不能声称"完整位姿可观测"。
  const fast_location::AlignmentAssessment assessment;
  EXPECT_FALSE(assessment.full_pose_observable);
  EXPECT_FALSE(assessment.six_dof.valid);
  EXPECT_FALSE(fast_location::isFullPoseObservable(assessment.six_dof));
}

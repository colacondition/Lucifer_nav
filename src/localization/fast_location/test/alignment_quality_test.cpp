#include <gtest/gtest.h>

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

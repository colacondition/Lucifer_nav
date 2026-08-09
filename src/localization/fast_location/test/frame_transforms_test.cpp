#include <array>
#include <limits>

#include <gtest/gtest.h>

#include "fast_location/frame_transforms.hpp"

TEST(FrameTransformsTest, ComposesTranslationAndYawIntoMapToOdom)
{
  constexpr double kHalfPi = 1.5707963267948966;
  const auto map_from_pcd = fast_location::planarPoseMatrix({10.0, -1.0, kHalfPi});
  const auto pcd_from_odom = fast_location::planarPoseMatrix({2.0, 0.0, 0.0});

  const auto map_from_odom =
    fast_location::composeMapToOdom(map_from_pcd, pcd_from_odom);

  EXPECT_NEAR(map_from_odom(0, 3), 10.0, 1e-5);
  EXPECT_NEAR(map_from_odom(1, 3), 1.0, 1e-5);
  EXPECT_NEAR(map_from_odom(0, 0), 0.0, 1e-5);
  EXPECT_NEAR(map_from_odom(1, 0), 1.0, 1e-5);
}

TEST(FrameTransformsTest, RecoversPcdToOdomFromMapInitialPose)
{
  const auto map_from_pcd = fast_location::planarPoseMatrix({10.0, 0.0, 0.0});
  const auto map_from_base = fast_location::planarPoseMatrix({13.0, 0.0, 0.0});
  const auto odom_from_base = fast_location::planarPoseMatrix({1.0, 0.0, 0.0});

  const auto pcd_from_odom = fast_location::initialPoseToPcdOdom(
    map_from_pcd, map_from_base, odom_from_base);

  EXPECT_NEAR(pcd_from_odom(0, 3), 2.0, 1e-5);
  EXPECT_NEAR(pcd_from_odom(1, 3), 0.0, 1e-5);
}

TEST(FrameTransformsTest, DetectsNonFiniteTransforms)
{
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform(0, 3) = std::numeric_limits<float>::quiet_NaN();

  EXPECT_FALSE(fast_location::isFiniteTransform(transform));
  EXPECT_TRUE(fast_location::isFiniteTransform(Eigen::Matrix4f::Identity()));
}

TEST(FrameTransformsTest, IdentityMapAlignmentPreservesPcdLocalization)
{
  const auto pcd_from_odom = fast_location::planarPoseMatrix({3.0, -2.0, 0.4});

  const auto map_from_odom = fast_location::composeMapToOdom(
    Eigen::Matrix4f::Identity(), pcd_from_odom);

  EXPECT_TRUE(map_from_odom.isApprox(pcd_from_odom, 1e-6f));
}

TEST(FrameTransformsTest, InitialPoseConversionHandlesRotatedPcdMap)
{
  constexpr double kHalfPi = 1.5707963267948966;
  const auto map_from_pcd = fast_location::planarPoseMatrix({5.0, 2.0, kHalfPi});
  const auto expected_pcd_from_odom = fast_location::planarPoseMatrix({2.0, 1.0, -0.2});
  const auto odom_from_base = fast_location::planarPoseMatrix({0.5, -0.2, 0.1});
  const auto map_from_base = map_from_pcd * expected_pcd_from_odom * odom_from_base;

  const auto actual = fast_location::initialPoseToPcdOdom(
    map_from_pcd, map_from_base, odom_from_base);

  EXPECT_TRUE(actual.isApprox(expected_pcd_from_odom, 1e-5f));
}

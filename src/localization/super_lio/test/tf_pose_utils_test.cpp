#include <cmath>
#include <limits>

#include <gtest/gtest.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#include "ros/tf_pose_utils.hpp"

namespace
{

tf2::Transform makeTransform(double x, double y, double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::Transform(q, tf2::Vector3(x, y, 0.0));
}

}

TEST(TfPoseUtilsTest, ComposesSensorPoseWithTimeAlignedSensorToBaseTransform)
{
  const tf2::Transform odom_to_livox = makeTransform(2.0, 1.0, 0.5);
  const tf2::Transform livox_to_base = makeTransform(0.4, -0.2, -0.5);
  tf2::Transform odom_to_base;

  ASSERT_TRUE(LI2Sup::composeLidarOdomToBase(
      odom_to_livox, livox_to_base, odom_to_base));

  EXPECT_NEAR(odom_to_base.getOrigin().x(), 2.4469181325, 1e-6);
  EXPECT_NEAR(odom_to_base.getOrigin().y(), 1.0162537031, 1e-6);
  EXPECT_NEAR(odom_to_base.getRotation().getAngle(), 0.0, 1e-6);
}

TEST(TfPoseUtilsTest, ComposesImuPoseWithMid360LidarExtrinsic)
{
  const tf2::Transform odom_to_imu = makeTransform(1.0, 2.0, 0.0);
  const tf2::Transform imu_to_lidar(
      tf2::Quaternion(0.0, 0.0, 0.0, 1.0),
      tf2::Vector3(-0.011, -0.02329, 0.04412));
  tf2::Transform odom_to_lidar;

  ASSERT_TRUE(LI2Sup::composeImuPoseToLidar(
      odom_to_imu, imu_to_lidar, odom_to_lidar));
  EXPECT_NEAR(odom_to_lidar.getOrigin().x(), 0.989, 1e-9);
  EXPECT_NEAR(odom_to_lidar.getOrigin().y(), 1.97671, 1e-9);
  EXPECT_NEAR(odom_to_lidar.getOrigin().z(), 0.04412, 1e-9);
}

TEST(TfPoseUtilsTest, RotatesLidarExtrinsicByImuOrientation)
{
  const tf2::Transform odom_to_imu =
      makeTransform(1.0, 2.0, 1.5707963267948966);
  const tf2::Transform imu_to_lidar(
      tf2::Quaternion(0.0, 0.0, 0.0, 1.0),
      tf2::Vector3(0.1, 0.0, 0.0));
  tf2::Transform odom_to_lidar;

  ASSERT_TRUE(LI2Sup::composeImuPoseToLidar(
      odom_to_imu, imu_to_lidar, odom_to_lidar));
  EXPECT_NEAR(odom_to_lidar.getOrigin().x(), 1.0, 1e-9);
  EXPECT_NEAR(odom_to_lidar.getOrigin().y(), 2.1, 1e-9);
}

TEST(TfPoseUtilsTest, RejectsNonFiniteImuToLidarExtrinsic)
{
  const tf2::Transform odom_to_imu = makeTransform(0.0, 0.0, 0.0);
  tf2::Transform imu_to_lidar = makeTransform(0.0, 0.0, 0.0);
  imu_to_lidar.getOrigin().setZ(
      std::numeric_limits<double>::quiet_NaN());
  tf2::Transform odom_to_lidar;

  EXPECT_FALSE(LI2Sup::composeImuPoseToLidar(
      odom_to_imu, imu_to_lidar, odom_to_lidar));
}

TEST(TfPoseUtilsTest, RejectsNonFiniteTransforms)
{
  tf2::Transform odom_to_livox = makeTransform(0.0, 0.0, 0.0);
  odom_to_livox.getOrigin().setX(std::numeric_limits<double>::quiet_NaN());
  const tf2::Transform livox_to_base = makeTransform(0.0, 0.0, 0.0);
  tf2::Transform odom_to_base;

  EXPECT_FALSE(LI2Sup::composeLidarOdomToBase(
      odom_to_livox, livox_to_base, odom_to_base));
}

// Copyright (c) 2026 colacondition
// Licensed under the MIT License. See LICENSE in the project root.
#include "fake_vel_transform/velocity_transform.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace fake_vel_transform
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kTol = 1e-12;

Twist2D makeTwist(double x, double y, double wz = 0.0)
{
  Twist2D twist;
  twist.linear_x = x;
  twist.linear_y = y;
  twist.angular_z = wz;
  return twist;
}

// R(angle) 作用在平面向量上的参考实现（与 header 里手写的 cos/sin 公式对拍）。
void rotateReference(double angle, double x, double y, double & out_x, double & out_y)
{
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  out_x = c * x - s * y;
  out_y = s * x + c * y;
}

TEST(VelocityTransform, RotationMatchesRotationMatrix)
{
  const Twist2D in = makeTwist(1.3, -0.7, 0.2);
  for (const double angle : {-kPi, -kPi / 2.0, -0.7, 0.0, 0.3, kPi / 2.0, kPi}) {
    const Twist2D out = rotateFakeToBase(in, angle);
    double expected_x = 0.0;
    double expected_y = 0.0;
    rotateReference(angle, in.linear_x, in.linear_y, expected_x, expected_y);
    EXPECT_NEAR(out.linear_x, expected_x, kTol) << "angle=" << angle;
    EXPECT_NEAR(out.linear_y, expected_y, kTol) << "angle=" << angle;
    // 平面旋转不改变角速度。
    EXPECT_DOUBLE_EQ(out.angular_z, in.angular_z);
  }
}

TEST(VelocityTransform, RotationPreservesNorm)
{
  const Twist2D in = makeTwist(1.7, -2.3);
  const Twist2D out = rotateFakeToBase(in, 1.234);
  EXPECT_NEAR(std::hypot(out.linear_x, out.linear_y), std::hypot(in.linear_x, in.linear_y), kTol);
}

// 端到端代数钉死：MPC 在 fake 系下发 v_F = R(φ)ᵀ·v_world，节点按
// v_B = R(φ-β)·v_F 旋转，两者复合必须等于云台系所需的 R(β)ᵀ·v_world =
// R(-β)·v_world。 这一条同时覆盖「angle_diff
// 符号」和「旋转顺序」：写反任何一处都会失败。
TEST(VelocityTransform, RotationCompositionMatchesWorldToBase)
{
  const double base_yaw = 0.7;   // β
  const double fake_yaw = -0.4;  // φ
  const double v_world_x = 1.2;
  const double v_world_y = 0.9;

  // MPC 侧：世界系期望速度转进 fake 系。
  double fake_x = 0.0;
  double fake_y = 0.0;
  rotateReference(-fake_yaw, v_world_x, v_world_y, fake_x, fake_y);
  const Twist2D fake_cmd = makeTwist(fake_x, fake_y);

  // 节点侧：offset = φ - β，TF 有效。
  const FrameTransform2D tf{true, fake_yaw - base_yaw};
  const Twist2D chassis = computeChassisTwist(fake_cmd, tf);

  // 期望：云台/底盘系速度 = R(-β)·v_world。
  double expected_x = 0.0;
  double expected_y = 0.0;
  rotateReference(-base_yaw, v_world_x, v_world_y, expected_x, expected_y);
  EXPECT_NEAR(chassis.linear_x, expected_x, kTol);
  EXPECT_NEAR(chassis.linear_y, expected_y, kTol);
}

// 还没收到 /plan 时 base_link_fake 与 base_link 重合（offset=0）：纯恒等透传。
TEST(VelocityTransform, IdentityWhenFakeFrameNotInitialized)
{
  const Twist2D in = makeTwist(0.8, 0.3, 0.05);
  const FrameTransform2D tf{true, 0.0};
  const Twist2D out = computeChassisTwist(in, tf);
  EXPECT_NEAR(out.linear_x, in.linear_x, kTol);
  EXPECT_NEAR(out.linear_y, in.linear_y, kTol);
  EXPECT_DOUBLE_EQ(out.angular_z, in.angular_z);
}

TEST(VelocityTransform, ZeroCommandStaysZero)
{
  const Twist2D out = computeChassisTwist(Twist2D{}, FrameTransform2D{true, 0.9});
  EXPECT_DOUBLE_EQ(out.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(out.linear_y, 0.0);
  EXPECT_DOUBLE_EQ(out.angular_z, 0.0);
}

// 角速度原样透传：模型是运动云台，节点不覆盖/不丢弃角速度指令。
TEST(VelocityTransform, AngularPassesThrough)
{
  const Twist2D in = makeTwist(0.6, 0.0, 0.8);
  const Twist2D out = computeChassisTwist(in, FrameTransform2D{true, 0.3});
  EXPECT_NEAR(out.angular_z, 0.8, kTol);
}

// 线速度旋转与角速度解耦：任意 offset 下角速度都不变。
TEST(VelocityTransform, RotationKeepsAngularForAnyOffset)
{
  const Twist2D in = makeTwist(1.0, 0.0, -1.2);
  for (const double offset : {-kPi, -1.0, 0.0, 0.5, kPi}) {
    const Twist2D out = computeChassisTwist(in, FrameTransform2D{true, offset});
    EXPECT_DOUBLE_EQ(out.angular_z, in.angular_z) << "offset=" << offset;
  }
}

// TF 失败（lookupTransform 抛异常 / yaw 非有限）时输出零速：fail-stop。
TEST(VelocityTransform, InvalidTransformZeroesOutput)
{
  const Twist2D in = makeTwist(1.0, 0.5, 0.2);

  const Twist2D out = computeChassisTwist(in, FrameTransform2D{});
  EXPECT_DOUBLE_EQ(out.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(out.linear_y, 0.0);
  EXPECT_DOUBLE_EQ(out.angular_z, 0.0);

  const Twist2D out_nonfinite = computeChassisTwist(in, FrameTransform2D{true, std::nan("")});
  EXPECT_DOUBLE_EQ(out_nonfinite.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(out_nonfinite.angular_z, 0.0);
}

// 输入本身非有限时也不能把 NaN 泄露给底盘。
TEST(VelocityTransform, NonFiniteCommandZeroesOutput)
{
  const FrameTransform2D tf{true, 0.5};
  const Twist2D out = computeChassisTwist(makeTwist(std::nan(""), 0.0, 0.0), tf);
  EXPECT_DOUBLE_EQ(out.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(out.linear_y, 0.0);
  EXPECT_DOUBLE_EQ(out.angular_z, 0.0);
}

// TF 短失败保持上一拍偏置；首拍失败时偏置为 0（base_link_fake 与 base_link
// 重合）。
TEST(VelocityTransform, OffsetHoldsLastValueOnFailedMeasurement)
{
  EXPECT_DOUBLE_EQ(offsetAfterMeasurement(false, 0.9, 0.0), 0.0);
  EXPECT_DOUBLE_EQ(offsetAfterMeasurement(true, 0.4, 0.0), 0.4);
  EXPECT_DOUBLE_EQ(offsetAfterMeasurement(false, 0.9, 0.4), 0.4);
  EXPECT_DOUBLE_EQ(offsetAfterMeasurement(true, std::nan(""), 0.4), 0.4);
  EXPECT_DOUBLE_EQ(offsetAfterMeasurement(false, 0.0, std::nan("")), 0.0);
}

}  // namespace
}  // namespace fake_vel_transform

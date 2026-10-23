// Copyright (c) 2026 colacondition
// Licensed under the MIT License. See LICENSE in the project root.
#ifndef FAKE_VEL_TRANSFORM__VELOCITY_TRANSFORM_HPP_
#define FAKE_VEL_TRANSFORM__VELOCITY_TRANSFORM_HPP_

// 虚拟云台速度转换的纯函数（无 ROS 依赖，可在 gtest 里直接验证）。
//
// 模型约定（本工程明确采用）
// ==========================
// 整车被简化为**一个可在地面平移的运动云台**：导航只关心云台参考点的 XY 平移，
// 不做底盘小陀螺/物理轮组 yaw 的建模，电控负责云台到底盘、全向轮解算与执行。
// 因此本节点只做一件事：把导航下发的"虚拟云台系"速度换算到"实际云台/雷达系"，
// 不注入任何自旋角速度。
//
// 坐标系
// ======
//   M = map（规划系）；B = base_link（雷达/云台实际 yaw）；
//   F = base_link_fake（虚拟云台，yaw 跟 /plan 的 XY 切向）。
//
// 记 β = yaw(M->B)，φ = yaw(M->F)，offset = φ - β。fake_vel_transform 发布的 TF
// 是 base_link -> base_link_fake，旋转角就是 offset，因此恒有 φ = β + offset。
//
// MPC 的 robot_base_frame=base_link_fake：它按 φ 跟踪路径，输出的 /cmd_vel 是
// **F 系**下的云台速度 v_F。速度平滑器只限幅不换系，所以 fake 节点拿到的是
// v_F。 底盘/云台实际需要的是 **B 系**速度：
//
//   v_F = R(φ)ᵀ · v_M          （MPC 把世界系期望速度转进 F 系）
//   v_B = R(β)ᵀ · v_M          （B 系实际需要执行的速度）
//   => v_B = R(β)ᵀ · R(φ) · v_F = R(φ - β) · v_F = R(offset) · v_F
//
// 即 offset = φ - β 时按 R(+offset) 旋转。历史代码写的是
//   angle_diff = -offset; out = R(-angle_diff) · v_F
// 而 R(-angle_diff) == R(+offset)，所以**符号与顺序都是对的**；本 header 把它
// 写成显式的 R(offset) 并配了组合验证用例（见 test/velocity_transform_test.cpp
// 的 RotationCompositionMatchesWorldToBase），把这条推导钉死。
//
// 角速度
// ======
// 绕 z 的角速度在平面刚体旋转下不变：ω_B = ω_F =
// cmd.angular_z（已由平滑器限幅）， 原样透传，不参与 R(offset)
// 变换，也不做任何覆盖。

#include <cmath>

namespace fake_vel_transform
{

// 平面速度指令（线速度 x/y + 角速度 z），对应 geometry_msgs/Twist 的三个分量。
struct Twist2D
{
  double linear_x{0.0};
  double linear_y{0.0};
  double angular_z{0.0};
};

// 一次 base_link -> base_link_fake TF 查询的结果建模成值类型：
// valid=false 表示 lookupTransform 失败或 yaw 非有限 —— 调用方必须发零速。
struct FrameTransform2D
{
  bool valid{false};
  double fake_from_base_yaw{0.0};
};

inline bool isFinite(const Twist2D & twist)
{
  return std::isfinite(twist.linear_x) && std::isfinite(twist.linear_y) &&
         std::isfinite(twist.angular_z);
}

// 显式全零判定。零速不必查 TF：直接回零，避免 TF 尚未就绪时刷超时告警。
inline bool isZeroCommand(const Twist2D & twist, double eps = 1e-6)
{
  return std::abs(twist.linear_x) < eps && std::abs(twist.linear_y) < eps &&
         std::abs(twist.angular_z) < eps;
}

// R(offset) 旋转：把虚拟云台系速度转到实际云台/雷达系，v_B = R(offset) · v_F。
inline Twist2D rotateFakeToBase(const Twist2D & fake, double fake_from_base_yaw)
{
  const double c = std::cos(fake_from_base_yaw);
  const double s = std::sin(fake_from_base_yaw);
  Twist2D base;
  base.linear_x = c * fake.linear_x - s * fake.linear_y;
  base.linear_y = s * fake.linear_x + c * fake.linear_y;
  // 平面旋转不改变绕 z 的角速度。
  base.angular_z = fake.angular_z;
  return base;
}

// 核心纯函数：F 系速度指令 -> B 系执行指令。
//
// 安全语义（与节点回调一致，抽出来是为了单测）：
//   - TF 不可用 / yaw 非有限 / 输入非有限 ->
//   零速（fail-stop，宁可停车也不带坏偏置走）；
//   - 输入全零 -> 零速；
//   - 线速度按 R(offset) 旋转；
//   - 角速度原样透传（不做小陀螺注入/覆盖）。
inline Twist2D computeChassisTwist(const Twist2D & fake_cmd, const FrameTransform2D & tf)
{
  if (!tf.valid || !std::isfinite(tf.fake_from_base_yaw) || !isFinite(fake_cmd)) {
    return Twist2D{};
  }
  if (isZeroCommand(fake_cmd)) {
    return Twist2D{};
  }
  return rotateFakeToBase(fake_cmd, tf.fake_from_base_yaw);
}

// TF 短失败时保持上一拍偏置，而不是把 fake 航向归零（归零会让轨迹瞬间旋转、
// 且 100 Hz 播音时看起来像抖动）。首次失败时 previous=0 即初始 base_link_fake
// 与 base_link 重合（恒等偏置）。measured 非有限同样视为无效测量。
inline double offsetAfterMeasurement(
  bool valid_measurement, double measured_offset, double previous_offset)
{
  if (!valid_measurement || !std::isfinite(measured_offset)) {
    return std::isfinite(previous_offset) ? previous_offset : 0.0;
  }
  return measured_offset;
}

}  // namespace fake_vel_transform

#endif  // FAKE_VEL_TRANSFORM__VELOCITY_TRANSFORM_HPP_

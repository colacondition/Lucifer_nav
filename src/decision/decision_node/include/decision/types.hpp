#pragma once

#include <cstdint>
#include <string>

namespace decision
{

enum class TargetName
{
  WaitHome,
  WaitHp,
  Home,
  Patrol,
  Center,
  WaitCenter,
  Unknown
};

enum class TargetMode
{
  Disabled,
  ExecutorFollow,
  ExecutorThrough
};

// 从哨兵自身信号推断出的交战态势。
enum class EngagementState
{
  Calm,        // 持续无掉血无开火：平静占区
  Engaging,    // 在开火或中等掉血：正面交战
  Suppressed   // 剧烈掉血但没开火：被压制/被侧后偷
};

// 战斗态势评估结果，由 DecisionContext 依据 is_attacked / shooter_heat 直接判定。
struct CombatAssessment
{
  bool valid{false};              // 数据是否足够做判断
  bool firing{false};             // shooter_heat 是否高于开火阈值（正在开火）
  bool hit{false};                // 是否正在挨打（is_attacked）
  EngagementState state{EngagementState::Calm};
};

std::string toString(EngagementState state);

struct Pose
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double qx{0.0};
  double qy{0.0};
  double qz{0.0};
  double qw{1.0};
};

struct TimedPose
{
  Pose pose;
  double stamp_sec{-1.0e9};
  std::string source;
};

std::string toString(TargetName target);
TargetName targetNameFromString(const std::string & value);

}  // namespace decision

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
  // 回家两条路线：复活后（无敌/虚弱）走隧道快线；残血回家走绕远安全线。
  // 两者都是"回家"目标：会清 center 段、成功到达置 home 段。
  HomeTunnel,
  HomeSafe,
  Patrol,
  Center,
  WaitCenter,
  // 追击时下发的目标名。
  //
  // ⚠️ 必须和 Patrol **分开**：两者共用 TargetName 的话，结果回调里"走完一组"那套
  // 记账会被互相污染 —— 追击的每条路径走完都会把巡逻的下次触发推后 group_gap_sec，
  // 于是目标消失后车会先站一会儿才恢复巡逻。
  //
  // 也不能用 WaitCenter：那是"守点型"目标（见 targetNeedsMaintainGoal），
  // 会被守点纠偏按 0.35m 容差每 0.8s 拽回锚点。
  Chase,
  // 中心占领区的四个角点（range.csv）。它**不是导航目标**，永远不会被下发 ——
  // 挂成 TargetName 只是为了复用现有的 CSV 读取/缓存/路径传递那一套，
  // 免得再写第二份解析器、再开第二条传路径的路。
  Zone,
  Unknown
};

enum class TargetMode
{
  Disabled,
  ExecutorFollow,
  ExecutorThrough
};


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

// 所有"回家"目标（Home / HomeTunnel / HomeSafe）共享的判定。
inline bool isHomeTarget(TargetName target)
{
  return target == TargetName::Home || target == TargetName::HomeTunnel ||
         target == TargetName::HomeSafe;
}

std::string toString(TargetName target);
TargetName targetNameFromString(const std::string & value);

}  // namespace decision

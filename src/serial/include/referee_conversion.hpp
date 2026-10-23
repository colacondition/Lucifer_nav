#pragma once

#include <cstdint>

#include <decision_interfaces/msg/game_status.hpp>
#include <decision_interfaces/msg/robot_status.hpp>

#include "packet.hpp"

namespace serial_driver
{

// 3v3 哨兵的固定身份。协议 0x0201 里有 robot_id / robot_level，但决策层只用哨兵自己的血量，
// 不需要这两个值 —— 固定成常量，省掉电控两路解析和帧里两个字节。
constexpr uint8_t kSentryRobotId = 7;

// 3v3 的比赛类型值（协议 0x0001 bit 0-3）。用来做一次「确认接的是 3v3 那套规则」的校验。
constexpr uint8_t kGameTypeRmUl3v3 = 4;

// game_type_progress 的拆位。协议原生布局是低 4 bit 比赛类型、高 4 bit 比赛阶段，
// 线上原样透传，拆位放在这一侧 —— 电控不用做移位。
inline uint8_t gameTypeOf(uint8_t game_type_progress)
{
  return static_cast<uint8_t>(game_type_progress & 0x0FU);
}

inline uint8_t gameProgressOf(uint8_t game_type_progress)
{
  return static_cast<uint8_t>((game_type_progress >> 4U) & 0x0FU);
}

// 血量差分。抽成函数是为了让「上一拍 → 这一拍」这条判据能单测。
inline bool hpDecreased(uint16_t previous_hp, uint16_t current_hp)
{
  return current_hp < previous_hp;
}

// 线上帧 -> 比赛状态。
inline decision_interfaces::msg::GameStatus toGameStatus(const RefereePacketV2 & packet)
{
  decision_interfaces::msg::GameStatus msg;
  msg.game_type = gameTypeOf(packet.game_type_progress);
  msg.game_progress = gameProgressOf(packet.game_type_progress);
  msg.stage_remain_time = packet.stage_remain_time;
  return msg;
}

// 线上帧 -> 自身状态。
//
// is_attacked 由调用方传入：它是「这一拍血量比上一拍低」的**跨帧**差分，而这一帧本身
// 只带一个瞬时血量，函数没法从里面推出来。判定逻辑见 hpDecreased()。
inline decision_interfaces::msg::RobotStatus toRobotStatus(
  const RefereePacketV2 & packet, bool is_attacked)
{
  decision_interfaces::msg::RobotStatus msg;
  msg.robot_id = kSentryRobotId;
  msg.current_hp = packet.sentry_hp;
  msg.is_attacked = is_attacked;
  return msg;
}

}  // namespace serial_driver

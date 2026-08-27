#pragma once

#include <cstdint>
#include <cstring>

namespace serial_driver
{

// NUC -> MCU chassis velocity command.
struct ChassisCommandPacket
{
  uint8_t head[2] = {'H', 'L'};
  float vel_x = 0.0f;
  float vel_y = 0.0f;
  float vel_w = 0.0f;
  uint16_t crc16 = 0;
} __attribute__((packed));

// NUC -> MCU 云台姿态指令。这一帧按固定周期发，平时 gimbal_toggle = 0（什么都不做），
// 需要换姿态时发**一帧** 1，电控收到就把云台切到另一个姿态。进洞前发一次 1（收下来），
// 出洞后再发一次 1（立起来）—— 两次都是同一个值，因为它是「切换」而不是「设成低」。
//
// 注意这是脉冲语义，不是电平语义：gimbal_toggle 不能理解成「想要的姿态」。连续发 1 会让
// 云台反复切换，所以驱动那侧只在请求翻转的那一个周期发 1（见 syncGimbalPosture）。
//
// 脉冲不幂等，所以这一帧丢了没法靠重发补 —— 但它丢了不会悄无声息：上行的
// GimbalPostureStatePacket 会一直报「高」，MPC 看到请求收而实测没收就把车停在洞外。
// 代价是车不动，而不是云台撞顶板。
//
// 为什么单独一帧，不搭在 ChassisCommandPacket 上：
//   1. 底盘帧是事件驱动的，只在 /cmd_vel_chassis 来消息时才发。车在洞里停住（到点、
//      或者 MPC 进恢复停发速度）时底盘帧就断了，搭在上面的字节跟着断 —— 而那正是最
//      需要能发出切换指令的时候。这一帧走自己的节奏，跟底盘发不发无关。
//   2. 底盘帧的字节布局不动，固件那侧已经跑通的解析一行都不用改。
//
// 帧头用 'H','G' 而不是底盘的 'H','L'：两种 NUC->MCU 帧长度不同（5 vs 16），同头的话
// 固件在字节流里没法区分，只能靠长度猜，一次错位就再也同步不回来。
//
// gimbal_toggle 声明成 uint8_t 而不是 bool：bool 的字节宽度是 ABI 约定的，跨到 MCU 的
// 编译器上没有保证。这里的宽度必须写死。
struct GimbalPosturePacket
{
  uint8_t head[2] = {'H', 'G'};
  uint8_t gimbal_toggle = 0;  // 0 = 不动，1 = 切换到另一个姿态（发一帧就够）
  uint16_t crc16 = 0;
} __attribute__((packed));

// MCU -> NUC 云台姿态回传。电控持续发，报告云台**实际**在哪个姿态。
//
// 帧头 'H','P'：跟裁判数据帧（'H','L'）和下行的姿态请求帧（'H','G'）都不同。三种帧
// 长度各异，靠第二个字节区分帧型，长度由帧型决定 —— 而不是反过来靠长度猜帧型。
//
// **极性跟下行的请求帧相反**：这里 0 = 低（已收下）、1 = 高（立着），是电控那侧定的。
// 所以字段名叫 gimbal_raised 而不是 lowered —— 名字跟线上的含义一字对应。取名叫
// lowered 的话 0 会被读成「没收下来」，而它实际是「已经收下来了」，错的方向恰好是
// 「以为云台没收、白等一场」或者反过来「以为收好了、直接进洞」。
// 转成 ROS 侧的 lowered 语义只在 serial_driver 里做一次。
struct GimbalPostureStatePacket
{
  uint8_t head[2] = {'H', 'P'};
  uint8_t gimbal_raised = 1;  // 0 = 低（已收下），1 = 高（立着）
  uint16_t crc16 = 0;
} __attribute__((packed));

// 线上极性 -> 导航侧的 lowered 语义。全工程只有这一处做这个翻转，其他地方一律拿
// lowered 说话。写成函数而不是在驱动里内联一句 `== 0`，是为了让它可测 —— 极性是电控
// 那侧定的约定，翻错了两个方向都不报错：要么白等一场，要么带着立起的云台进洞。
inline bool isGimbalLowered(const GimbalPostureStatePacket & packet)
{
  return packet.gimbal_raised == 0;
}

// MCU -> NUC simulated referee data.
struct DecisionPacket
{
  uint8_t head[2] = {'H', 'L'};
  uint16_t current_hp = 0;
  uint8_t game_progress = 0;
  uint16_t stage_remain_time = 0;
  uint16_t crc16 = 0;
} __attribute__((packed));

template<typename T>
inline T bufferToStruct(const uint8_t * buffer)
{
  T result{};
  std::memcpy(&result, buffer, sizeof(T));
  return result;
}

template<typename T>
inline void structToBuffer(const T & inputStruct, uint8_t * outputArray)
{
  std::memcpy(outputArray, reinterpret_cast<const uint8_t *>(&inputStruct), sizeof(T));
}

}  // namespace serial_driver

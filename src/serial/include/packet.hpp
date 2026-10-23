#pragma once

#include <cstdint>
#include <cstring>

namespace serial_driver
{

// ==================== 帧头 ====================
// 上行（MCU -> NUC）和下行（NUC -> MCU）都以 'H' 开头，用第二个字节定帧型，帧型再定长度。
// 不能反过来靠长度猜帧型：一次错位就再也同步不回来。
constexpr uint8_t kFrameHead0 = static_cast<uint8_t>('H');
// 上行：裁判数据（变长，帧内带 payload_len）。
// v1 的定长 'H','L' 裁判帧已删除 —— 它的三个字段（自身血量、比赛阶段、剩余时间）
// 已经并进这一帧，上行只剩这一个帧型。
constexpr uint8_t kRefereeHead1 = static_cast<uint8_t>('R');
// 上行：云台姿态回传
constexpr uint8_t kPostureStateHead1 = static_cast<uint8_t>('P');
// 下行：底盘速度
constexpr uint8_t kChassisHead1 = static_cast<uint8_t>('L');
// 下行：云台切换脉冲
constexpr uint8_t kGimbalHead1 = static_cast<uint8_t>('G');

// ==================== 上行裁判帧 ====================
// 只装决策层**真正在用**的字段。当前只用三个：
//   game_type_progress / stage_remain_time  —— 算中心走位的激进度、判比赛是否进行
//   sentry_hp                               —— 低血回家
//
// 曾经列过又砍掉的（中心占领状态、队友血量、伤害差、射击热量、中心 RFID）都因为决策层
// 简化后不再需要而删除。以后要加回来**追加在尾部**并改 payload_len 即可，不需要改帧型、
// 也不会错位 —— 这正是这个帧留 payload_len 的原因。
//
// 为什么没有 valid_mask：NUC 侧的帧是**原子**的 —— CRC 过了就说明整帧完好，不存在
// 「半帧」；而各路数据由电控**保持上次收到的值**，不会每帧清零。两条合起来，
// 「帧收到了」就等于「每一路都有定义值」，再挂一个有效位是重复信息。
//
// ⚠️ 因此对电控有两条硬要求：
//   1. **每路数据保持上次收到的值**，不要因为这一拍没收到对应裁判报文就填 0。
//   2. 两路数据刷新率不同（0x0201 是 10Hz、0x0001 是 1Hz），各自维护缓存，
//      对应的报文到了才更新它。
constexpr uint8_t kRefereePayloadLen = 5;
// 防御性上界：一个被干扰的 payload_len 可以让我们无限等帧，必须挡住。
constexpr uint8_t kRefereeMaxPayloadLen = 48;

// 帧总长 = head(2) + payload_len(1) + payload + crc16(2)。
// 做成函数而不是 sizeof(struct)：payload_len 变大时总长跟着变，解析器按帧内的
// len 算长度和 CRC 偏移，所以以后往尾部追加字段不需要改帧型、也不会错位。
constexpr std::size_t refereeFrameSize(uint8_t payload_len)
{
  return static_cast<std::size_t>(payload_len) + 5U;
}

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

// MCU -> NUC 裁判数据（变长）。字段一一对应官方协议，来源见行尾注释。
//
// 线上布局（小端）：
//   偏移 0-1   'H' 'R'
//   偏移 2     payload_len          = 5（len 之后、CRC 之前的字节数）
//   偏移 3     game_type_progress   0x0001 的 byte 0 **原样搬过来**（电控不做位移）
//   偏移 4-5   stage_remain_time    0x0001 偏移 1
//   偏移 6-7   sentry_hp            0x0201 偏移 2
//   偏移 8-9   crc16
//
// game_type_progress 一个字节同时装两个 4 bit（协议原生布局）：
//   低 4 bit = game_type（3v3 应为 4）  高 4 bit = game_progress
// 原样透传交给 NUC 拆位，电控那一侧不用做移位。
//
// ⚠️ payload_len > 5 时，CRC 的位置不再是 offsetof(crc16) —— 校验由解析器按 len 算出的
// 总长完成，**不要拿这个结构体的 crc16 字段当校验值用**。
struct RefereePacketV2
{
  uint8_t head[2] = {'H', 'R'};
  uint8_t payload_len = kRefereePayloadLen;
  uint8_t game_type_progress = 0;
  uint16_t stage_remain_time = 0;
  uint16_t sentry_hp = 0;
  uint16_t crc16 = 0;
} __attribute__((packed));

// 收帧扫描的结论。
enum class FrameScanStatus
{
  Ok,         // frame_size 有效，再确认缓冲里有这么多字节就能取帧
  NeedMore,   // 数据不够，等下一批字节（**不要丢字节**）
  NotAFrame,  // 这个 'H' 是数据里碰巧出现的，调用方丢 1 字节重新同步
};

struct FrameScanResult
{
  FrameScanStatus status{FrameScanStatus::NeedMore};
  std::size_t frame_size{0};
};

// 从缓冲头部判定这一帧有多长。抽成纯函数是为了能脱离节点测：
// 「帧型定长度」这套规则一旦飘了，表现是回传永远收不到、日志里干干净净，
// 留在节点私有方法里没有任何测试拦得住。
inline FrameScanResult scanFrame(const uint8_t * buffer, std::size_t available)
{
  if (available < 2U) {
    return {FrameScanStatus::NeedMore, 0U};
  }
  if (buffer[0] != kFrameHead0) {
    return {FrameScanStatus::NotAFrame, 0U};
  }

  // 帧型由第二个字节决定，长度再由帧型决定 —— 不能反过来靠长度猜帧型。
  if (buffer[1] == kRefereeHead1) {
    if (available < 3U) {
      return {FrameScanStatus::NeedMore, 0U};  // 还看不到 payload_len
    }
    const uint8_t payload_len = buffer[2];
    if (payload_len < kRefereePayloadLen || payload_len > kRefereeMaxPayloadLen) {
      // len 不合理：这个 'H','R' 不是帧头。不能按一个荒谬的长度去等，否则会被
      // 一段噪声钉死在这里。
      return {FrameScanStatus::NotAFrame, 0U};
    }
    return {FrameScanStatus::Ok, refereeFrameSize(payload_len)};
  }
  if (buffer[1] == kPostureStateHead1) {
    return {FrameScanStatus::Ok, sizeof(GimbalPostureStatePacket)};
  }
  return {FrameScanStatus::NotAFrame, 0U};
}

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

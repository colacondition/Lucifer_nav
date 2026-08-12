// 云台姿态帧的线上格式测试。
//
// 这一帧的每个字节都要跟固件那侧对上，而对不上的表现是「电控收到一堆 CRC 不过的垃圾」
// 或者更糟 ——「解析成了另一种帧，云台在错误的时候动作」。所以这里锁的不是行为而是布局：
// 帧头、字段偏移、总长度、CRC 覆盖范围。
//
// 为什么值得单独测：sizeof 和偏移是编译器行为，__attribute__((packed)) 掉了、字段顺序
// 被调了、有人把 uint8_t 改成 bool —— 这些改动都能编译通过，都会在实车上表现成偶发的
// 云台异常，而且没有任何一层会报错。

#include <cstdint>
#include <cstring>

#include <gtest/gtest.h>

#include "crc.hpp"
#include "packet.hpp"

namespace serial_driver
{
namespace
{

TEST(GimbalPosturePacket, LayoutIsFiveBytesWithNoPadding)
{
  // 2 字节帧头 + 1 字节标志 + 2 字节 CRC。多一个字节就是固件那侧读错位。
  EXPECT_EQ(sizeof(GimbalPosturePacket), 5u);
  EXPECT_EQ(offsetof(GimbalPosturePacket, head), 0u);
  EXPECT_EQ(offsetof(GimbalPosturePacket, gimbal_toggle), 2u);
  EXPECT_EQ(offsetof(GimbalPosturePacket, crc16), 3u);
  // 宽度写死，不能跟着 ABI 的 bool 变。
  EXPECT_EQ(sizeof(GimbalPosturePacket{}.gimbal_toggle), 1u);
}

TEST(GimbalPosturePacket, DefaultToggleDoesNothing)
{
  // 默认值必须是 0。默认成 1 的话，任何一个没显式赋值就发出去的帧都会让云台切换一次 ——
  // 而指令是脉冲语义，切错了就是姿态反了，车带着立起的云台进洞。
  const GimbalPosturePacket packet{};
  EXPECT_EQ(packet.gimbal_toggle, 0u);
}

TEST(GimbalPosturePacket, HeadDiffersFromChassisFrame)
{
  // 两种 NUC->MCU 帧长度不同（5 vs 16）。帧头相同的话固件在字节流里只能靠长度猜，
  // 一次错位就再也同步不回来 —— 所以第二个字节必须不一样。
  const GimbalPosturePacket posture{};
  const ChassisCommandPacket chassis{};
  EXPECT_EQ(posture.head[0], chassis.head[0]);   // 'H' 保持一致，方便固件先同步
  EXPECT_NE(posture.head[1], chassis.head[1]);
  EXPECT_EQ(posture.head[0], static_cast<uint8_t>('H'));
  EXPECT_EQ(posture.head[1], static_cast<uint8_t>('G'));
  EXPECT_NE(sizeof(GimbalPosturePacket), sizeof(ChassisCommandPacket));
}

TEST(GimbalPostureStatePacket, LayoutIsFiveBytesWithNoPadding)
{
  // 2 字节帧头 + 1 字节姿态 + 2 字节 CRC = 5，中间不能有填充。驱动是按 sizeof 从字节流里
  // 切帧的，编译器塞一个字节进来就会跟固件的布局错开，而错开的表现是校验失败、姿态永远
  // 读不到，不是编译错误。
  EXPECT_EQ(sizeof(GimbalPostureStatePacket), 5u);
  EXPECT_EQ(offsetof(GimbalPostureStatePacket, head), 0u);
  EXPECT_EQ(offsetof(GimbalPostureStatePacket, gimbal_raised), 2u);
  EXPECT_EQ(offsetof(GimbalPostureStatePacket, crc16), 3u);
  EXPECT_EQ(sizeof(GimbalPostureStatePacket{}.gimbal_raised), 1u);
}

TEST(GimbalPostureStatePacket, DefaultIsRaisedNotLowered)
{
  // 默认值必须是「高」。默认成「低」的话，回传帧丢了或者结构体没被填满时，导航会以为
  // 云台已经收好了 —— 车直接带着立起的云台进洞。错的方向必须是安全的那一侧。
  const GimbalPostureStatePacket packet{};
  EXPECT_EQ(packet.gimbal_raised, 1u);
}

TEST(GimbalPostureStatePacket, HeadIsDistinctFromBothOtherFrames)
{
  // 上行现在有两种帧（裁判 'H','L' 和姿态 'H','P'），长度不同。驱动靠第二个字节定帧型、
  // 帧型定长度。三种帧的第二字节两两不同是这套解析能工作的前提。
  const GimbalPostureStatePacket state{};
  const DecisionPacket decision{};
  const GimbalPosturePacket request{};

  EXPECT_EQ(state.head[0], static_cast<uint8_t>('H'));
  EXPECT_EQ(state.head[1], static_cast<uint8_t>('P'));
  EXPECT_NE(state.head[1], decision.head[1]);
  EXPECT_NE(state.head[1], request.head[1]);
  // 上行两种帧长度不同 —— 所以必须靠帧型字节区分，不能靠长度。
  EXPECT_NE(sizeof(GimbalPostureStatePacket), sizeof(DecisionPacket));
}

TEST(IsGimbalLowered, ZeroOnTheWireMeansLowered)
{
  // 极性是电控那侧定的：线上 0 = 低（已收下）、1 = 高（立着）。全工程只有 isGimbalLowered
  // 做这个翻转，所以这条测试就是那个约定的唯一守卫。
  //
  // 翻反了两个方向都不报错，而且都不好受：把 1 读成「已收下」是车带着立起的云台进洞；
  // 把 0 读成「还没收下」是脉冲一直发不停、云台被反复切换。
  GimbalPostureStatePacket lowered{};
  lowered.gimbal_raised = 0;
  EXPECT_TRUE(isGimbalLowered(lowered)) << "线上 0 = 低，应该读成 lowered=true";

  GimbalPostureStatePacket raised{};
  raised.gimbal_raised = 1;
  EXPECT_FALSE(isGimbalLowered(raised)) << "线上 1 = 高，应该读成 lowered=false";

  // 默认构造出来的帧必须读成「没收下」。读成「已收下」的话，回传还没到就以为云台收好了。
  EXPECT_FALSE(isGimbalLowered(GimbalPostureStatePacket{}));
}

TEST(GimbalPostureStatePacket, CrcCoversTheStateByte)
{
  // 姿态字节必须在 CRC 覆盖范围内。不在的话线上一位翻转会让导航以为云台收好了，
  // 而校验照样通过。
  GimbalPostureStatePacket packet{};
  packet.gimbal_raised = 1;
  crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
  EXPECT_TRUE(
    crc16::Verify_CRC16_Check_Sum(
      reinterpret_cast<const uint8_t *>(&packet), sizeof(packet)));

  uint8_t corrupted[sizeof(GimbalPostureStatePacket)];
  std::memcpy(corrupted, &packet, sizeof(packet));
  corrupted[offsetof(GimbalPostureStatePacket, gimbal_raised)] = 0;
  EXPECT_FALSE(crc16::Verify_CRC16_Check_Sum(corrupted, sizeof(corrupted)));
}

TEST(GimbalPosturePacket, CrcCoversTheWholeFrameAndVerifies)
{
  for (const uint8_t toggle : {uint8_t{0}, uint8_t{1}}) {
    GimbalPosturePacket packet{};
    packet.gimbal_toggle = toggle;
    crc16::Append_CRC16_Check_Sum(
      reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

    EXPECT_TRUE(
      crc16::Verify_CRC16_Check_Sum(
        reinterpret_cast<const uint8_t *>(&packet), sizeof(packet)))
      << "toggle=" << static_cast<int>(toggle);
  }
}

TEST(GimbalPosturePacket, CrcRejectsAFlippedFlag)
{
  // 标志位必须在 CRC 覆盖范围内。不在的话线上一位翻转会让云台在错误的时候切换一次，
  // 而校验照样通过 —— 脉冲语义下这种误触发直接把姿态弄反。
  GimbalPosturePacket packet{};
  packet.gimbal_toggle = 1;
  crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

  uint8_t corrupted[sizeof(GimbalPosturePacket)];
  std::memcpy(corrupted, &packet, sizeof(packet));
  corrupted[offsetof(GimbalPosturePacket, gimbal_toggle)] = 0;

  EXPECT_FALSE(crc16::Verify_CRC16_Check_Sum(corrupted, sizeof(corrupted)));
}

TEST(GimbalPosturePacket, DistinctFlagValuesProduceDistinctFrames)
{
  // 「不动」和「切换」在线上必须是不同的字节序列。听起来废话，但如果标志位被写在了 CRC
  // 之后、或者根本没被赋值，这条会失败 —— 而实车表现只是「云台从来不动」。
  GimbalPosturePacket idle{};
  idle.gimbal_toggle = 0;
  crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&idle), sizeof(idle));

  GimbalPosturePacket pulse{};
  pulse.gimbal_toggle = 1;
  crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&pulse), sizeof(pulse));

  EXPECT_NE(std::memcmp(&idle, &pulse, sizeof(idle)), 0);
}

}  // namespace
}  // namespace serial_driver

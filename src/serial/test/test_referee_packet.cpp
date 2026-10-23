// 上行裁判帧（'H','R'）的线上格式与解析测试。
//
// 这一帧是「电控 ↔ NUC」之间的自定义帧，两侧的字节布局必须逐字节对齐。对不上的表现是
// 「一帧都收不到，而日志里干干净净」——因为 CRC 不过时解析器只是丢字节重同步，不报错。
// 所以这里锁的不是行为而是布局：帧头、字段偏移、总长度、CRC 覆盖范围。
//
// 另外单独锁两条**语义**：game_type_progress 的拆位方向（拆反了会把「3v3」读成
// 「比赛中」，而且不报错），以及「上一拍血量」的差分方向（上电第一帧不能被读成
// 「被打了一大管」）。两条都抽成了自由函数，所以能脱离节点测。

#include <cstdint>
#include <cstring>
#include <vector>

#include <gtest/gtest.h>

#include <decision_interfaces/msg/game_status.hpp>
#include <decision_interfaces/msg/robot_status.hpp>

#include "crc.hpp"
#include "packet.hpp"
#include "referee_conversion.hpp"

namespace serial_driver
{
namespace
{

// 把结构体按线上格式打成字节数组并补上 CRC。
std::vector<uint8_t> wireFrame(const RefereePacketV2 & packet)
{
  std::vector<uint8_t> bytes(sizeof(packet));
  std::memcpy(bytes.data(), &packet, sizeof(packet));
  crc16::Append_CRC16_Check_Sum(bytes.data(), static_cast<uint32_t>(bytes.size()));
  return bytes;
}

TEST(RefereePacketV2, LayoutIsTenBytesWithNoPadding)
{
  // head(2) + payload_len(1) + payload(5) + crc16(2)。多一个填充字节就是两侧错位。
  EXPECT_EQ(sizeof(RefereePacketV2), 10u);
  EXPECT_EQ(kRefereePayloadLen, 5u);
  EXPECT_EQ(refereeFrameSize(kRefereePayloadLen), sizeof(RefereePacketV2));

  EXPECT_EQ(offsetof(RefereePacketV2, head), 0u);
  EXPECT_EQ(offsetof(RefereePacketV2, payload_len), 2u);
  EXPECT_EQ(offsetof(RefereePacketV2, game_type_progress), 3u);
  EXPECT_EQ(offsetof(RefereePacketV2, stage_remain_time), 4u);
  EXPECT_EQ(offsetof(RefereePacketV2, sentry_hp), 6u);
  EXPECT_EQ(offsetof(RefereePacketV2, crc16), 8u);

  // 宽度写死，不能跟着 ABI 的 bool 变。
  EXPECT_EQ(sizeof(RefereePacketV2{}.stage_remain_time), 2u);
  EXPECT_EQ(sizeof(RefereePacketV2{}.sentry_hp), 2u);
}

TEST(RefereePacketV2, DefaultsAreSafe)
{
  const RefereePacketV2 packet{};
  EXPECT_EQ(packet.head[0], static_cast<uint8_t>('H'));
  EXPECT_EQ(packet.head[1], static_cast<uint8_t>('R'));
  EXPECT_EQ(packet.payload_len, kRefereePayloadLen);
  EXPECT_EQ(packet.sentry_hp, 0u);
}

TEST(RefereePacketV2, HeadIsDistinctFromThePostureFrame)
{
  // 上行现在两种帧（'R' 裁判 / 'P' 姿态），长度各异。靠第二个字节定帧型，
  // 两者第二字节不同是这套解析能工作的前提。
  const RefereePacketV2 referee{};
  const GimbalPostureStatePacket posture{};
  EXPECT_NE(referee.head[1], posture.head[1]);
  EXPECT_EQ(referee.head[0], posture.head[0]);
}

// ==================== scanFrame ====================

TEST(ScanFrame, PostureIsFixedLength)
{
  const std::vector<uint8_t> posture = {'H', 'P'};
  const auto scan = scanFrame(posture.data(), posture.size());
  EXPECT_EQ(scan.status, FrameScanStatus::Ok);
  EXPECT_EQ(scan.frame_size, sizeof(GimbalPostureStatePacket));
}

TEST(ScanFrame, RefereeLengthComesFromTheFrameItself)
{
  std::vector<uint8_t> buffer = {'H', 'R', kRefereePayloadLen};
  const auto scan = scanFrame(buffer.data(), buffer.size());
  EXPECT_EQ(scan.status, FrameScanStatus::Ok);
  EXPECT_EQ(scan.frame_size, sizeof(RefereePacketV2));

  // 尾部追加字段（len 变大）时总长跟着变，不用改帧型。
  buffer[2] = kRefereePayloadLen + 8;
  const auto grown = scanFrame(buffer.data(), buffer.size());
  EXPECT_EQ(grown.status, FrameScanStatus::Ok);
  EXPECT_EQ(grown.frame_size, sizeof(RefereePacketV2) + 8u);
}

TEST(ScanFrame, WaitsForTheLengthByte)
{
  // 只收到 'H','R' 时还不能定长 —— 必须等，不能猜也不能丢。
  const std::vector<uint8_t> buffer = {'H', 'R'};
  EXPECT_EQ(scanFrame(buffer.data(), buffer.size()).status, FrameScanStatus::NeedMore);
}

TEST(ScanFrame, RejectsAbsurdPayloadLength)
{
  // 一个被干扰的 len 不能让我们按荒谬长度一直等下去，否则会被一段噪声钉死。
  for (const uint8_t bad : {uint8_t{0}, uint8_t{kRefereePayloadLen - 1}, uint8_t{200}}) {
    std::vector<uint8_t> buffer = {'H', 'R', bad};
    const auto scan = scanFrame(buffer.data(), buffer.size());
    EXPECT_EQ(scan.status, FrameScanStatus::NotAFrame)
      << "payload_len=" << static_cast<int>(bad);
  }
}

TEST(ScanFrame, V1RefereeFrameIsNoLongerAccepted)
{
  // v1 的定长 'H','L' 裁判帧已删除，字段并进了 'R'。一个 'H','L' 现在只能算噪声 ——
  // 这是刻意的：两侧必须一起烧，避免「上游还在发 v1、下游按新长度等」的静默失联。
  const std::vector<uint8_t> v1 = {'H', 'L'};
  EXPECT_EQ(scanFrame(v1.data(), v1.size()).status, FrameScanStatus::NotAFrame);
}

TEST(ScanFrame, RejectsNonFrameAndUnknownType)
{
  const std::vector<uint8_t> not_head = {'X', 'L'};
  EXPECT_EQ(
    scanFrame(not_head.data(), not_head.size()).status, FrameScanStatus::NotAFrame);

  // 'H' 后面跟了个不认识的帧型字节：这个 'H' 是数据里碰巧出现的，不是帧头。
  const std::vector<uint8_t> unknown = {'H', 'Z'};
  EXPECT_EQ(
    scanFrame(unknown.data(), unknown.size()).status, FrameScanStatus::NotAFrame);
}

TEST(ScanFrame, ShortBufferNeedsMore)
{
  const std::vector<uint8_t> one = {'H'};
  EXPECT_EQ(scanFrame(one.data(), one.size()).status, FrameScanStatus::NeedMore);
  EXPECT_EQ(scanFrame(nullptr, 0).status, FrameScanStatus::NeedMore);
}

// ==================== CRC ====================

TEST(RefereePacketV2, CrcCoversTheWholeFrame)
{
  RefereePacketV2 packet{};
  packet.game_type_progress = static_cast<uint8_t>((4U << 4U) | kGameTypeRmUl3v3);
  packet.sentry_hp = 350;
  const auto bytes = wireFrame(packet);
  EXPECT_TRUE(crc16::Verify_CRC16_Check_Sum(bytes.data(), static_cast<uint32_t>(bytes.size())));

  // 血量位翻转必须让校验失败：它是低血回家的唯一依据。
  auto corrupted = bytes;
  corrupted[offsetof(RefereePacketV2, sentry_hp)] = 0;
  EXPECT_FALSE(
    crc16::Verify_CRC16_Check_Sum(corrupted.data(), static_cast<uint32_t>(corrupted.size())));
}

// ==================== 拆位 ====================

TEST(GameTypeProgress, LowNibbleIsTypeHighNibbleIsProgress)
{
  // 协议原生布局：低 4 bit 比赛类型、高 4 bit 比赛阶段。线上一个字节原样透传，
  // 拆位在这侧做 —— 拆反了会把「4（3v3）」读成「比赛中」，而且不会有任何报错。
  const uint8_t raw = static_cast<uint8_t>((4U << 4U) | kGameTypeRmUl3v3);
  EXPECT_EQ(gameTypeOf(raw), kGameTypeRmUl3v3);
  EXPECT_EQ(gameProgressOf(raw), 4u);

  // 全 0：未开始比赛的 0 型（不适用）与 0 阶段。
  EXPECT_EQ(gameTypeOf(0), 0u);
  EXPECT_EQ(gameProgressOf(0), 0u);

  // 上边界不能串位。
  EXPECT_EQ(gameTypeOf(0x0F), 0x0Fu);
  EXPECT_EQ(gameProgressOf(0xF0), 0x0Fu);
}

TEST(HpDecreased, AttackedIsStrictlyLowerThanPrevious)
{
  EXPECT_TRUE(hpDecreased(400, 399));
  EXPECT_TRUE(hpDecreased(400, 0));
  // 没变不算挨打；回血更不算。
  EXPECT_FALSE(hpDecreased(400, 400));
  EXPECT_FALSE(hpDecreased(400, 401));
  // 上电第一帧：上一拍是 0，这一拍是真血量，绝不能读成「被打了一大管」。
  EXPECT_FALSE(hpDecreased(0, 400));
}

// ==================== 线上帧 -> ROS 消息 ====================

TEST(ToGameStatus, SplitsTypeAndProgress)
{
  RefereePacketV2 packet{};
  packet.game_type_progress = static_cast<uint8_t>((4U << 4U) | kGameTypeRmUl3v3);
  packet.stage_remain_time = 217;

  const auto msg = toGameStatus(packet);
  EXPECT_EQ(msg.game_type, kGameTypeRmUl3v3);
  EXPECT_EQ(msg.game_progress, 4u);
  EXPECT_EQ(msg.stage_remain_time, 217u);
}

TEST(ToRobotStatus, CarriesHpAndAttackFlag)
{
  RefereePacketV2 packet{};
  packet.sentry_hp = 143;

  const auto hit = toRobotStatus(packet, true);
  EXPECT_EQ(hit.robot_id, kSentryRobotId);
  EXPECT_EQ(hit.current_hp, 143u);
  EXPECT_TRUE(hit.is_attacked);

  const auto calm = toRobotStatus(packet, false);
  EXPECT_FALSE(calm.is_attacked);
}

TEST(ToRobotStatus, FillsOnlyTheFieldsTheFrameCarries)
{
  // RobotStatus 只该有上行帧真正带的三样东西（robot_id 是 3v3 编制常量）。
  // 上一版还发过 shooter_heat / team_color，它们的数据已不在帧里，
  // 消息和这里的填充都一起删掉了 —— 留着不填的字段，下游看到默认值会以为它有意义。
  RefereePacketV2 packet{};
  packet.sentry_hp = 400;
  const auto msg = toRobotStatus(packet, false);
  EXPECT_EQ(msg.robot_id, kSentryRobotId);
  EXPECT_EQ(msg.current_hp, 400u);
  EXPECT_FALSE(msg.is_attacked);
}

}  // namespace
}  // namespace serial_driver

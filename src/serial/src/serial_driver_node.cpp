#include "serial_driver_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace serial_driver
{
namespace
{

constexpr auto kTransmitInterval = std::chrono::milliseconds(1);
constexpr auto kReceiveInterval = std::chrono::milliseconds(5);
constexpr auto kStatusInterval = std::chrono::seconds(1);
constexpr auto kReconnectDelay = std::chrono::seconds(1);
// 云台姿态帧的发送周期。20 Hz：导航侧判定跑 10 Hz，发得比它快一档，最坏情况下的额外
// 延迟不超过一个判定周期。平时这一帧的内容是 0（不动），5 字节 × 20 Hz 对带宽无影响。
constexpr auto kGimbalPostureInterval = std::chrono::milliseconds(50);
constexpr uint8_t kPacketHead0 = static_cast<uint8_t>('H');
constexpr uint8_t kPacketHead1 = static_cast<uint8_t>('L');
// 云台姿态回传的帧型字节。上行现在有两种帧，长度不同（裁判 9 字节、姿态 5 字节），
// 靠第二个字节定帧型、帧型定长度 —— 不能反过来靠长度猜。
constexpr uint8_t kPostureStateHead1 = static_cast<uint8_t>('P');
constexpr std::size_t kDecisionPacketSize = sizeof(DecisionPacket);
constexpr std::size_t kPostureStatePacketSize = sizeof(GimbalPostureStatePacket);

std::string packetToHex(const uint8_t * data, std::size_t size)
{
  std::ostringstream stream;
  stream << std::hex << std::uppercase << std::setfill('0');

  for (std::size_t i = 0; i < size; ++i) {
    if (i != 0) {
      stream << ' ';
    }
    stream << std::setw(2) << static_cast<unsigned int>(data[i]);
  }

  return stream.str();
}

std::string packetToHex(const ChassisCommandPacket & packet)
{
  return packetToHex(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
}

std::string packetToHex(const GimbalPosturePacket & packet)
{
  return packetToHex(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
}

}  // namespace

SerialDriverNode::SerialDriverNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("serial_driver_node", options)
{
  getParam();
  port_ = std::make_shared<Port>(config_);

  RCLCPP_INFO(
    get_logger(),
    "Starting serial_driver_node with device=%s baud_rate=%d flow_control=%s",
    config_->devname.c_str(), config_->baudrate, config_->flowcontrol ? "true" : "false");

  for (int attempt = 1; attempt <= 4 && !port_->isPortOpen(); ++attempt) {
    RCLCPP_INFO(get_logger(), "Opening serial port attempt %d/4", attempt);
    port_->openPort();
  }

  if (!port_->isPortOpen()) {
    RCLCPP_ERROR(
      get_logger(), "Failed to open serial port after 4 attempts. device=%s",
      config_->devname.c_str());
  } else {
    RCLCPP_INFO(
      get_logger(), "Serial port ready. device=%s fd=%d",
      config_->devname.c_str(), port_->fd);
  }

  chassis_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel_chassis", rclcpp::SensorDataQoS(),
    std::bind(&SerialDriverNode::ChassisCmdCallback, this, std::placeholders::_1));

  // 收云台请求。transient_local + reliable 要跟 rm_tunnel_posture 的发布端对上 ——
  // QoS 不匹配的表现是「话题在、一帧都收不到」，而且不报错，云台会直接撞在顶板上。
  gimbal_posture_sub_ = this->create_subscription<decision_interfaces::msg::GimbalPosture>(
    "gimbal_posture", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&SerialDriverNode::GimbalPostureCallback, this, std::placeholders::_1));

  robot_status_pub_ = this->create_publisher<decision_interfaces::msg::RobotStatus>(
    "robot_status", rclcpp::SensorDataQoS());
  game_status_pub_ = this->create_publisher<decision_interfaces::msg::GameStatus>(
    "game_status", rclcpp::SensorDataQoS());
  // 云台实测姿态。跟请求同样是状态型话题，用 transient_local 让晚起的订阅者立刻拿到
  // 当前姿态，而不是等下一帧回传。
  gimbal_posture_state_pub_ = this->create_publisher<decision_interfaces::msg::GimbalPostureState>(
    "gimbal_posture_state", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  transmit_timer_ = this->create_wall_timer(
    kTransmitInterval, std::bind(&SerialDriverNode::transmit, this));
  receive_timer_ = this->create_wall_timer(
    kReceiveInterval, std::bind(&SerialDriverNode::receive, this));
  status_timer_ = this->create_wall_timer(
    kStatusInterval, std::bind(&SerialDriverNode::logStatus, this));
  gimbal_posture_timer_ = this->create_wall_timer(
    kGimbalPostureInterval, [this]() {
      syncGimbalPosture();
    });

  RCLCPP_INFO(
    get_logger(),
    "Subscribed to cmd_vel_chassis; RX=HL+current_hp+game_progress+stage_remain_time+crc16; chassis scale x=%.1f y=%.1f w=%.1f",
    chassis_vel_x_scale_, chassis_vel_y_scale_, chassis_vel_w_scale_);
  RCLCPP_INFO(
    get_logger(),
    "Gimbal posture: sub=gimbal_posture pub=gimbal_posture_state; "
    "TX=HG+lower+crc16 (%zu bytes, sent only while state differs, checked every %ld ms); "
    "RX=HP+raised+crc16 (%zu bytes, 0=lowered)",
    sizeof(GimbalPosturePacket),
    static_cast<long>(kGimbalPostureInterval.count()),
    sizeof(GimbalPostureStatePacket));
}

int SerialDriverNode::transmit()
{
  if (!port_->isPortOpen()) {
    return last_write_size_.load(std::memory_order_relaxed);
  }

  constexpr std::size_t packet_size = sizeof(ChassisCommandPacket);
  // 复用成员缓冲区，避免栈分配
  uint8_t * buffer = transmit_buffer_cache_;

  while (true) {
    {
      std::lock_guard<std::mutex> lock(transmit_mutex);
      if (transmit_buffer.size() < packet_size) {
        break;
      }

      std::copy(
        transmit_buffer.begin(), transmit_buffer.begin() + packet_size, buffer);
      transmit_buffer.erase(
        transmit_buffer.begin(), transmit_buffer.begin() + packet_size);
    }

    const auto packet = bufferToStruct<ChassisCommandPacket>(buffer);
    int bytes_written = 0;
    {
      // 锁只包住这一次 write，不要包整个 while —— 队列里攒了几十帧时会把云台帧饿死。
      std::lock_guard<std::mutex> lock(write_mutex_);
      bytes_written = port_->transmit(buffer, static_cast<int>(packet_size));
    }
    last_write_size_.store(bytes_written, std::memory_order_relaxed);

    if (bytes_written != static_cast<int>(packet_size)) {
      failed_send_count_.fetch_add(1, std::memory_order_relaxed);
      RCLCPP_ERROR(
        get_logger(),
        "Serial write failed. expected=%zu actual=%d device=%s vel_x=%.3f vel_y=%.3f vel_w=%.3f "
        "crc16=0x%04X raw=[%s]",
        packet_size, bytes_written, config_->devname.c_str(), packet.vel_x, packet.vel_y,
        packet.vel_w, static_cast<unsigned int>(packet.crc16), packetToHex(packet).c_str());
      reopenPort("Write failure");
      return bytes_written;
    }

    sent_packet_count_.fetch_add(1, std::memory_order_relaxed);
    // 日志移至 DEBUG 级别，降低 1kHz 循环 CPU 占用
    RCLCPP_DEBUG(
      get_logger(),
      "Serial write ok. bytes=%d device=%s vel_x=%.3f vel_y=%.3f vel_w=%.3f crc16=0x%04X",
      bytes_written, config_->devname.c_str(), packet.vel_x, packet.vel_y, packet.vel_w,
      static_cast<unsigned int>(packet.crc16));
  }

  return last_write_size_.load(std::memory_order_relaxed);
}

void SerialDriverNode::receive()
{
  // write_mutex_ 同时保护 fd 的 close/open 切换（reopenPort 用它）：读也纳入
  // 同一把锁，避免与重连的 close/open 形成数据竞争。read 是非阻塞的，锁只
  // 包住单次 read 不包整个 while。
  std::lock_guard<std::mutex> port_lock(write_mutex_);
  if (!port_->isPortOpen()) {
    return;
  }

  uint8_t chunk[64];
  while (true) {
    const int bytes_read = port_->receive(chunk);
    if (bytes_read <= 0) {
      break;
    }

    {
      std::lock_guard<std::mutex> lock(receive_mutex_);
      receive_buffer_.insert(receive_buffer_.end(), chunk, chunk + bytes_read);
      // Prevent unbounded growth if the stream is corrupt.
      if (receive_buffer_.size() > 4096) {
        receive_buffer_.erase(
          receive_buffer_.begin(),
          receive_buffer_.begin() + static_cast<std::ptrdiff_t>(receive_buffer_.size() - 2048));
      }
    }
  }

  processReceiveBuffer();
}

void SerialDriverNode::processReceiveBuffer()
{
  std::lock_guard<std::mutex> lock(receive_mutex_);

  // 上行有两种帧：'H''L' 裁判数据、'H''P' 云台姿态回传。长度不同，所以必须先按第二个
  // 字节定帧型再按帧型取长度。原来的写法假定只有一种帧长，会把姿态帧当垃圾一个字节
  // 一个字节吃掉 —— 表现是回传永远收不到，而且日志里干干净净。
  while (receive_buffer_.size() >= 2) {
    if (receive_buffer_[0] != kPacketHead0) {
      receive_buffer_.erase(receive_buffer_.begin());
      continue;
    }

    std::size_t frame_size = 0;
    if (receive_buffer_[1] == kPacketHead1) {
      frame_size = kDecisionPacketSize;
    } else if (receive_buffer_[1] == kPostureStateHead1) {
      frame_size = kPostureStatePacketSize;
    } else {
      // 'H' 后面跟了个不认识的帧型字节：这个 'H' 是数据里碰巧出现的，不是帧头。
      receive_buffer_.erase(receive_buffer_.begin());
      continue;
    }

    if (receive_buffer_.size() < frame_size) {
      // 帧还没收全。留在缓冲里等下一次 receive()，不要丢 —— 丢了就得等对面重发。
      break;
    }

    if (!handlePacket(receive_buffer_.data(), frame_size)) {
      // 帧型对但 CRC 不过：丢一个字节重新同步。
      receive_buffer_.erase(receive_buffer_.begin());
      continue;
    }

    receive_buffer_.erase(
      receive_buffer_.begin(),
      receive_buffer_.begin() + static_cast<std::ptrdiff_t>(frame_size));
  }
}

bool SerialDriverNode::handlePacket(const uint8_t * data, std::size_t size)
{
  if (data[0] != kPacketHead0) {
    return false;
  }

  // 全帧 CRC，两种上行帧同一个算法（也跟下行的底盘帧一致）。
  if (!crc16::Verify_CRC16_Check_Sum(data, static_cast<uint32_t>(size))) {
    return false;
  }

  if (data[1] == kPacketHead1 && size == kDecisionPacketSize) {
    publishDecisionPacket(bufferToStruct<DecisionPacket>(data));
    return true;
  }

  if (data[1] == kPostureStateHead1 && size == kPostureStatePacketSize) {
    publishGimbalPostureState(bufferToStruct<GimbalPostureStatePacket>(data));
    return true;
  }

  return false;
}

void SerialDriverNode::publishDecisionPacket(const DecisionPacket & packet)
{
  decision_interfaces::msg::RobotStatus robot_msg;
  robot_msg.robot_id = 7;
  robot_msg.current_hp = packet.current_hp;
  robot_msg.shooter_heat = 0;
  robot_msg.team_color = false;

  const auto previous_hp = last_current_hp_.exchange(packet.current_hp, std::memory_order_relaxed);
  robot_msg.is_attacked = packet.current_hp < previous_hp;

  decision_interfaces::msg::GameStatus game_msg;
  game_msg.game_progress = packet.game_progress;
  game_msg.stage_remain_time = packet.stage_remain_time;

  last_game_progress_.store(packet.game_progress, std::memory_order_relaxed);
  last_stage_remain_time_.store(packet.stage_remain_time, std::memory_order_relaxed);
  received_decision_count_.fetch_add(1, std::memory_order_relaxed);

  if (robot_status_pub_) {
    robot_status_pub_->publish(robot_msg);
  }
  if (game_status_pub_) {
    game_status_pub_->publish(game_msg);
  }

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "Published decision packet. current_hp=%u game_progress=%u stage_remain_time=%u is_attacked=%s",
    static_cast<unsigned int>(packet.current_hp), static_cast<unsigned int>(packet.game_progress),
    static_cast<unsigned int>(packet.stage_remain_time), robot_msg.is_attacked ? "true" : "false");
}

void SerialDriverNode::ChassisCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (!port_->isPortOpen()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Received cmd_vel_chassis but serial port is closed. device=%s", config_->devname.c_str());
  }

  try {
    uint8_t buffer[sizeof(ChassisCommandPacket)];
    ChassisCommandPacket packet{};
    packet.vel_x = static_cast<float>(msg->linear.x * chassis_vel_x_scale_);
    packet.vel_y = static_cast<float>(msg->linear.y * chassis_vel_y_scale_);
    packet.vel_w = static_cast<float>(msg->angular.z * chassis_vel_w_scale_);
    crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
    structToBuffer(packet, buffer);

    std::size_t queue_bytes = 0;
    {
      std::lock_guard<std::mutex> lock(transmit_mutex);
      // 端口关闭时 transmit() 直接丢弃、队列只进不出会无界增长。给队列设上限
      // （10 包 ≈ 240 B），超限丢最旧——对速度指令流来说旧指令本来就不该在
      // 重连后一股脑灌给底盘，保留最新才是安全语义。
      constexpr std::size_t kMaxQueuedChassisBytes = sizeof(ChassisCommandPacket) * 10;
      while (transmit_buffer.size() + sizeof(packet) > kMaxQueuedChassisBytes &&
             !transmit_buffer.empty())
      {
        if (transmit_buffer.size() < sizeof(ChassisCommandPacket)) {
          transmit_buffer.clear();
          break;
        }
        transmit_buffer.erase(
          transmit_buffer.begin(), transmit_buffer.begin() + sizeof(ChassisCommandPacket));
      }
      transmit_buffer.insert(transmit_buffer.end(), buffer, buffer + sizeof(packet));
      queue_bytes = transmit_buffer.size();
    }

    received_cmd_count_.fetch_add(1, std::memory_order_relaxed);
    queued_packet_count_.fetch_add(1, std::memory_order_relaxed);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Queued chassis command. ros(x=%.3f y=%.3f w=%.3f) -> mcu(x=%.3f y=%.3f w=%.3f) "
      "scale(x=%.1f y=%.1f w=%.1f) queue_bytes=%llu crc16=0x%04X raw=[%s]",
      msg->linear.x, msg->linear.y, msg->angular.z,
      packet.vel_x, packet.vel_y, packet.vel_w,
      chassis_vel_x_scale_, chassis_vel_y_scale_, chassis_vel_w_scale_,
      static_cast<unsigned long long>(queue_bytes),
      static_cast<unsigned int>(packet.crc16), packetToHex(packet).c_str());
  } catch (const std::exception & e) {
    failed_send_count_.fetch_add(1, std::memory_order_relaxed);
    RCLCPP_ERROR(get_logger(), "Error while queuing chassis command: %s", e.what());
    reopenPort("Queue failure");
  }
}

void SerialDriverNode::publishGimbalPostureState(const GimbalPostureStatePacket & packet)
{
  // 极性翻转只在 isGimbalLowered 里做（packet.hpp），这里之后一律是 lowered 语义。
  const bool lowered = isGimbalLowered(packet);

  decision_interfaces::msg::GimbalPostureState msg;
  msg.lowered = lowered;

  const bool previous_lowered = gimbal_state_lowered_.exchange(lowered, std::memory_order_relaxed);
  const bool first = gimbal_state_rx_count_.fetch_add(1, std::memory_order_relaxed) == 0;

  if (gimbal_posture_state_pub_) {
    gimbal_posture_state_pub_->publish(msg);
  }

  if (first || previous_lowered != lowered) {
    // 回传是持续的，每帧都打会把日志刷没，所以只在翻转时打。请求值一起打出来：
    // 这两个值分开看都正常，只有对比才能发现「导航一直在请求收，云台压根没动」。
    RCLCPP_INFO(
      get_logger(), "Gimbal posture state: lowered=%s (requested lower=%s)",
      lowered ? "true" : "false",
      has_gimbal_request_.load(std::memory_order_relaxed) ?
      (gimbal_lower_.load(std::memory_order_relaxed) ? "true" : "false") : "none");
  }
}

void SerialDriverNode::GimbalPostureCallback(
  const decision_interfaces::msg::GimbalPosture::SharedPtr msg)
{
  // 只存值，发送交给定时器。回调里直接写串口会让这条链路的时序取决于导航侧的发布节奏，
  // 而且写阻塞时会拖住导航节点所在的执行器。
  gimbal_lower_.store(msg->lower, std::memory_order_relaxed);
  has_gimbal_request_.store(true, std::memory_order_relaxed);

  // 只在翻转时打日志：这一帧 20 Hz 重发，每帧都打会把日志刷没。
  const bool previous = gimbal_lower_logged_.exchange(msg->lower, std::memory_order_relaxed);
  if (previous != msg->lower) {
    RCLCPP_INFO(
      get_logger(), "Gimbal posture request changed: lower=%s", msg->lower ? "true" : "false");
  }
}

void SerialDriverNode::syncGimbalPosture()
{
  // 平时发 0（什么都不做），请求翻转时发一帧 1 —— 只发这一帧。
  //
  // 判据是「请求变了」而不是「实测跟请求不一致」。后者看似更稳，实际会连发：脉冲发出去
  // 之后串口往返 + 云台动作要几十毫秒，这期间回传还是旧值，20 Hz 下一致性判据会再触发
  // 一次，云台被切回去。用请求翻转做判据就没有这个窗口 —— 一次请求对应一帧指令。
  //
  // 也正因为如此，这里不需要等回传、不需要知道当前姿态：请求从「抬」变成「收」，发一次
  // 切换就是对的。回传只用来让 MPC 知道云台到位没有（见 rm_mpc_controller），不参与发送。
  const bool want_lower = gimbal_lower_.load(std::memory_order_relaxed);
  const bool toggle_pending =
    has_gimbal_request_.load(std::memory_order_relaxed) &&
    gimbal_commanded_lower_.load(std::memory_order_relaxed) != want_lower;

  if (!transmitGimbalPosture(toggle_pending ? 1 : 0)) {
    // 写失败（端口没开、write 返回短），指令根本没上线。不记账，下个周期还会发。
    // 这不是「不信电控」—— 检查的是我们自己 write 的返回值。
    return;
  }

  if (toggle_pending) {
    gimbal_commanded_lower_.store(want_lower, std::memory_order_relaxed);
    RCLCPP_INFO(get_logger(), "发送云台切换指令：目标=%s", want_lower ? "低" : "高");
  }
}

bool SerialDriverNode::transmitGimbalPosture(uint8_t toggle)
{
  if (!port_->isPortOpen()) {
    return false;
  }

  GimbalPosturePacket packet{};
  packet.gimbal_toggle = toggle;
  crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

  uint8_t buffer[sizeof(GimbalPosturePacket)];
  structToBuffer(packet, buffer);

  int bytes_written = 0;
  {
    std::lock_guard<std::mutex> lock(write_mutex_);
    bytes_written = port_->transmit(buffer, static_cast<int>(sizeof(buffer)));
  }

  if (bytes_written != static_cast<int>(sizeof(buffer))) {
    // 不重连、不清缓冲：底盘速度帧的失败路径已经在管端口重开了，这里再插一手会在
    // 串口抖动时把重连打成两倍频。下一个周期自然会重发，丢一帧的代价是 50 ms 延迟。
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Gimbal posture write failed. expected=%zu actual=%d device=%s raw=[%s]",
      sizeof(buffer), bytes_written, config_->devname.c_str(), packetToHex(packet).c_str());
    return false;
  }

  if (toggle != 0) {
    // 只统计脉冲。把 0 帧也算进去的话这个数就只是个周期计数，看不出发过几次切换。
    gimbal_sent_count_.fetch_add(1, std::memory_order_relaxed);
  }
  RCLCPP_DEBUG(
    get_logger(), "Gimbal posture frame sent. toggle=%u raw=[%s]",
    static_cast<unsigned int>(packet.gimbal_toggle), packetToHex(packet).c_str());
  return true;
}

void SerialDriverNode::logStatus()
{
  std::size_t queue_bytes = 0;
  {
    std::lock_guard<std::mutex> lock(transmit_mutex);
    queue_bytes = transmit_buffer.size();
  }

  RCLCPP_INFO(
    get_logger(),
    "Serial status: port_open=%s device=%s fd=%d queue_bytes=%llu decision_rx=%llu received=%llu queued=%llu "
    "sent=%llu failed=%llu last_write=%d last_hp=%u last_game=%u last_time=%u "
    "gimbal_want=%s gimbal_is=%s gimbal_toggles=%llu gimbal_rx=%llu",
    port_->isPortOpen() ? "true" : "false", config_->devname.c_str(), port_->fd,
    static_cast<unsigned long long>(queue_bytes),
    static_cast<unsigned long long>(received_decision_count_.load(std::memory_order_relaxed)),
    static_cast<unsigned long long>(received_cmd_count_.load(std::memory_order_relaxed)),
    static_cast<unsigned long long>(queued_packet_count_.load(std::memory_order_relaxed)),
    static_cast<unsigned long long>(sent_packet_count_.load(std::memory_order_relaxed)),
    static_cast<unsigned long long>(failed_send_count_.load(std::memory_order_relaxed)),
    last_write_size_.load(std::memory_order_relaxed),
    static_cast<unsigned int>(last_current_hp_.load(std::memory_order_relaxed)),
    static_cast<unsigned int>(last_game_progress_.load(std::memory_order_relaxed)),
    static_cast<unsigned int>(last_stage_remain_time_.load(std::memory_order_relaxed)),
    has_gimbal_request_.load(std::memory_order_relaxed) ?
    (gimbal_lower_.load(std::memory_order_relaxed) ? "lowered" : "raised") : "none",
    gimbal_state_rx_count_.load(std::memory_order_relaxed) > 0 ?
    (gimbal_state_lowered_.load(std::memory_order_relaxed) ? "lowered" : "raised") : "unknown",
    static_cast<unsigned long long>(gimbal_sent_count_.load(std::memory_order_relaxed)),
    static_cast<unsigned long long>(gimbal_state_rx_count_.load(std::memory_order_relaxed)));
}

bool SerialDriverNode::reopenPort(const char * reason)
{
  RCLCPP_WARN(get_logger(), "%s. Reopening serial port...", reason);
  {
    // 关端口瞬间锁住 fd 切换（receive/transmit 都经 write_mutex_ 访问 fd）。
    std::lock_guard<std::mutex> lock(write_mutex_);
    port_->closePort();
    {
      std::lock_guard<std::mutex> receive_lock(receive_mutex_);
      receive_buffer_.clear();
    }
  }

  // 重连延时放在锁外：旧实现锁内 sleep 1s，单线程执行器下整节点（含串口接收
  // 解析与云台姿态处理）都会冻结。锁外睡眠期间 receive/transmit 因端口关闭
  // 自然失败返回，不影响其他路径。
  rclcpp::sleep_for(kReconnectDelay);

  {
    std::lock_guard<std::mutex> lock(write_mutex_);
    port_->openPort();
  }

  if (!port_->isPortOpen()) {
    RCLCPP_ERROR(
      get_logger(), "Reopen failed. device=%s", config_->devname.c_str());
    return false;
  }

  RCLCPP_INFO(
    get_logger(), "Reopened serial port. device=%s fd=%d",
    config_->devname.c_str(), port_->fd);
  return true;
}

void SerialDriverNode::getParam()
{
  int baud_rate{};
  bool flowcontrol = false;
  auto parity = Parity::NONE;
  auto stop_bits = StopBit::ONE;

  try {
    device_name_ = declare_parameter<std::string>("device_name", "/dev/ttyACM0");
    // 默认与 config/serial_driver.yaml 一致（标准 POSIX 速率；串口层不再支持非标速率）。
    baud_rate = declare_parameter<int>("baud_rate", 115200);
    flowcontrol = declare_parameter<bool>("flow_control", false);
    chassis_vel_x_scale_ = declare_parameter<double>("chassis_vel_x_scale", -1.0);
    chassis_vel_y_scale_ = declare_parameter<double>("chassis_vel_y_scale", -1.0);
    chassis_vel_w_scale_ = declare_parameter<double>("chassis_vel_w_scale", 1.0);
  } catch (const rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "Invalid serial parameter type: %s", ex.what());
    throw;
  }

  try {
    const auto parity_string = declare_parameter<std::string>("parity", "none");

    if (parity_string == "none") {
      parity = Parity::NONE;
    } else if (parity_string == "odd") {
      parity = Parity::ODD;
    } else if (parity_string == "even") {
      parity = Parity::EVEN;
    } else {
      throw std::invalid_argument(
              "The parity parameter must be one of: none, odd, or even.");
    }
  } catch (const rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid: %s", ex.what());
    throw;
  }

  try {
    const auto stop_bits_string = declare_parameter<std::string>("stop_bits", "1.0");

    if (stop_bits_string == "1" || stop_bits_string == "1.0") {
      stop_bits = StopBit::ONE;
    } else if (stop_bits_string == "1.5") {
      stop_bits = StopBit::ONE_POINT_FIVE;
    } else if (stop_bits_string == "2" || stop_bits_string == "2.0") {
      stop_bits = StopBit::TWO;
    } else {
      throw std::invalid_argument(
              "The stop_bits parameter must be one of: 1, 1.5, or 2.");
    }
  } catch (const rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid: %s", ex.what());
    throw;
  }

  config_ = std::make_shared<SerialConfig>(
    baud_rate, 8, flowcontrol, stop_bits, parity, device_name_);
}

}  // namespace serial_driver

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(serial_driver::SerialDriverNode)

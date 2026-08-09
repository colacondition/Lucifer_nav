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
constexpr uint8_t kPacketHead0 = static_cast<uint8_t>('H');
constexpr uint8_t kPacketHead1 = static_cast<uint8_t>('L');
constexpr std::size_t kDecisionPacketSize = sizeof(DecisionPacket);

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

  robot_status_pub_ = this->create_publisher<decision_interfaces::msg::RobotStatus>(
    "robot_status", rclcpp::SensorDataQoS());
  game_status_pub_ = this->create_publisher<decision_interfaces::msg::GameStatus>(
    "game_status", rclcpp::SensorDataQoS());

  transmit_timer_ = this->create_wall_timer(
    kTransmitInterval, std::bind(&SerialDriverNode::transmit, this));
  receive_timer_ = this->create_wall_timer(
    kReceiveInterval, std::bind(&SerialDriverNode::receive, this));
  status_timer_ = this->create_wall_timer(
    kStatusInterval, std::bind(&SerialDriverNode::logStatus, this));

  RCLCPP_INFO(
    get_logger(),
    "Subscribed to cmd_vel_chassis; RX=HL+current_hp+game_progress+stage_remain_time+crc16; chassis scale x=%.1f y=%.1f w=%.1f",
    chassis_vel_x_scale_, chassis_vel_y_scale_, chassis_vel_w_scale_);
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
    const int bytes_written = port_->transmit(buffer, static_cast<int>(packet_size));
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

  // RX frame: 'H''L' + decision payload + uint16 crc16.
  while (receive_buffer_.size() >= kDecisionPacketSize) {
    // Sync on head bytes.
    if (receive_buffer_[0] != kPacketHead0 || receive_buffer_[1] != kPacketHead1) {
      receive_buffer_.erase(receive_buffer_.begin());
      continue;
    }

    if (!handlePacket(receive_buffer_.data(), kDecisionPacketSize)) {
      // Head matched but CRC/content invalid: drop one byte and resync.
      receive_buffer_.erase(receive_buffer_.begin());
      continue;
    }

    receive_buffer_.erase(
      receive_buffer_.begin(),
      receive_buffer_.begin() + static_cast<std::ptrdiff_t>(kDecisionPacketSize));
  }
}

bool SerialDriverNode::handlePacket(const uint8_t * data, std::size_t size)
{
  if (size != kDecisionPacketSize) {
    return false;
  }

  if (data[0] != kPacketHead0 || data[1] != kPacketHead1) {
    return false;
  }

  // Full frame CRC covers the whole decision packet (same algorithm as chassis TX).
  if (!crc16::Verify_CRC16_Check_Sum(data, static_cast<uint32_t>(size))) {
    return false;
  }

  const auto packet = bufferToStruct<DecisionPacket>(data);
  publishDecisionPacket(packet);

  return true;
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
    "sent=%llu failed=%llu last_write=%d last_hp=%u last_game=%u last_time=%u",
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
    static_cast<unsigned int>(last_stage_remain_time_.load(std::memory_order_relaxed)));
}

bool SerialDriverNode::reopenPort(const char * reason)
{
  RCLCPP_WARN(get_logger(), "%s. Reopening serial port...", reason);
  port_->closePort();
  {
    std::lock_guard<std::mutex> lock(receive_mutex_);
    receive_buffer_.clear();
  }
  rclcpp::sleep_for(kReconnectDelay);
  port_->openPort();

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
    baud_rate = declare_parameter<int>("baud_rate", 961200);
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

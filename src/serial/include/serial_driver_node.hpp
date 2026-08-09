#ifndef SERIAL_DRIVER_MY_NODE_HPP_
#define SERIAL_DRIVER_MY_NODE_HPP_

#include <atomic>
#include <cstdint>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include <decision_interfaces/msg/game_status.hpp>
#include <decision_interfaces/msg/robot_status.hpp>

#include "crc.hpp"
#include "packet.hpp"
#include "serial_port.hpp"

namespace serial_driver
{

class SerialDriverNode : public rclcpp::Node
{
public:
  explicit SerialDriverNode(const rclcpp::NodeOptions & options);

private:
  void getParam();
  int transmit();
  void receive();
  void processReceiveBuffer();
  bool handlePacket(const uint8_t * data, std::size_t size);
  void publishDecisionPacket(const DecisionPacket & packet);
  void ChassisCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void logStatus();
  bool reopenPort(const char * reason);

  template<typename T>
  T bufferToStruct(const uint8_t * buffer)
  {
    T value{};
    std::memcpy(&value, buffer, sizeof(T));
    return value;
  }

  template<typename T>
  void structToBuffer(const T & value, uint8_t * buffer)
  {
    std::memcpy(buffer, &value, sizeof(T));
  }

  std::shared_ptr<SerialConfig> config_;
  std::shared_ptr<Port> port_;
  std::string device_name_;
  std::deque<uint8_t> transmit_buffer;
  std::mutex transmit_mutex;
  std::vector<uint8_t> receive_buffer_;
  std::mutex receive_mutex_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr chassis_cmd_sub_;
  rclcpp::Publisher<decision_interfaces::msg::RobotStatus>::SharedPtr robot_status_pub_;
  rclcpp::Publisher<decision_interfaces::msg::GameStatus>::SharedPtr game_status_pub_;
  rclcpp::TimerBase::SharedPtr transmit_timer_;
  rclcpp::TimerBase::SharedPtr receive_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  // ROS cmd_vel_chassis -> MCU packet axis scales (use -1 when MCU axis is opposite).
  double chassis_vel_x_scale_{-1.0};
  double chassis_vel_y_scale_{-1.0};
  double chassis_vel_w_scale_{1.0};

  std::atomic<uint64_t> received_cmd_count_{0};
  std::atomic<uint64_t> queued_packet_count_{0};
  std::atomic<uint64_t> sent_packet_count_{0};
  std::atomic<uint64_t> failed_send_count_{0};
  std::atomic<uint64_t> received_decision_count_{0};
  std::atomic<int> last_write_size_{0};
  std::atomic<uint16_t> last_current_hp_{0};
  std::atomic<uint8_t> last_game_progress_{0};
  std::atomic<uint16_t> last_stage_remain_time_{0};

  // 预分配缓冲区，避免 1kHz 热循环中重复分配
  uint8_t transmit_buffer_cache_[sizeof(ChassisCommandPacket)];
};

}  // namespace serial_driver

#endif  // SERIAL_DRIVER_MY_NODE_HPP_

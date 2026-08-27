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
#include <decision_interfaces/msg/gimbal_posture.hpp>
#include <decision_interfaces/msg/gimbal_posture_state.hpp>
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
  void publishGimbalPostureState(const GimbalPostureStatePacket & packet);
  void ChassisCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void GimbalPostureCallback(const decision_interfaces::msg::GimbalPosture::SharedPtr msg);
  // 按周期发姿态帧：请求翻转时发一帧切换指令（且只发一帧），其余时候发 0。
  void syncGimbalPosture();
  // 把一帧云台指令写到串口。返回 false 表示没写成功，下个周期自然会重发。
  bool transmitGimbalPosture(uint8_t toggle);
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
  // 串口写锁。底盘速度帧和云台姿态帧各跟一个定时器写同一个 fd；
  // launch 起的是独立进程（默认单线程 spin），回调本身串行。锁仍保留：
  // 节点也注册为 component，若被装进多线程容器，没这把锁两次 write 会拼帧。
  std::mutex write_mutex_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr chassis_cmd_sub_;
  rclcpp::Subscription<decision_interfaces::msg::GimbalPosture>::SharedPtr gimbal_posture_sub_;
  rclcpp::Publisher<decision_interfaces::msg::RobotStatus>::SharedPtr robot_status_pub_;
  rclcpp::Publisher<decision_interfaces::msg::GameStatus>::SharedPtr game_status_pub_;
  rclcpp::Publisher<decision_interfaces::msg::GimbalPostureState>::SharedPtr
    gimbal_posture_state_pub_;
  rclcpp::TimerBase::SharedPtr transmit_timer_;
  rclcpp::TimerBase::SharedPtr receive_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr gimbal_posture_timer_;
  // ROS cmd_vel_chassis -> MCU packet axis scales (use -1 when MCU axis is opposite).
  double chassis_vel_x_scale_{-1.0};
  double chassis_vel_y_scale_{-1.0};
  double chassis_vel_w_scale_{1.0};

  std::atomic<uint64_t> received_cmd_count_{0};
  std::atomic<uint64_t> queued_packet_count_{0};
  std::atomic<uint64_t> sent_packet_count_{0};
  std::atomic<uint64_t> failed_send_count_{0};
  // 连续写失败计数：区分「瞬时 EAGAIN 丢帧重试」与「持续故障阈值 reopen」，
  // 见底盘下发路径的分级处理。仅单写线程访问。
  std::atomic<int> write_fail_streak_{0};
  bool allow_fallback_{true};
  int write_fail_reopen_threshold_{20};
  std::atomic<uint64_t> received_decision_count_{0};
  std::atomic<int> last_write_size_{0};

  // 导航侧想要的姿态。存当前值、按周期跟回传比对，不进 transmit_buffer 排队 —— 排队的话
  // 串口一堵就攒下一串过期的指令，之后一次性冲给电控，云台会按几秒前的地图位置动作。
  std::atomic<bool> gimbal_lower_{false};
  // 只用来判断日志要不要打，不参与发送决策。
  std::atomic<bool> gimbal_lower_logged_{false};
  std::atomic<bool> has_gimbal_request_{false};
  std::atomic<uint64_t> gimbal_sent_count_{0};
  // 电控回传的实测姿态（已转成 lowered 语义，线上是 0=低）。不参与发送决策 —— 我们照周期
  // 发当前想要的值就行。留着是为了发到 /gimbal_posture_state 和打进状态日志，用来看云台
  // 真的到没到位。
  std::atomic<bool> gimbal_state_lowered_{false};
  std::atomic<uint64_t> gimbal_state_rx_count_{0};
  // 已经发过切换指令的目标姿态。跟 gimbal_lower_ 不一致就说明有一次翻转还没发出去。
  // 初值 false 要跟电控的默认姿态（高，即没收下来）对齐 —— 对不齐会在启动后凭空发一帧。
  std::atomic<bool> gimbal_commanded_lower_{false};
  std::atomic<uint16_t> last_current_hp_{0};
  std::atomic<uint8_t> last_game_progress_{0};
  std::atomic<uint16_t> last_stage_remain_time_{0};

  // 预分配缓冲区，避免 1kHz 热循环中重复分配
  uint8_t transmit_buffer_cache_[sizeof(ChassisCommandPacket)];
};

}  // namespace serial_driver

#endif  // SERIAL_DRIVER_MY_NODE_HPP_

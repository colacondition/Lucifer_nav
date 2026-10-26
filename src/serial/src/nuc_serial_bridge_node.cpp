// 导航侧串口桥：用 nuc_serial::SerialLink 替代原来的私有协议 serial_driver。
//
// 对外接口与 serial_driver_node 保持一致（下游零改动）：
//   订阅 /cmd_vel_chassis (Twist)        -> 下行 vel_x/vel_y/vel_w
//   订阅 /gimbal_posture (GimbalPosture) -> 下行 gimbal_toggle 脉冲
//   发布 /robot_status /game_status /gimbal_posture_state <- 上行
//
// 自瞄与导航同包同串口时，自瞄侧只需在同一个 SerialLink 上再调 setAim()；
// 本节点不碰 aim 字段（flags bit1 因此为 0）。
#include <chrono>
#include <memory>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include <decision_interfaces/msg/game_status.hpp>
#include <decision_interfaces/msg/gimbal_posture.hpp>
#include <decision_interfaces/msg/gimbal_posture_state.hpp>
#include <decision_interfaces/msg/robot_status.hpp>
#include <nuc_serial/serial_link.hpp>

namespace
{
using namespace std::chrono_literals;

class NucSerialBridge : public rclcpp::Node
{
public:
  NucSerialBridge()
  : rclcpp::Node("nuc_serial_bridge")
  {
    const std::string device = declare_parameter<std::string>("device_name", "/dev/ttyACM0");
    const int baud = declare_parameter<int>("baud_rate", 115200);
    const double scale_x = declare_parameter<double>("chassis_vel_x_scale", -1.0);
    const double scale_y = declare_parameter<double>("chassis_vel_y_scale", -1.0);
    const double scale_w = declare_parameter<double>("chassis_vel_w_scale", 1.0);
    scale_[0] = scale_x;
    scale_[1] = scale_y;
    scale_[2] = scale_w;

    nuc_serial::SerialLink::Options opt;
    opt.device = device;
    opt.baud = baud;
    link_ = std::make_unique<nuc_serial::SerialLink>(opt);
    if (!link_->start()) {
      RCLCPP_ERROR(get_logger(), "串口打开失败：%s", device.c_str());
    }

    robot_pub_ = create_publisher<decision_interfaces::msg::RobotStatus>(
      "/robot_status", rclcpp::QoS(rclcpp::KeepLast(10)));
    game_pub_ = create_publisher<decision_interfaces::msg::GameStatus>(
      "/game_status", rclcpp::QoS(rclcpp::KeepLast(10)));
    // 状态型话题：transient_local，晚起的订阅者也能立刻拿到当前姿态
    posture_pub_ = create_publisher<decision_interfaces::msg::GimbalPostureState>(
      "/gimbal_posture_state", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_chassis", rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        cmd_[0] = static_cast<float>(msg->linear.x * scale_[0]);
        cmd_[1] = static_cast<float>(msg->linear.y * scale_[1]);
        cmd_[2] = static_cast<float>(msg->angular.z * scale_[2]);
        cmd_stamp_ = std::chrono::steady_clock::now();
        have_cmd_ = true;
      });
    posture_sub_ = create_subscription<decision_interfaces::msg::GimbalPosture>(
      "/gimbal_posture", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      [this](decision_interfaces::msg::GimbalPosture::ConstSharedPtr msg) {
        if (msg->lower) {
          toggle_pending_ = true;   // 脉冲：下一拍发一次 1
        }
      });

    timer_ = create_wall_timer(20ms, [this]() { tick(); });
    RCLCPP_INFO(get_logger(), "nuc_serial_bridge 就绪：%s @ %d", device.c_str(), baud);
  }

private:
  static std::uint32_t monoMs()
  {
    return static_cast<std::uint32_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
  }

  void tick()
  {
    // 下行：速度超过 200 ms 没更新就清零（防止上游挂掉后一直发旧速度）
    const auto age_ms = have_cmd_
      ? std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - cmd_stamp_).count()
      : 1000000;
    const bool fresh = age_ms <= 200;
    link_->setNav(
      fresh ? cmd_[0] : 0.0F, fresh ? cmd_[1] : 0.0F, fresh ? cmd_[2] : 0.0F,
      toggle_pending_);
    link_->setTick(monoMs());   // 下行 nuc_tick_ms，供 MCU 回显测延迟
    toggle_pending_ = false;

    // 上行：发布给决策/MPC
    nuc_serial::UpPayload up;
    if (!link_->latest(up)) {
      return;
    }
    decision_interfaces::msg::RobotStatus robot;
    robot.robot_id = 7;               // 3v3 编制的哨兵
    robot.current_hp = up.sentry_hp;
    robot.is_attacked = false;        // 协议里暂未带
    robot_pub_->publish(robot);

    decision_interfaces::msg::GameStatus game;
    game.game_type = 4;               // RMUL 3V3
    game.game_progress = up.game_progress;
    game.stage_remain_time = up.stage_remain_time;
    game_pub_->publish(game);

    decision_interfaces::msg::GimbalPostureState posture;
    posture.lowered = (up.gimbal_raised == 0);
    posture_pub_->publish(posture);
  }

  std::unique_ptr<nuc_serial::SerialLink> link_;
  rclcpp::Publisher<decision_interfaces::msg::RobotStatus>::SharedPtr robot_pub_;
  rclcpp::Publisher<decision_interfaces::msg::GameStatus>::SharedPtr game_pub_;
  rclcpp::Publisher<decision_interfaces::msg::GimbalPostureState>::SharedPtr posture_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<decision_interfaces::msg::GimbalPosture>::SharedPtr posture_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  float cmd_[3] = {0.0F, 0.0F, 0.0F};
  double scale_[3] = {-1.0, -1.0, 1.0};
  bool have_cmd_ = false;
  bool toggle_pending_ = false;
  std::chrono::steady_clock::time_point cmd_stamp_{};
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NucSerialBridge>());
  rclcpp::shutdown();
  return 0;
}

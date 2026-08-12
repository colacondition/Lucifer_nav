// 仿真专用云台模拟器。
//
// 实车链路上 `/gimbal_posture_state` 由 serial_driver 转发电控回传的实测姿态，
// 电控**持续上报**当前云台姿态（不是只在姿态变化时发一次），MPC 会等 lowered 翻
// 过来才放车走（见 mpc_controller_node.cpp 的「等云台收下来再进洞」）。仿真里没
// 有电控、也没有 serial_driver，所以这条回传在仿真里永远是空 —— 一旦语义地图有
// 隧道，MPC 会因为 gimbal_lowered_ 停在 false 而永久停车。
//
// 这个节点顶替电控：收到 rm_tunnel_posture 的收/放请求后，等动作延迟（默认 0.5 s，
// 模拟电控执行时间）再翻转内部姿态，并以固定频率持续回传当前姿态 —— 和电控上报
// 行为一致。仿真就能复现「请求 → 停顿 → 放下/升起 → 走」的完整时序，且可视化端
// 能实时看到云台当前姿态，不用改任何导航侧代码。

#include <algorithm>
#include <chrono>
#include <string>

#include <decision_interfaces/msg/gimbal_posture.hpp>
#include <decision_interfaces/msg/gimbal_posture_state.hpp>
#include <rclcpp/rclcpp.hpp>

namespace simulated_gimbal
{

class SimulatedGimbal : public rclcpp::Node
{
public:
  explicit SimulatedGimbal(const rclcpp::NodeOptions & options)
  : Node("simulated_gimbal", options)
  {
    posture_topic_ = declare_parameter<std::string>("posture_topic", "/gimbal_posture");
    state_topic_ = declare_parameter<std::string>("state_topic", "/gimbal_posture_state");
    action_delay_ = declare_parameter<double>("action_delay", 0.5);
    report_rate_ = declare_parameter<double>("report_rate", 20.0);

    // 与实车 serial_driver 一样用 transient_local + reliable：
    // 状态型话题，晚起的订阅者（MPC / 云台可视化）也该立刻拿到当前值。
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    posture_sub_ = create_subscription<decision_interfaces::msg::GimbalPosture>(
      posture_topic_, qos,
      [this](decision_interfaces::msg::GimbalPosture::SharedPtr msg) {
        onPosture(msg->lower);
      });
    state_pub_ = create_publisher<decision_interfaces::msg::GimbalPostureState>(
      state_topic_, qos);

    // 持续上报当前姿态，频率 report_rate_，和电控的周期性上报一致。
    const auto report_period =
      std::chrono::duration<double>(1.0 / std::max(1.0, report_rate_));
    report_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(report_period),
      [this]() {
        publishState(current_lowered_);
      });

    // 开机先回一次「高」，让晚起的 MPC 拿到确定初值而不是干等。
    publishState(current_lowered_);

    RCLCPP_INFO(
      get_logger(),
      "simulated gimbal ready: %s -> %s (action %.2f s, report %.0f Hz)",
      posture_topic_.c_str(), state_topic_.c_str(), action_delay_, report_rate_);
  }

private:
  void onPosture(bool lower)
  {
    // 请求只在目标变化时才需要一次动作。rm_tunnel_posture 每帧都发，
    // 连续发相同目标时不该反复触发动作。
    if (lower == target_) {
      return;
    }
    target_ = lower;

    // 动作是一次性的：收或放各需 action_delay_ 秒，到点翻转当前姿态。
    // 翻转前 current_lowered_ 保持旧值 —— 持续上报会让 MPC/可视化看到
    // 「还在旧的姿态」，正好模拟电控执行需要时间。
    if (action_timer_) {
      action_timer_->cancel();
    }
    const auto period = std::chrono::duration<double>(std::max(0.01, action_delay_));
    action_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {
        current_lowered_ = target_;
        RCLCPP_INFO(get_logger(), "gimbal action done: lowered=%s",
                    current_lowered_ ? "true" : "false");
      });
    RCLCPP_INFO(get_logger(), "gimbal action: lower=%s (%.2f s)", lower ? "true" : "false",
                action_delay_);
  }

  void publishState(bool lowered)
  {
    decision_interfaces::msg::GimbalPostureState msg;
    msg.lowered = lowered;
    state_pub_->publish(msg);
  }

  std::string posture_topic_;
  std::string state_topic_;
  double action_delay_{0.5};
  double report_rate_{20.0};
  bool target_{false};
  bool current_lowered_{false};

  rclcpp::Subscription<decision_interfaces::msg::GimbalPosture>::SharedPtr posture_sub_;
  rclcpp::Publisher<decision_interfaces::msg::GimbalPostureState>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr report_timer_;
  rclcpp::TimerBase::SharedPtr action_timer_;
};

}  // namespace simulated_gimbal

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(simulated_gimbal::SimulatedGimbal)

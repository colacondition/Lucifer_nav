#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

namespace navigation2
{

class RmVelocitySmoother : public rclcpp::Node
{
public:
  explicit RmVelocitySmoother(const rclcpp::NodeOptions & options)
  : Node("rm_velocity_smoother", options)
  {
    input_topic_ = declare_parameter<std::string>("input_cmd_vel_topic", "/cmd_vel_nav");
    output_topic_ = declare_parameter<std::string>("output_cmd_vel_topic", "/cmd_vel");
    smoothing_frequency_ = declare_parameter<double>("smoothing_frequency", 20.0);
    velocity_timeout_ = declare_parameter<double>("velocity_timeout", 0.5);
    max_velocity_ = toArray(declare_parameter<std::vector<double>>(
      "max_velocity", std::vector<double>{2.0, 2.0, 3.0}), {2.0, 2.0, 3.0});
    min_velocity_ = toArray(declare_parameter<std::vector<double>>(
      "min_velocity", std::vector<double>{-2.0, -2.0, -3.0}), {-2.0, -2.0, -3.0});
    max_accel_ = toArray(declare_parameter<std::vector<double>>(
      "max_accel", std::vector<double>{4.0, 4.0, 6.0}), {4.0, 4.0, 6.0});
    max_decel_ = toArray(declare_parameter<std::vector<double>>(
      "max_decel", std::vector<double>{-4.0, -4.0, -6.0}), {-4.0, -4.0, -6.0});
    deadband_velocity_ = toArray(declare_parameter<std::vector<double>>(
      "deadband_velocity", std::vector<double>{0.0, 0.0, 0.0}), {0.0, 0.0, 0.0});

    // std::clamp 要求 lo <= hi；min > max 的配置在 debug 构建触发断言，
    // release 下也会把速度夹到错误值。非法时回退默认边界而不是带病运行。
    for (std::size_t i = 0; i < 3; ++i) {
      if (!(min_velocity_[i] <= max_velocity_[i])) {
        RCLCPP_ERROR(
          get_logger(),
          "Invalid velocity bounds at axis %zu: min=%.3f > max=%.3f; "
          "falling back to default limits", i, min_velocity_[i], max_velocity_[i]);
        min_velocity_ = {-2.0, -2.0, -3.0};
        max_velocity_ = {2.0, 2.0, 3.0};
        break;
      }
    }

    // 订阅输入速度，输出平滑速度。
    // mt 容器下订阅与定时器共享 last_cmd_time_/latest cmd，串行化。
    cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = cb_group_;

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      input_topic_, rclcpp::QoS(1),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        target_cmd_ = *msg;
        last_cmd_time_ = now();
      },
      sub_options);
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_topic_, rclcpp::QoS(1));

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, smoothing_frequency_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {
        update();
      },
      cb_group_);

    RCLCPP_INFO(
      get_logger(), "rm_velocity_smoother ready: %s -> %s", input_topic_.c_str(),
      output_topic_.c_str());
  }

private:
  static std::array<double, 3> toArray(
    const std::vector<double> & values, const std::array<double, 3> & fallback)
  {
    std::array<double, 3> output = fallback;
    for (std::size_t i = 0; i < std::min<std::size_t>(3, values.size()); ++i) {
      output[i] = values[i];
    }
    return output;
  }

  static std::array<double, 3> twistToArray(const geometry_msgs::msg::Twist & twist)
  {
    return {twist.linear.x, twist.linear.y, twist.angular.z};
  }

  static geometry_msgs::msg::Twist arrayToTwist(const std::array<double, 3> & values)
  {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = values[0];
    twist.linear.y = values[1];
    twist.angular.z = values[2];
    return twist;
  }

  void update()
  {
    // 超时就把目标归零。
    const auto stamp = now();
    double dt = 1.0 / std::max(1.0, smoothing_frequency_);
    if (last_update_time_.nanoseconds() != 0) {
      dt = std::max(0.001, (stamp - last_update_time_).seconds());
    }
    last_update_time_ = stamp;

    auto target = twistToArray(target_cmd_);
    if (last_cmd_time_.nanoseconds() == 0 ||
      (stamp - last_cmd_time_).seconds() > velocity_timeout_)
    {
      target = std::array<double, 3>{0.0, 0.0, 0.0};
    }

    auto current = twistToArray(current_cmd_);
    for (std::size_t i = 0; i < 3; ++i) {
      target[i] = std::clamp(target[i], min_velocity_[i], max_velocity_[i]);
      const double delta = target[i] - current[i];
      const double limit = delta >= 0.0 ? std::abs(max_accel_[i]) * dt :
        std::abs(max_decel_[i]) * dt;
      current[i] += std::clamp(delta, -limit, limit);
      current[i] = std::clamp(current[i], min_velocity_[i], max_velocity_[i]);
      if (std::abs(current[i]) <= deadband_velocity_[i]) {
        current[i] = 0.0;
      }
    }

    current_cmd_ = arrayToTwist(current);
    cmd_pub_->publish(current_cmd_);
  }

  std::string input_topic_;
  std::string output_topic_;
  double smoothing_frequency_{20.0};
  double velocity_timeout_{0.5};
  std::array<double, 3> max_velocity_{2.0, 2.0, 3.0};
  std::array<double, 3> min_velocity_{-2.0, -2.0, -3.0};
  std::array<double, 3> max_accel_{4.0, 4.0, 6.0};
  std::array<double, 3> max_decel_{-4.0, -4.0, -6.0};
  std::array<double, 3> deadband_velocity_{0.0, 0.0, 0.0};

  geometry_msgs::msg::Twist target_cmd_;
  geometry_msgs::msg::Twist current_cmd_;
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace navigation2

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(navigation2::RmVelocitySmoother)

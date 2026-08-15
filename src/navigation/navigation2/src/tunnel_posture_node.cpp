// 隧道云台请求节点。
//
// 唯一的职责：车快要进洞 / 正在洞里的时候，告诉电控把云台收下来。收多低、怎么收、
// 到位没到位全归电控 —— 所以这个节点不需要任何车体参数。
//
// 为什么单独一个节点而不是塞进代价地图：代价地图管栅格，这里管一个发给执行侧的请求。
// 两件事的失效方式完全不同 —— 代价地图算错是「路规划得难看」，这个漏发是「云台撞在
// 顶板上」。放在一起会让后者的 20 行逻辑被埋进 900 行栅格代码里。

#include "tunnel_posture.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <decision_interfaces/msg/gimbal_posture.hpp>
#include <decision_interfaces/msg/semantic_map.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "semantic_map_consumer.hpp"

namespace navigation2
{

class RmTunnelPosture : public rclcpp::Node
{
public:
  explicit RmTunnelPosture(const rclcpp::NodeOptions & options)
  : Node("rm_tunnel_posture", options),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
  {
    global_frame_ = declare_parameter<std::string>("global_frame", "map");
    robot_base_frame_ = declare_parameter<std::string>("robot_base_frame", "base_link_fake");
    semantic_map_topic_ =
      declare_parameter<std::string>("semantic_map_topic", "/map_server/semantic_map");
    path_topic_ = declare_parameter<std::string>("path_topic", "/plan");
    posture_topic_ = declare_parameter<std::string>("posture_topic", "/gimbal_posture");
    update_frequency_ = declare_parameter<double>("update_frequency", 10.0);
    hysteresis_ = declare_parameter<double>("hysteresis", 0.3);
    decider_ = GimbalLowerDecider(hysteresis_);

    // mt 容器下订阅与定时器共享 receiver_/path_points_，串行化（同 planner 纪律）。
    cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = cb_group_;

    semantic_map_sub_ = create_subscription<decision_interfaces::msg::SemanticMap>(
      semantic_map_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      [this](decision_interfaces::msg::SemanticMap::ConstSharedPtr msg) {
        try {
          if (!receiver_.update(*msg)) {
            return;
          }
        } catch (const std::exception & ex) {
          // 拒收整帧，保留上一张好图 —— 半张图会让隧道格错位，在错误的位置收云台。
          RCLCPP_ERROR(get_logger(), "Rejected semantic map: %s", ex.what());
          return;
        }
        RCLCPP_INFO(
          get_logger(), "Tunnel posture got semantic map: %zu tunnels",
          receiver_.map().tunnels().size());
      },
      sub_options);

    // reliable + transient_local：这是个状态型请求而不是数据流，漏一帧的代价是云台
    // 该收的时候没收。晚起的电控节点也应当立刻拿到当前请求。
    posture_pub_ = create_publisher<decision_interfaces::msg::GimbalPosture>(
      posture_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    // 规划路径：判定「车是不是真的要穿洞」。只关心 xy，抽出后查本体格。
    // 收不到路径时 points 为空，will_cross 恒 false —— 只有车已在本体内（inside
    // 兜底）才会收，贴着洞口路过不会再误收。
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      [this](nav_msgs::msg::Path::ConstSharedPtr msg) {
        std::vector<Eigen::Vector2d> points;
        points.reserve(msg->poses.size());
        for (const auto & pose : msg->poses) {
          points.emplace_back(pose.pose.position.x, pose.pose.position.y);
        }
        path_points_ = std::move(points);
      },
      sub_options);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, update_frequency_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {
        tick();
      },
      cb_group_);

    // 开机先发一次 false，让电控有个确定的初值而不是等第一次进洞。
    publish(false);

    RCLCPP_INFO(
      get_logger(), "rm_tunnel_posture ready: semantic_map=%s output=%s",
      semantic_map_topic_.c_str(), posture_topic_.c_str());
  }

private:
  void tick()
  {
    if (receiver_.map().tunnels().empty()) {
      // 图里没有隧道（或还没收到图）：不需要 tf，也不该因为拿不到 tf 报警。
      publish(false);
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(global_frame_, robot_base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      // 拿不到位姿时保持上一次的请求，不要翻成 false。定位短暂丢失时把云台抬起来
      // 是最坏的选择 —— 车可能正在洞里。
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Cannot get robot transform %s -> %s: %s",
        global_frame_.c_str(), robot_base_frame_.c_str(), ex.what());
      publish(decider_.lower());
      return;
    }

    // 只有规划路径穿过隧道本体才收（inside 兜底在 decider 里判）。
    const bool will_cross = pathCrossesTunnel(receiver_.map(), path_points_);
    publish(
      decider_.update(
        receiver_.map(), transform.transform.translation.x, transform.transform.translation.y,
        will_cross));
  }

  void publish(bool lower)
  {
    decision_interfaces::msg::GimbalPosture msg;
    msg.lower = lower;
    posture_pub_->publish(msg);

    if (!has_published_ || last_lower_ != lower) {
      RCLCPP_INFO(get_logger(), "Gimbal posture request: lower=%s", lower ? "true" : "false");
      last_lower_ = lower;
      has_published_ = true;
    }
  }

  std::string global_frame_;
  std::string robot_base_frame_;
  std::string semantic_map_topic_;
  std::string path_topic_;
  std::string posture_topic_;
  double update_frequency_{10.0};
  double hysteresis_{0.3};

  SemanticMapReceiver receiver_;
  GimbalLowerDecider decider_;
  std::vector<Eigen::Vector2d> path_points_;
  bool last_lower_{false};
  bool has_published_{false};

  rclcpp::Subscription<decision_interfaces::msg::SemanticMap>::SharedPtr semantic_map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  rclcpp::Publisher<decision_interfaces::msg::GimbalPosture>::SharedPtr posture_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace navigation2

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(navigation2::RmTunnelPosture)

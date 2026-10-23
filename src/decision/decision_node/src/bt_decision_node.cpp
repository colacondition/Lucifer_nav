// 决策节点（**唯一生产节点**）：用 BehaviorTree.CPP v3 执行 config/trees/rmul.xml。
//
// 旧 FSM 版的 `bt_action_replacement_node`（src/decision_node.cpp + DecisionStateMachine）
// 已删除；本节点沿用同一个节点名 `bt_action_replacement`（yaml 根键/launch 参数段不变），
// 并把旧 node 的全部接线搬到了 `MissionRuntime`（mission_runtime.hpp）里：
//   - 上行 QoS（robot/game 默认 reliable；localization 与 fast_location 一样
//     reliable + transient_local；自瞄载荷 reliable depth 10）
//   - FollowWaypoints action 下发、执行器结果超时守卫、/goal_pose 立即抢占
//   - TF map->base_link（车体）与 map->lidar（云台）：云台系目标 → map、center_zone 夹取
//   - 中心区网格/patrol.csv 点池与游标、组间停顿
//   - /decision/state（当前激活任务）与 center_patrol/target_debug
//
// 本节点自己只做：建黑板、每拍同步输入、tick 树、发布状态。优先级/行为全在 XML。

#include <chrono>
#include <exception>
#include <memory>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <decision_interfaces/msg/game_status.hpp>
#include <decision_interfaces/msg/robot_status.hpp>

#include "decision/bt_nodes.hpp"
#include "decision/decision_config.hpp"
#include "decision/mission_runtime.hpp"

namespace
{

constexpr char kDefaultTreeRelativePath[] = "/trees/rmul.xml";

rclcpp::QoS latchedQos()
{
  rclcpp::QoS qos(1);
  qos.reliable().transient_local();
  return qos;
}

}  // namespace

class BtDecisionNode : public rclcpp::Node
{
public:
  BtDecisionNode()
  : Node("bt_action_replacement")
  {
    decision::declareDecisionParameters(*this);
    config_ = decision::loadDecisionConfig(*this);
    runtime_ = std::make_unique<decision::MissionRuntime>(config_, *this);

    tree_file_ = declare_parameter<std::string>("tree_file", "");

    // 黑板：backend 指针是唯一"活"的条目，其余派生输入每拍同步。
    blackboard_ = decision::bt::makeDecisionBlackboard();
    decision::bt::setBackend(blackboard_, runtime_.get());

    decision::bt::registerDecisionNodes(factory_);
    tree_ = loadTree();

    decision_state_pub_ = create_publisher<std_msgs::msg::String>(
      config_.topics.decision_state, latchedQos());

    // ---------- 上行输入（QoS 与上游严格对齐） ----------
    robot_status_sub_ = create_subscription<decision_interfaces::msg::RobotStatus>(
      config_.topics.robot_status, 10,
      [this](const decision_interfaces::msg::RobotStatus & msg) {
        runtime_->updateRobotStatus(msg, nowSec());
      });
    game_status_sub_ = create_subscription<decision_interfaces::msg::GameStatus>(
      config_.topics.game_status, 10,
      [this](const decision_interfaces::msg::GameStatus & msg) {
        runtime_->updateGameStatus(msg, nowSec());
      });
    if (config_.integrity_gate.enable) {
      // fast_location 的 localization_status 是 reliable + transient_local；订阅端
      // 必须同 QoS，否则一条都收不到（而且不报错）。
      localization_status_sub_ = create_subscription<std_msgs::msg::String>(
        config_.topics.localization_status, latchedQos(),
        [this](const std_msgs::msg::String & msg) {
          runtime_->updateLocalizationStatus(msg.data);
        });
    }
    if (config_.target_chase.enable) {
      // 上游 sp_vision_25 用默认 QoS（RELIABLE / depth 10）；这里不能改成
      // SensorDataQoS(best_effort)，否则静默收不到。
      target_sub_ = create_subscription<std_msgs::msg::String>(
        config_.target_chase.topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
        [this](const std_msgs::msg::String & msg) {
          runtime_->updateTargetPayload(msg.data, nowSec());
        });
    }

    runtime_->initialize();

    const double loop_hz = config_.loop_hz > 0.0 ? config_.loop_hz : 10.0;
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / loop_hz), [this]() { tick(); });

    publishDecisionState(runtime_->activeState(), /*force=*/true);
    RCLCPP_INFO(
      get_logger(),
      "bt_decision_node（BT 唯一生产节点）已启动：loop=%.1fHz tree=%s",
      loop_hz, tree_file_.c_str());
  }

private:
  double nowSec()
  {
    return get_clock()->now().seconds();
  }

  std::unique_ptr<BT::Tree> loadTree()
  {
    std::string tree_file = tree_file_;
    if (tree_file.empty()) {
      tree_file = ament_index_cpp::get_package_share_directory("decision") +
        kDefaultTreeRelativePath;
    }
    tree_file_ = tree_file;
    RCLCPP_INFO(get_logger(), "loading behavior tree: %s", tree_file.c_str());
    try {
      return std::make_unique<BT::Tree>(
        factory_.createTreeFromFile(tree_file, blackboard_));
    } catch (const std::exception & error) {
      RCLCPP_FATAL(
        get_logger(), "failed to load behavior tree '%s': %s", tree_file.c_str(),
        error.what());
      throw;
    }
  }

  void tick()
  {
    runtime_->checkDuplicateNodes();
    runtime_->beginTick();  // 清本拍位姿缓存 + 执行器超时守卫
    decision::bt::syncBlackboard(blackboard_, *runtime_);
    try {
      tree_->tickRoot();
    } catch (const std::exception & error) {
      // 单个叶子异常不该打死整个决策进程：报错跳过本拍，下一拍重试。
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "behavior tree tick failed: %s", error.what());
    }
    runtime_->endTick();  // 丢弃未消费事件 + 节流发布 target_debug
    publishDecisionState(runtime_->activeState(), /*force=*/false);
  }

  void publishDecisionState(const std::string & state, bool force)
  {
    if (!force && state == last_published_state_) {
      return;
    }
    last_published_state_ = state;
    std_msgs::msg::String message;
    message.data = state;
    decision_state_pub_->publish(message);
    RCLCPP_INFO(get_logger(), "决策状态 -> %s", state.c_str());
  }

  decision::DecisionConfig config_;
  std::unique_ptr<decision::MissionRuntime> runtime_;
  BT::BehaviorTreeFactory factory_;
  BT::Blackboard::Ptr blackboard_;
  std::unique_ptr<BT::Tree> tree_;
  std::string tree_file_;
  std::string last_published_state_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr decision_state_pub_;
  rclcpp::Subscription<decision_interfaces::msg::RobotStatus>::SharedPtr robot_status_sub_;
  rclcpp::Subscription<decision_interfaces::msg::GameStatus>::SharedPtr game_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr localization_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr target_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<BtDecisionNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("bt_decision_node"), "fatal: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

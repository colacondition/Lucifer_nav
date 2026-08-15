#include "grid_utils.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace navigation2
{

class RmNav2Compat : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ServerGoalHandle<NavigateToPose>;

  explicit RmNav2Compat(const rclcpp::NodeOptions & options)
  : Node("rm_nav2_compat", options),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
  {
    loadParameters();

    // 只做单向桥接。
    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic_, rclcpp::QoS(10));
    status_sub_ = create_subscription<std_msgs::msg::String>(
      status_topic_, rclcpp::QoS(10),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(status_mutex_);
        latest_status_ = msg->data;
      });

    action_server_ = rclcpp_action::create_server<NavigateToPose>(
      this,
      action_name_,
      [this](
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const NavigateToPose::Goal> goal) {
        return handleGoal(goal);
      },
      [this](const std::shared_ptr<GoalHandleNavigateToPose>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandleNavigateToPose> goal_handle) {
        // 单执行线程 + 目标槽位 + 代次抢占，替代「每个目标 spawn 一个 detached
        // 线程」：旧实现不 join，组件卸载时线程仍可能访问 node 成员（UAF），
        // 且新目标不会真的抢占旧目标（两个线程并行跑、互发 /goal_pose）。
        const uint64_t gen =
          goal_generation_.fetch_add(1, std::memory_order_acq_rel) + 1;
        {
          std::lock_guard<std::mutex> lock(goal_slot_mutex_);
          pending_goal_ = std::make_pair(goal_handle, gen);
        }
        goal_slot_cv_.notify_all();
      });

    RCLCPP_INFO(
      get_logger(), "rm_nav2_compat ready: action=%s -> goal_topic=%s",
      action_name_.c_str(), goal_topic_.c_str());

    exec_thread_ = std::thread([this]() { executorLoop(); });
  }

  ~RmNav2Compat() override
  {
    // 唤醒执行线程并等待其退出：它只在本循环内访问 node 成员，join 保证
    // 组件卸载时不再有在途访问（修复旧 detached 线程的 UAF 面）。
    shutdown_ = true;
    goal_slot_cv_.notify_all();
    if (exec_thread_.joinable()) {
      exec_thread_.join();
    }

    // Humble 的 rclcpp_action 模板化 ServerGoalHandle 析构会对「从未终结」的
    // 目标做自动 cancel 过渡（server_goal_handle.hpp ~ServerGoalHandle：
    // try_canceling() → on_terminal_state_ → ServerBase::publish_result）。
    // 若 rcl action server 已先被销毁/目标已从 rcl 层移除，publish_result 抛
    // "Asked to publish result for goal that does not exist"，且发生在析构中
    // （成员 pending_goal_ 的销毁晚于 action_server_）→ std::terminate →
    // 容器 exit -6（ros2/rclcpp#2757，实测线上复现：Ctrl-C 时容器必死）。
    // 修复：在成员销毁（action_server_ 先行析构）之前，把在途目标和槽位里
    // 未消费的目标都显式终结掉，析构路径不再走自动 cancel。tryAbort 内部
    // 已兜异常（目标已过期时 abort 本身会抛，但此时目标必已是终结态，安全）。
    auto abort_result = std::make_shared<NavigateToPose::Result>();
    std::shared_ptr<GoalHandleNavigateToPose> pending;
    {
      std::lock_guard<std::mutex> lock(goal_slot_mutex_);
      pending = pending_goal_.first;
      pending_goal_.first.reset();
    }
    if (active_handle_) {
      tryAbort(active_handle_, abort_result);
      active_handle_.reset();
    }
    if (pending) {
      tryAbort(pending, abort_result);
    }
  }

private:
  // 目标执行线程：串行消费目标槽位，代次不一致即视为被新目标抢占。
  //
  // 注意：这个线程不在 executor 里，任何未捕获异常都会 std::terminate 干掉
  // 整个容器进程（线上已出现过一次：目标被抢占的瞬间 publish_feedback 抛
  // UnknownGoalHandleError → 容器死亡 → 地图/代价图/点云全部消失）。这里和
  // executeGoal 内部都必须兜住异常。
  void executorLoop()
  {
    for (;;) {
      std::shared_ptr<GoalHandleNavigateToPose> goal_handle;
      uint64_t gen = 0;
      {
        std::unique_lock<std::mutex> lock(goal_slot_mutex_);
        goal_slot_cv_.wait(lock, [this] {
          return shutdown_ || pending_goal_.first != nullptr;
        });
        if (shutdown_) {
          return;
        }
        goal_handle = std::move(pending_goal_.first);
        gen = pending_goal_.second;
        pending_goal_.first.reset();
        pending_goal_.second = 0;
      }
      try {
        executeGoal(goal_handle, gen);
      } catch (const std::exception & ex) {
        RCLCPP_ERROR(
          get_logger(), "executeGoal threw, container kept alive: %s", ex.what());
        try {
          clearActiveGoalIfCurrent(gen);
        } catch (...) {
        }
      } catch (...) {
        RCLCPP_ERROR(get_logger(), "executeGoal threw non-std exception, container kept alive");
        try {
          clearActiveGoalIfCurrent(gen);
        } catch (...) {
        }
      }
    }
  }

  void loadParameters()
  {
    // 目标输入和状态输出的话题名。
    global_frame_ = declare_parameter<std::string>("global_frame", "map");
    robot_base_frame_ = declare_parameter<std::string>("robot_base_frame", "base_link_fake");
    action_name_ = declare_parameter<std::string>("action_name", "/navigate_to_pose");
    goal_topic_ = declare_parameter<std::string>("goal_topic", "/goal_pose");
    status_topic_ = declare_parameter<std::string>("status_topic", "/navigation2/status");
    feedback_frequency_ = declare_parameter<double>("feedback_frequency", 10.0);
    action_timeout_ = declare_parameter<double>("action_timeout", 0.0);
  }

  rclcpp_action::GoalResponse handleGoal(std::shared_ptr<const NavigateToPose::Goal> goal)
  {
    if (goal->pose.header.frame_id.empty()) {
      RCLCPP_WARN(get_logger(), "Rejecting NavigateToPose goal with empty frame_id");
      return rclcpp_action::GoalResponse::REJECT;
    }

    std::lock_guard<std::mutex> lock(active_goal_mutex_);
    if (active_goal_) {
      RCLCPP_WARN(get_logger(), "Preempting active NavigateToPose goal with a new goal");
    }
    active_goal_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  std::string getLatestStatus()
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return latest_status_;
  }

  bool getRobotPose(geometry_msgs::msg::PoseStamped & pose)
  {
    try {
      const auto transform =
        tf_buffer_->lookupTransform(global_frame_, robot_base_frame_, tf2::TimePointZero);
      pose.header.frame_id = global_frame_;
      pose.header.stamp = now();
      pose.pose.position.x = transform.transform.translation.x;
      pose.pose.position.y = transform.transform.translation.y;
      pose.pose.position.z = transform.transform.translation.z;
      pose.pose.orientation = transform.transform.rotation;
      return true;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  void clearActiveGoal()
  {
    std::lock_guard<std::mutex> lock(active_goal_mutex_);
    active_goal_ = false;
  }

  // 只有当前目标仍是「最新代次」时才清 active 标志：被抢占的旧目标退出时
  // 不能把新目标的 active 标志误清掉。
  void clearActiveGoalIfCurrent(uint64_t gen)
  {
    if (goal_generation_.load(std::memory_order_acquire) == gen) {
      clearActiveGoal();
    }
  }

  // rclcpp_action 的 goal handle 在目标已被终止（抢占/取消竞态）时，终端调用
  // 和 publish_feedback 会抛 UnknownGoalHandleError。本节点跑在容器里，任何
  // 未捕获异常都会终止整个进程（线上事故：容器死亡 → 地图/点云全部消失）。
  // 所有 handle 操作统一走安全包装。
  void trySucceed(
    const std::shared_ptr<GoalHandleNavigateToPose> & handle,
    const std::shared_ptr<NavigateToPose::Result> & result)
  {
    try { handle->succeed(result); } catch (const std::exception & ex) {
      RCLCPP_WARN(get_logger(), "succeed() on terminated goal ignored: %s", ex.what());
    } catch (...) {}
  }

  void tryAbort(
    const std::shared_ptr<GoalHandleNavigateToPose> & handle,
    const std::shared_ptr<NavigateToPose::Result> & result)
  {
    try { handle->abort(result); } catch (const std::exception & ex) {
      RCLCPP_WARN(get_logger(), "abort() on terminated goal ignored: %s", ex.what());
    } catch (...) {}
  }

  void tryCancel(
    const std::shared_ptr<GoalHandleNavigateToPose> & handle,
    const std::shared_ptr<NavigateToPose::Result> & result)
  {
    try { handle->canceled(result); } catch (const std::exception & ex) {
      RCLCPP_WARN(get_logger(), "canceled() on terminated goal ignored: %s", ex.what());
    } catch (...) {}
  }

  void tryFeedback(
    const std::shared_ptr<GoalHandleNavigateToPose> & handle,
    const std::shared_ptr<NavigateToPose::Feedback> & feedback)
  {
    try { handle->publish_feedback(feedback); } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "publish_feedback() on terminated goal ignored: %s", ex.what());
    } catch (...) {}
  }

  void executeGoal(
    const std::shared_ptr<GoalHandleNavigateToPose> goal_handle, uint64_t gen)
  {
    // 全程持有句柄直到析构显式终结：保证「最后一个 ServerGoalHandle 引用」
    // 在析构体内、action server 仍存活时释放（见析构函数注释），不会落到
    // 成员销毁顺序导致的「句柄比 server 活得久 → 析构中 publish_result 抛
    // 异常 → std::terminate」路径上。
    active_handle_ = goal_handle;
    const auto goal = goal_handle->get_goal();
    auto goal_pose = goal->pose;
    // 没时间戳就补当前时间。
    if (goal_pose.header.stamp.sec == 0 && goal_pose.header.stamp.nanosec == 0) {
      goal_pose.header.stamp = now();
    }
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      latest_status_.clear();
    }
    goal_pub_->publish(goal_pose);

    RCLCPP_INFO(
      get_logger(), "Accepted NavigateToPose goal at (%.2f, %.2f) in %s",
      goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.header.frame_id.c_str());

    const auto start_time = now();
    rclcpp::Rate rate(std::max(1.0, feedback_frequency_));
    while (rclcpp::ok() && !shutdown_) {
      // 新目标被接受（代次已变）：本目标被抢占，取消后退出，让执行线程
      // 去跑槽位里的新目标。
      if (goal_generation_.load(std::memory_order_acquire) != gen) {
        tryCancel(goal_handle, std::make_shared<NavigateToPose::Result>());
        RCLCPP_INFO(get_logger(), "NavigateToPose goal preempted by a new goal");
        return;
      }

      if (goal_handle->is_canceling()) {
        tryCancel(goal_handle, std::make_shared<NavigateToPose::Result>());
        clearActiveGoalIfCurrent(gen);
        RCLCPP_INFO(get_logger(), "NavigateToPose goal canceled");
        return;
      }

      const auto status = getLatestStatus();
      if (status == "GOAL_REACHED") {
        trySucceed(goal_handle, std::make_shared<NavigateToPose::Result>());
        clearActiveGoalIfCurrent(gen);
        RCLCPP_INFO(get_logger(), "NavigateToPose goal succeeded");
        return;
      }
      if (status == "BLOCKED" || status == "PATH_TIMEOUT") {
        tryAbort(goal_handle, std::make_shared<NavigateToPose::Result>());
        clearActiveGoalIfCurrent(gen);
        RCLCPP_WARN(get_logger(), "NavigateToPose goal aborted: %s", status.c_str());
        return;
      }
      if (action_timeout_ > 0.0 && (now() - start_time).seconds() > action_timeout_) {
        tryAbort(goal_handle, std::make_shared<NavigateToPose::Result>());
        clearActiveGoalIfCurrent(gen);
        RCLCPP_WARN(get_logger(), "NavigateToPose goal timed out");
        return;
      }

      auto feedback = std::make_shared<NavigateToPose::Feedback>();
      geometry_msgs::msg::PoseStamped current_pose;
      if (getRobotPose(current_pose)) {
        feedback->current_pose = current_pose;
        feedback->distance_remaining = static_cast<float>(
          std::hypot(
            goal_pose.pose.position.x - current_pose.pose.position.x,
            goal_pose.pose.position.y - current_pose.pose.position.y));
      }
      const int64_t navigation_time_ns = (now() - start_time).nanoseconds();
      feedback->navigation_time.sec = static_cast<int32_t>(navigation_time_ns / 1000000000LL);
      feedback->navigation_time.nanosec =
        static_cast<uint32_t>(navigation_time_ns % 1000000000LL);
      tryFeedback(goal_handle, feedback);
      rate.sleep();
    }

    // 循环退出但目标仍在（rclcpp 关停/组件卸载）：不动 goal handle ——
    // 析构顺序保证本线程先 join，action server 随后自己收尾所有目标。
    clearActiveGoalIfCurrent(gen);
  }

  std::string global_frame_;
  std::string robot_base_frame_;
  std::string action_name_;
  std::string goal_topic_;
  std::string status_topic_;
  double feedback_frequency_{10.0};
  double action_timeout_{0.0};

  std::mutex status_mutex_;
  std::string latest_status_;

  std::mutex active_goal_mutex_;
  bool active_goal_{false};

  // 目标执行线程 + 槽位。单线程串行执行，新目标通过代次抢占旧目标。
  std::thread exec_thread_;
  std::mutex goal_slot_mutex_;
  std::condition_variable goal_slot_cv_;
  std::pair<std::shared_ptr<GoalHandleNavigateToPose>, uint64_t> pending_goal_;
  std::atomic<uint64_t> goal_generation_{0};
  std::atomic<bool> shutdown_{false};
  // 当前正在执行/最后一个执行过的目标句柄（仅 exec 线程写，析构在 join 后读），
  // 用于析构时显式终结，规避 Humble 句柄析构自动 cancel 的竞态。
  std::shared_ptr<GoalHandleNavigateToPose> active_handle_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace navigation2

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(navigation2::RmNav2Compat)

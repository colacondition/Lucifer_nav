#include <algorithm>
#include <atomic>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

#include <decision_interfaces/msg/gimbal_posture.hpp>
#include <decision_interfaces/msg/gimbal_posture_state.hpp>
#include <decision_interfaces/msg/semantic_map.hpp>

#include "local_path_safety.hpp"
#include "semantic_map_consumer.hpp"
#include "mpc/mpc_solver.hpp"
#include "mpc/path_reference.hpp"
#include "mpc/progress_monitor.hpp"
#include "mpc/recovery_planner.hpp"
#include "mpc/route_tracker.hpp"
#include "mpc/speed_profile.hpp"
#include "utils/pose_predictor.hpp"
#include "utils/rcl_tf.hpp"

namespace navigation2
{

namespace
{

// 全局规划器以 planning_frequency 定频发布：没有触发重规划时会走
// publishLastPath() 原样重发上一条路径。若把「收到 Path 消息」当成
// 「路径变了」，tracker 会每拍被重置，多假设与单调进度都无从累积。
// 因此这里比几何而不是比消息。
bool samePathGeometry(
  const std::vector<Eigen::Vector2d> & previous, const nav_msgs::msg::Path & path,
  double epsilon = 1e-4)
{
  if (previous.size() != path.poses.size()) {
    return false;
  }
  for (std::size_t i = 0; i < previous.size(); ++i) {
    const auto & position = path.poses[i].pose.position;
    if (std::abs(previous[i].x() - position.x) > epsilon ||
      std::abs(previous[i].y() - position.y) > epsilon)
    {
      return false;
    }
  }
  return true;
}

std::vector<Eigen::Vector2d> pathPoints(const nav_msgs::msg::Path & path)
{
  std::vector<Eigen::Vector2d> points;
  points.reserve(path.poses.size());
  for (const auto & pose : path.poses) {
    points.emplace_back(pose.pose.position.x, pose.pose.position.y);
  }
  return points;
}

double normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

// 执行层状态。原来的 control() 是无状态的：每拍要么跟踪要么停车，异常一律
// 「停车 + 请求重规划」。车贴到障碍上时规划器的起点检查过不了，于是停车 →
// 重规划失败 → 继续停车，死循环。恢复链的作用是先物理脱困再谈规划。
enum class NavState
{
  Follow,          // 正常跟踪。
  StuckReverse,    // 定速倒退固定距离。
  HazardRecovery,  // 采样安全点并驶入，保持一段时间。
  Failed,          // 恢复用尽，停车报错，等新目标。
};

}  // namespace

class RmMpcController : public rclcpp::Node
{
public:
  explicit RmMpcController(const rclcpp::NodeOptions & options)
  : Node("rm_mpc_controller", options)
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    path_topic_ = declare_parameter<std::string>("path_topic", "/plan");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/Odometry");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_nav");
    target_frame_ = declare_parameter<std::string>("target_frame", "map");
    // 与全局规划器保持同一坐标系。
    robot_base_frame_ = declare_parameter<std::string>("robot_base_frame", "base_link_fake");
    use_tf_pose_ = declare_parameter<bool>("use_tf_pose", true);
    control_fps_ = declare_parameter<double>("control_fps", 30.0);
    expected_speed_ = declare_parameter<double>("expected_speed", 1.5);
    goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.2);
    // 接近段（原 goal_approach_controller）：到点减速/对准/置零并进本节点，
    // 避免 30 Hz QP 与下游覆写之间的容差死区。恢复期和航点中间点仍可通过
    // approach_enabled_topic 关掉。approach.enable=false 时整段不生效（测试用）。
    approach_enable_ = declare_parameter<bool>("approach.enable", true);
    approach_distance_ = declare_parameter<double>("approach.distance", 1.5);
    approach_velocity_ = declare_parameter<double>("approach.velocity", 0.5);
    direct_approach_distance_ = declare_parameter<double>("approach.direct_distance", 0.5);
    direct_approach_kp_ = declare_parameter<double>("approach.direct_kp", 1.0);
    max_track_error_ = declare_parameter<double>("max_track_error", 0.5);
    blind_radius_ = declare_parameter<double>("blind_radius", 0.1);
    delay_time_ = declare_parameter<double>("delay_time", 0.0);
    default_wz_ = declare_parameter<double>("default_wz", 0.0);

    use_delay_comp_ = declare_parameter<bool>("delay_compensation.enable", false);
    delay_comp_max_dt_ = declare_parameter<double>("delay_compensation.max_dt", 0.5);

    local_costmap_topic_ = declare_parameter<std::string>(
      "local_safety.costmap_topic", "/local_costmap/costmap");
    safety_policy_.obstacle_threshold = declare_parameter<int>(
      "local_safety.obstacle_threshold", 50);
    safety_policy_.unknown_is_obstacle = declare_parameter<bool>(
      "local_safety.unknown_is_obstacle", true);
    local_costmap_timeout_ = declare_parameter<double>("local_safety.costmap_timeout", 0.5);
    safety_policy_.check_steps = declare_parameter<int>("local_safety.check_steps", 10);
    replan_topic_ = declare_parameter<std::string>(
      "local_safety.replan_topic", "/navigation2/replan_request");
    replan_cooldown_ = declare_parameter<double>("local_safety.replan_cooldown", 0.5);

    // 指令反馈。本节点发布的指令仍会被下游改写：
    //   MPC -> /cmd_vel_nav -> rm_velocity_smoother -> /cmd_vel
    //       -> fake_vel_transform -> 底盘
    // 接近段已并进本节点；velocity_smoother 仍会限幅并在输入超时后归零。
    // 「本节点发了多少」不能当作「车被驱动了多少」—— 用它做卡住判据会被喂假数据。
    // 这里订阅链路末端，用真实执行值喂失效检测。
    executed_cmd_topic_ = declare_parameter<std::string>("feedback.executed_cmd_topic", "/cmd_vel");
    // rm_velocity_smoother 以 smoothing_frequency 无条件定频发布，所以这条流
    // 是连续的：超时意味着下游真的停了，而不是「本来就没有指令」。
    executed_cmd_timeout_ = declare_parameter<double>("feedback.executed_cmd_timeout", 0.3);
    // 云台收放：rm_tunnel_posture 发请求，serial_driver 转发电控回传的实测姿态。
    // 请求「收下来」而回传还没变成「低」时，本节点停车等 —— 判据见 control()。
    gimbal_posture_topic_ = declare_parameter<std::string>(
      "gimbal.posture_topic", "/gimbal_posture");
    gimbal_posture_state_topic_ = declare_parameter<std::string>(
      "gimbal.posture_state_topic", "/gimbal_posture_state");
    // 本节点发非零、下游却持续为零，说明指令被覆写。持续这么久才报，避开
    // velocity_smoother 的正常加速爬坡（4.0 m/s² @ 20 Hz，0→1.5 m/s 约 0.375 s）。
    override_detect_time_ = declare_parameter<double>("feedback.override_detect_time", 1.0);

    // 弧长进度跟踪参数（多假设，替代单帧最近点投影）。
    mpc::RouteTrackerParams tracker_params;
    tracker_params.hypothesis_spacing =
      declare_parameter<double>("tracker.hypothesis_spacing", tracker_params.hypothesis_spacing);
    tracker_params.search_window =
      declare_parameter<double>("tracker.search_window", tracker_params.search_window);
    tracker_params.refine_step =
      declare_parameter<double>("tracker.refine_step", tracker_params.refine_step);
    tracker_params.max_hypotheses = static_cast<std::size_t>(std::max<int>(
      1, declare_parameter<int>(
        "tracker.max_hypotheses", static_cast<int>(tracker_params.max_hypotheses))));
    tracker_params.position_sigma =
      declare_parameter<double>("tracker.position_sigma", tracker_params.position_sigma);
    tracker_params.direction_weight =
      declare_parameter<double>("tracker.direction_weight", tracker_params.direction_weight);
    tracker_params.min_speed_for_direction = declare_parameter<double>(
      "tracker.min_speed_for_direction", tracker_params.min_speed_for_direction);
    tracker_params.weight_floor =
      declare_parameter<double>("tracker.weight_floor", tracker_params.weight_floor);
    tracker_params.merge_distance =
      declare_parameter<double>("tracker.merge_distance", tracker_params.merge_distance);
    tracker_params.seed_prior_length =
      declare_parameter<double>("tracker.seed_prior_length", tracker_params.seed_prior_length);
    tracker_params.arc_rate_alpha =
      declare_parameter<double>("tracker.arc_rate_alpha", tracker_params.arc_rate_alpha);
    tracker_params.velocity_alpha =
      declare_parameter<double>("tracker.velocity_alpha", tracker_params.velocity_alpha);
    // 跟踪丢失沿用既有的 max_track_error 阈值，只是改为需要持续超限。
    tracker_params.max_track_error = max_track_error_;
    tracker_params.lost_grace_time =
      declare_parameter<double>("tracker.lost_grace_time", tracker_params.lost_grace_time);
    route_tracker_.configure(tracker_params);

    // 失效检测参数。判据基于世界系位移而非弧长里程碑：规划器每移动
    // min_replan_distance 就换一次路径，弧长原点随之重置，弧长计时器
    // 累计不到超时阈值。
    mpc::ProgressMonitorParams progress_params;
    progress_params.min_displacement =
      declare_parameter<double>("progress.min_displacement", progress_params.min_displacement);
    progress_params.no_progress_timeout = declare_parameter<double>(
      "progress.no_progress_timeout", progress_params.no_progress_timeout);
    progress_params.stuck_timeout =
      declare_parameter<double>("progress.stuck_timeout", progress_params.stuck_timeout);
    progress_params.cmd_epsilon =
      declare_parameter<double>("progress.cmd_epsilon", progress_params.cmd_epsilon);
    progress_monitor_.configure(progress_params);

    // 弧长域速度剖面：按曲率限侧向加速度，前/后向扫描保证加减速可达。
    // 替代原来的 expected_speed 常数 —— 过弯不减速时参考轨迹本身不可跟踪。
    speed_profile_enabled_ = declare_parameter<bool>("speed_profile.enable", true);
    {
      mpc::SpeedProfileParams sp_params;
      sp_params.max_lateral_accel = declare_parameter<double>(
        "speed_profile.max_lateral_accel", sp_params.max_lateral_accel);
      sp_params.max_tangential_accel = declare_parameter<double>(
        "speed_profile.max_tangential_accel", sp_params.max_tangential_accel);
      sp_params.max_tangential_decel = declare_parameter<double>(
        "speed_profile.max_tangential_decel", sp_params.max_tangential_decel);
      sp_params.min_speed = declare_parameter<double>(
        "speed_profile.min_speed", sp_params.min_speed);
      sp_params.sample_spacing = declare_parameter<double>(
        "speed_profile.sample_spacing", sp_params.sample_spacing);
      sp_params.curvature_window = declare_parameter<int>(
        "speed_profile.curvature_window", sp_params.curvature_window);
      sp_params.stop_at_goal = declare_parameter<bool>(
        "speed_profile.stop_at_goal", sp_params.stop_at_goal);
      speed_profile_.configure(sp_params);
    }

    // 隧道限速窗口：速度剖面在洞内的样本按 TunnelSpec 的 [vmin, vmax] 夹。收不到
    // 语义地图或图里没隧道时 tunnelSpecAtPoint 返回 nullptr，窗口不生效。
    tunnel_speed_window_enabled_ = declare_parameter<bool>("tunnel_speed_window.enable", true);
    semantic_map_topic_ = declare_parameter<std::string>(
      "semantic_map_topic", "/map_server/semantic_map");
    if (tunnel_speed_window_enabled_) {
      speed_profile_.set_tunnel_window(
        [this](const Eigen::Vector2d & p, double & vmin, double & vmax) {
          std::lock_guard<std::mutex> lk(map_mutex_);
          const TunnelSpec * spec = tunnelSpecAtPoint(receiver_.map(), p.x(), p.y());
          if (spec == nullptr) {
            return false;
          }
          vmin = spec->velocity_min;
          vmax = spec->velocity_max;
          return true;
        });
    }

    // 恢复链参数。车贴到障碍上后规划器起点检查过不了，必须先物理脱困再
    // 重规划，否则会陷入「停车 -> 请求重规划 -> 起点不可行 -> 继续停车」。
    recovery_enabled_ = declare_parameter<bool>("recovery.enable", true);
    recovery_reverse_speed_ = declare_parameter<double>("recovery.reverse_speed", 0.3);
    recovery_reverse_distance_ = declare_parameter<double>("recovery.reverse_distance", 0.4);
    recovery_max_speed_ = declare_parameter<double>("recovery.max_speed", 0.4);
    recovery_kp_ = declare_parameter<double>("recovery.kp", 1.0);
    recovery_reach_tolerance_ = declare_parameter<double>("recovery.reach_tolerance", 0.12);
    recovery_dwell_time_ = declare_parameter<double>("recovery.dwell_time", 0.4);
    recovery_max_duration_ = declare_parameter<double>("recovery.max_duration", 10.0);
    recovery_max_attempts_ = declare_parameter<int>("recovery.max_attempts", 3);
    goal_change_threshold_ = declare_parameter<double>("recovery.goal_change_threshold", 0.3);
    // 跟踪丢失 / 求解失败 / 安全否决这三类否决持续多久才升级为物理脱困。
    // 单帧尖峰（一帧坏点云、一次 QP 抖动）由重规划快路径处理，不该进恢复；
    // 只有持续否决才说明重规划解决不了问题。
    veto_recovery_time_ = declare_parameter<double>("recovery.veto_recovery_time", 1.5);
    // 目标附近抑制失效检测。接近段已并进本节点，goal_tolerance 是唯一到点半径；
    // 这条带仍要比它宽一圈，避免到点蠕动被 stuck 判据接走。
    recovery_suppress_near_goal_ =
      declare_parameter<double>("recovery.suppress_near_goal", 0.35);
    if (recovery_suppress_near_goal_ < goal_tolerance_) {
      RCLCPP_WARN(
        get_logger(),
        "recovery.suppress_near_goal (%.2f m) < goal_tolerance (%.2f m); "
        "failure detection may fire while parked at the goal",
        recovery_suppress_near_goal_, goal_tolerance_);
    }
    approach_enabled_topic_ = declare_parameter<std::string>(
      "recovery.approach_enabled_topic", "/goal_approach_controller/enabled");

    // 恢复期的危险判据与通行判据。注意通行只看 lethal：车已经在膨胀圈里，
    // 用正常的 obstacle_threshold(50) 检查会让所有方向立即被否。
    hazard_policy_.hazard_cost = declare_parameter<int>("recovery.hazard_cost", 80);
    hazard_policy_.lethal_cost = declare_parameter<int>("recovery.lethal_cost", 99);
    hazard_policy_.unknown_is_hazard =
      declare_parameter<bool>("recovery.unknown_is_hazard", true);
    hazard_policy_.out_of_map_is_hazard =
      declare_parameter<bool>("recovery.out_of_map_is_hazard", true);

    safe_point_params_.ring_radii = declare_parameter<std::vector<double>>(
      "recovery.ring_radii", std::vector<double>{0.3, 0.6, 0.9, 1.2});
    safe_point_params_.samples_per_ring =
      declare_parameter<int>("recovery.samples_per_ring", 12);
    safe_point_params_.distance_penalty =
      declare_parameter<double>("recovery.distance_penalty", 20.0);

    mpc::MpcParams mp;
    mp.steps = declare_parameter<int>("predict_steps", 30);
    mp.dt = declare_parameter<double>("predict_dt", 0.1);
    mp.max_speed = declare_parameter<double>("max_speed", 2.0);
    mp.max_accel = declare_parameter<double>("max_accel", 1.0);
    mp.turtle_max_speed = declare_parameter<double>("turtle_max_speed", 1.0);
    mp.Q = declare_parameter<std::vector<double>>("Q", std::vector<double>{15.0, 15.0});
    mp.R = declare_parameter<std::vector<double>>("R", std::vector<double>{0.1, 0.1});
    mp.Rd = declare_parameter<std::vector<double>>("Rd", std::vector<double>{1.0, 0.05});
    // 参数非法时回退到安全默认值而不是把节点带崩：负 steps 会在 configure()
    // 里转成巨大 resize，过短的 Q/R/Rd 会越界。configure() 本身也保留同样的
    // 库级校验作为第二道防线。
    if (!mpc::paramsAreValid(mp)) {
      RCLCPP_ERROR(
        get_logger(),
        "Invalid MPC parameters (predict_steps/predict_dt/Q/R/Rd); "
        "falling back to defaults");
      mp = mpc::MpcParams{};
    }
    steps_ = mp.steps;
    predict_dt_ = mp.dt;
    solver_.configure(mp);

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_, rclcpp::QoS(1).reliable(),
      [this](nav_msgs::msg::Path::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lk(mtx_);
        // 只有几何真的变了才算换路径：规划器未重规划时会原样重发。
        if (samePathGeometry(last_path_points_, *msg)) {
          return;
        }
        last_path_points_ = pathPoints(*msg);
        ref_.set_path(*msg, expected_speed_);
        has_path_ = ref_.valid();
        // 路径换了，弧长坐标系原点也跟着换，标记让控制线程重锚 tracker。
        path_changed_ = true;

        // FAILED 态要靠「新目标」才能解除。MPC 只订阅 /plan，不订阅
        // /goal_pose，所以用路径终点代表目标：终点移动超过阈值才算新目标。
        // 不能用「收到路径」代替——规划器会定频重发、重规划也会换路径，
        // 那样 FAILED 会被立刻清掉，恢复次数上限形同虚设。
        if (!msg->poses.empty()) {
          const auto & last = msg->poses.back().pose.position;
          const Eigen::Vector2d goal(last.x, last.y);
          if (!last_goal_ || (*last_goal_ - goal).norm() > goal_change_threshold_) {
            goal_changed_ = true;
          }
          last_goal_ = goal;
        }
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lk(mtx_);
        odom_ = *msg;
        has_odom_ = true;
      });
    const auto costmap_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    local_costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      local_costmap_topic_, costmap_qos,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lk(mtx_);
        local_costmap_ = std::move(msg);
      });

    // 指令链路末端反馈。本节点的输出要经过 rm_velocity_smoother（限加速度、
    // 超时归零）才到底盘。订阅链路末端把开环变成闭环：失效判据必须用
    // 「底盘实际收到什么」而不是「本节点想发什么」。
    executed_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      executed_cmd_topic_, rclcpp::QoS(10),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lk(mtx_);
        executed_speed_ = std::hypot(msg->linear.x, msg->linear.y);
        last_executed_cmd_time_ = now();
      });

    // 云台收放的请求与实测。两条都订：请求告诉我们「要不要等」，实测告诉我们「等到了没」。
    // 只看请求会在云台还立着的时候就放车走；只看实测则分不清「云台是高的」和「本来就该是高的」。
    const auto posture_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    gimbal_posture_sub_ = create_subscription<decision_interfaces::msg::GimbalPosture>(
      gimbal_posture_topic_, posture_qos,
      [this](decision_interfaces::msg::GimbalPosture::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lk(mtx_);
        gimbal_lower_requested_ = msg->lower;
      });
    gimbal_posture_state_sub_ = create_subscription<decision_interfaces::msg::GimbalPostureState>(
      gimbal_posture_state_topic_, posture_qos,
      [this](decision_interfaces::msg::GimbalPostureState::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lk(mtx_);
        gimbal_lowered_ = msg->lowered;
        has_gimbal_state_ = true;
      });

    // 语义地图只给隧道限速窗口用。同源同 QoS（transient_local），收不到时静默失效。
    if (tunnel_speed_window_enabled_) {
      const auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
      semantic_map_sub_ = create_subscription<decision_interfaces::msg::SemanticMap>(
        semantic_map_topic_, map_qos,
        [this](decision_interfaces::msg::SemanticMap::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lk(map_mutex_);
          try {
            receiver_.update(*msg);
          } catch (const std::exception & ex) {
            RCLCPP_ERROR(get_logger(), "Rejected semantic map: %s", ex.what());
          }
        });
    }

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, rclcpp::QoS(1));
    cmd_norm_pub_ = create_publisher<std_msgs::msg::Float64>("/cmd_vel_norm", rclcpp::QoS(1));
    predict_path_pub_ = create_publisher<nav_msgs::msg::Path>("predict_path", rclcpp::QoS(1));
    vel_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/vel_marker", rclcpp::QoS(1));
    state_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/now_state_marker", rclcpp::QoS(1));
    replan_pub_ = create_publisher<std_msgs::msg::Empty>(replan_topic_, rclcpp::QoS(1));
    // transient_local：航点执行器与本节点都写这个开关。恢复期关掉接近段，
    // 否则 0.5 m 内的对准会把倒车指令吃掉；follow 中间点也会临时关掉。
    approach_enabled_pub_ = create_publisher<std_msgs::msg::Bool>(
      approach_enabled_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    approach_enabled_sub_ = create_subscription<std_msgs::msg::Bool>(
      approach_enabled_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        approach_enabled_.store(msg->data, std::memory_order_relaxed);
      });
    // 初始状态：FOLLOW 态下接近段启用。
    // transient_local 只在有消息时 latch；不先发一条，后启动的订阅方永远不会
    // 收到任何值，集成测试里 approach_enabled 列表始终为空。
    setApproachEnabled(true);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, control_fps_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period), [this]() { control(); });

    RCLCPP_INFO(
      get_logger(),
      "rm_mpc_controller ready: path=%s odom=%s costmap=%s cmd=%s delay_comp=%s",
      path_topic_.c_str(), odom_topic_.c_str(), local_costmap_topic_.c_str(),
      cmd_vel_topic_.c_str(),
      use_delay_comp_ ? "on" : "off");
  }

private:
  // 优先用 TF，失败再退回里程计。
  bool currentState(
    const nav_msgs::msg::Odometry & odom, Eigen::Vector2d & pos, double & yaw)
  {
    if (use_tf_pose_) {
      try {
        const auto tf = tf_buffer_->lookupTransform(
          target_frame_, robot_base_frame_, tf2::TimePointZero);
        pos = Eigen::Vector2d(
          tf.transform.translation.x, tf.transform.translation.y);
        yaw = tf2::getYaw(tf.transform.rotation);
        return true;
      } catch (const tf2::TransformException & ex) {
        // 退回里程计原始位姿意味着定位链断了，必须可见。节流到 2 s。
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "TF %s->%s unavailable, falling back to raw odom pose: %s",
          target_frame_.c_str(), robot_base_frame_.c_str(), ex.what());
      }
    }
    geometry_msgs::msg::Pose pose = odom.pose.pose;
    if (use_delay_comp_) {
      pose = navigation2::utils::predict_pose(odom, now(), delay_comp_max_dt_).pose;
    }
    pos = Eigen::Vector2d(pose.position.x, pose.position.y);
    yaw = tf2::getYaw(pose.orientation);
    return true;
  }

  void control()
  {
    nav_msgs::msg::Odometry odom;
    mpc::PathReference ref;
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr local_costmap;
    bool ready;
    bool path_changed;
    bool goal_changed;
    double executed_speed;
    std::optional<rclcpp::Time> executed_stamp;
    bool gimbal_lower_requested;
    bool gimbal_lowered;
    bool has_gimbal_state;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      ready = has_path_ && has_odom_;
      odom = odom_;
      ref = ref_;
      local_costmap = local_costmap_;
      path_changed = path_changed_;
      path_changed_ = false;
      goal_changed = goal_changed_;
      goal_changed_ = false;
      executed_speed = executed_speed_;
      executed_stamp = last_executed_cmd_time_;
      gimbal_lower_requested = gimbal_lower_requested_;
      gimbal_lowered = gimbal_lowered_;
      has_gimbal_state = has_gimbal_state_;
    }

    // 控制周期实测值。定时器抖动、以及仿真时钟下 wall timer 与 /clock 的
    // 步长差异都会让名义 1/control_fps 失真，进度跟踪的积分要用实测值。
    const rclcpp::Time control_stamp = now();
    double dt = 1.0 / std::max(1.0, control_fps_);
    if (last_control_time_) {
      const double measured = (control_stamp - *last_control_time_).seconds();
      // 时钟跳变（仿真重置、use_sim_time 初值）时退回名义周期。
      if (std::isfinite(measured) && measured > 0.0 && measured < 1.0) {
        dt = measured;
      }
    }
    last_control_time_ = control_stamp;

    // 链路末端反馈的有效性。rm_velocity_smoother 以 smoothing_frequency 无条件
    // 定频发布（输入超时它自己会把目标归零后继续发零），所以这条话题正常情况
    // 下是连续流；一旦静默就说明平滑器本身没在跑，而不是「没有指令」。
    const bool feedback_fresh = executed_stamp &&
      (control_stamp - *executed_stamp).seconds() <= std::max(0.0, executed_cmd_timeout_);
    if (!feedback_fresh) {
      // 过期就按零处理：不能拿旧值继续喂失效判据。
      executed_speed = 0.0;
    }
    if (!executed_stamp) {
      // 从未收到过反馈。整条链路可能没起（单独调试 MPC）或平滑器挂了。
      // 此时 stuck 判据失效（永远读到零），只剩 noProgress 兜底，必须说清楚。
      RCLCPP_WARN_ONCE(
        get_logger(),
        "No message ever received on %s; stuck detection is degraded to noProgress only. "
        "Is rm_velocity_smoother running?",
        executed_cmd_topic_.c_str());
    } else if (!feedback_fresh) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Feedback on %s is stale (>%.2f s); treating executed speed as zero",
        executed_cmd_topic_.c_str(), executed_cmd_timeout_);
    }

    // 越权检测（可观测，不改变行为）。下游把本节点的非零指令压成零并持续
    // 一段时间，就是「有指令但车不动」这类故障的真正来源。
    // 平滑器的加减速斜坡是合法的短暂偏离（max_accel 4.0 × 0.05 s = 0.2 m/s
    // 每拍），所以用持续时间而不是单帧幅值差来判定。
    const double cmd_epsilon = progress_monitor_.params().cmd_epsilon;
    if (feedback_fresh && last_published_speed_ > cmd_epsilon && executed_speed <= cmd_epsilon) {
      override_time_ += dt;
      if (override_time_ >= std::max(override_detect_time_, 0.0)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "Downstream is zeroing our command: published %.2f m/s but %s reports %.2f m/s "
          "for %.1f s (check rm_velocity_smoother)",
          last_published_speed_, executed_cmd_topic_.c_str(), executed_speed, override_time_);
      }
    } else {
      override_time_ = 0.0;
    }

    if (path_changed) {
      route_tracker_.on_path_replaced();
      if (speed_profile_enabled_) {
        speed_profile_.rebuild(ref, expected_speed_);
      }
    }

    // FAILED 只由「真的换了目标」解除，而不是由「收到 Path 消息」解除：
    // 规划器会以 planning_frequency 原样重发，用后者会让 FAILED 立刻被清掉。
    if (goal_changed && nav_state_ == NavState::Failed) {
      nav_state_ = NavState::Follow;
      recovery_attempts_ = 0;
      recovery_elapsed_ = 0.0;
      recovery_dwell_ = 0.0;
      safe_point_.reset();
      setApproachEnabled(true);
      progress_monitor_.reset();
    }

    if (!ready) {
      publishStop();
      // 没有路径/里程计时车本来就不该动，不能算无进展。
      progress_monitor_.reset();
      return;
    }

    // 等云台真的收下来再走。请求「收下来」而回传还是「高」，就停在原地。
    //
    // 电控的执行是可信的，这里不做重试、不设超时 —— 这个门只是在等那一小段动作时间。
    // 提前量（run_up）是按距离给的，正常情况下车走到洞口之前回传早就翻过来了，一帧都不会停。
    //
    // 「还没收到过任何回传」这种情况不需要单独判：gimbal_lowered_ 的初值就是 false，所以
    // 电控没起来、回传帧没实现、串口没插，一律落在「没收好」这一侧 —— 车停着等。这个初值
    // 是安全方向的选择，翻过来的代价是云台撞在顶板上。改它之前先看
    // test_gimbal_gate_no_feedback.py。
    //
    // 只在请求收的方向拦。请求抬起而云台还没抬起来只是少打一会儿，不值得停车。
    if (gimbal_lower_requested && !gimbal_lowered) {
      publishStop();
      // 不算无进展：车是被我们自己拦住的，不是卡住了。让 stuck 判据在这里累积会触发
      // 倒车恢复 —— 云台还立着的时候倒车，只是把撞击换个方向。
      progress_monitor_.reset();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "等云台收下来再进洞：%s。车已停住。",
        has_gimbal_state ? "电控回传的姿态还是「高」" : "还没收到过电控的姿态回传");
      return;
    }

    Eigen::Vector2d pos;
    double yaw;
    currentState(odom, pos, yaw);

    // ---- 恢复链 FSM 调度。恢复态优先于一切正常跟踪逻辑：卡住就是卡住，
    // 离目标多近都不改变这个事实。----
    if (nav_state_ == NavState::Failed) {
      publishStop();
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Navigation FAILED after %d recovery attempts; waiting for a new goal",
        recovery_max_attempts_);
      return;
    }

    if (nav_state_ == NavState::StuckReverse || nav_state_ == NavState::HazardRecovery) {
      recovery_elapsed_ += dt;
      // 双重上限，避免在两个恢复态之间无限振荡。
      if (recovery_elapsed_ > recovery_max_duration_) {
        publishStop();
        failRecovery("recovery exceeded max duration");
        return;
      }
      if (recovery_attempts_ > recovery_max_attempts_) {
        publishStop();
        failRecovery("recovery attempts exhausted");
        return;
      }
      if (nav_state_ == NavState::StuckReverse) {
        runStuckReverse(local_costmap, pos, yaw);
      } else {
        runHazardRecovery(local_costmap, pos, yaw, dt);
      }
      return;
    }

    const double goal_distance = (ref.goal() - pos).norm();

    // 到达目标后直接停止跟踪。
    if (goal_distance < goal_tolerance_) {
      publishStop();
      // 到达目标后的静止是正常的。
      progress_monitor_.reset();
      return;
    }

    // 多假设弧长进度跟踪，替代原来的全路径最近点扫描（回绕路径上会跳变、
    // 且进度可回退）。
    route_tracker_.update(ref, pos, dt);

    // 目标附近关掉失效检测。接近段已并进本节点，到点静止是期望行为。
    const bool near_goal = goal_distance < recovery_suppress_near_goal_;
    if (near_goal) {
      progress_monitor_.reset();
    } else {
      // 失效检测喂的是**链路末端实际下发的**速率，不是本节点发布的指令。
      // 下游 rm_velocity_smoother 仍会限幅或超时归零。
      progress_monitor_.update(pos, executed_speed, dt);

      // 这是打断「停车 → 重规划 → 起点不可行 → 继续停车」死循环的关键：
      // 安全检查反复否决时下发速度恒为零、位移不增长，noProgress 会在
      // no_progress_timeout 后触发，把控制权交给物理脱困。
      if (recovery_enabled_ && (progress_monitor_.stuck() || progress_monitor_.noProgress())) {
        const bool is_stuck = progress_monitor_.stuck();
        publishStop();
        enterRecovery(
          NavState::StuckReverse, pos,
          is_stuck ? "commanding velocity but not moving" : "no displacement progress");
        return;
      }
    }

    if (progress_monitor_.stuck()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Stuck: chassis commanded %.2f m/s but displacement < %.2f m for %.1f s (recovery disabled)",
        executed_speed, progress_monitor_.params().min_displacement,
        progress_monitor_.commandedStagnantTime());
    } else if (progress_monitor_.noProgress()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No progress: displacement < %.2f m for %.1f s (recovery disabled)",
        progress_monitor_.params().min_displacement, progress_monitor_.stagnantTime());
    }

    if (route_tracker_.lost()) {
      // 残差持续超限才判丢失，单帧尖峰不再直接触发重规划。
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Route tracking lost (residual %.2f m > %.2f m)", route_tracker_.residual(),
        max_track_error_);
      handleVeto(local_costmap, pos, dt, "route tracking lost");
      return;
    }

    // 生成参考窗口，太近的点跳过。弧长域采样：s = s_now + speed * dt * i。
    Eigen::MatrixXd xref(2, steps_);
    Eigen::MatrixXd uref(2, steps_);
    const double total_s = ref.total_length();
    const double speed_nominal = ref.expected_speed();
    const double ds = speed_nominal * predict_dt_;
    // ds <= 0 时下面的「太近就跳过」分支永远推不动 s，会死循环。这只可能来自
    // expected_speed / predict_dt 被配成 0，属于配置错误，在这里一次性拦掉。
    // 注意不能用 break 退出采样循环：xref/uref 是 Eigen::MatrixXd，不做零初始化，
    // 提前跳出会把未初始化内存喂给 QP。
    if (!(ds > 0.0)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Invalid reference step ds=%.6f (expected_speed=%.3f, predict_dt=%.3f); cannot build "
        "MPC reference", ds, speed_nominal, predict_dt_);
      handleVeto(local_costmap, pos, dt, "invalid reference step");
      return;
    }
    double s = route_tracker_.arc_length() + ds + speed_nominal * delay_time_;
    for (int i = 0; i < steps_; ++i) {
      const double s_clamped = std::min(s, total_s);
      Eigen::Vector2d rp = ref.pos_by_arc(s_clamped);
      // 速度前馈：优先用速度剖面值，不可用时退回常数。
      // 速度剖面按曲率限速，让 MPC 的参考在弯道处不超出可跟踪范围。
      const double v_ref =
        (speed_profile_enabled_ && speed_profile_.valid())
        ? speed_profile_.speed_at(s_clamped)
        : speed_nominal;
      Eigen::Vector2d rv =
        (s <= total_s)
        ? Eigen::Vector2d(ref.tangent_by_arc(s_clamped) * v_ref)
        : Eigen::Vector2d::Zero();
      if (s <= total_s && (rp - pos).norm() < blind_radius_) {
        // s 单调递增（ds > 0 已在循环前保证），越过 total_s 后本分支不再成立，
        // 因此不会死循环。这里不能 break：那样会留下未初始化的 xref/uref 列。
        s += ds;
        --i;
        continue;
      }
      xref(0, i) = rp.x();
      xref(1, i) = rp.y();
      uref(0, i) = rv.x();
      uref(1, i) = rv.y();
      s += ds;
    }

    auto vseq = solver_.solve(xref, uref, pos, /*turtle=*/false);
    if (vseq.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "MPC solve failed");
      handleVeto(local_costmap, pos, dt, "MPC solve failed");
      return;
    }

    // 复用预分配缓冲区，避免 30Hz 分配。
    predicted_positions_buffer_.clear();
    predicted_positions_buffer_.reserve(vseq.size() + 1);
    predicted_positions_buffer_.push_back(pos);
    for (const auto & velocity : vseq) {
      predicted_positions_buffer_.push_back(predicted_positions_buffer_.back() + velocity * predict_dt_);
    }

    if (!costmapFresh(local_costmap) ||
      !isPathSafe(*local_costmap, predicted_positions_buffer_, safety_policy_))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "MPC predicted path is unsafe or local costmap is unavailable");
      handleVeto(local_costmap, pos, dt, "predicted path unsafe");
      return;
    }

    // 走到这里说明本拍产出了可执行指令，否决连击清零。
    veto_streak_ = 0.0;

    // 世界系速度转到底盘系，再套接近段（恢复指令走 publishWorldVelocity，不经过这里）。
    const double vx_w = vseq[0].x();
    const double vy_w = vseq[0].y();
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = std::cos(yaw) * vx_w + std::sin(yaw) * vy_w;
    cmd.linear.y = -std::sin(yaw) * vx_w + std::cos(yaw) * vy_w;
    cmd.angular.z = default_wz_;
    applyApproach(cmd, pos, yaw, ref.goal());
    cmd_pub_->publish(cmd);

    std_msgs::msg::Float64 norm;
    norm.data = std::hypot(cmd.linear.x, cmd.linear.y);
    cmd_norm_pub_->publish(norm);
    // 记下本拍实际下发的速率，下一拍喂给 ProgressMonitor 判定卡住。
    last_published_speed_ = norm.data;

    publishMarkers(pos, vx_w, vy_w);
    publishPredict(vseq, pos);
  }

  void publishStop()
  {
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = default_wz_;
    cmd_pub_->publish(cmd);
    // 停车也要记下来：ProgressMonitor 的 stuck 判据只在有指令时累计，
    // 停车期间不该算卡住（但 noProgress 那一路仍在计时）。
    last_published_speed_ = 0.0;
  }

  // 世界系速度转底盘系并下发。恢复期与正常跟踪共用，保证 last_published_speed_
  // 在两条路径上都被正确记录。
  void publishWorldVelocity(const Eigen::Vector2d & v_world, double yaw)
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = std::cos(yaw) * v_world.x() + std::sin(yaw) * v_world.y();
    cmd.linear.y = -std::sin(yaw) * v_world.x() + std::cos(yaw) * v_world.y();
    // 恢复期保留 default_wz_：哨兵的小陀螺由这个参数驱动，脱困时不该停转。
    cmd.angular.z = default_wz_;
    cmd_pub_->publish(cmd);
    last_published_speed_ = v_world.norm();
  }

  void setApproachEnabled(bool enabled)
  {
    approach_enabled_.store(enabled, std::memory_order_relaxed);
    std_msgs::msg::Bool msg;
    msg.data = enabled;
    approach_enabled_pub_->publish(msg);
  }

  // 接近减速/对准。只改 FOLLOW 指令；恢复期通过 setApproachEnabled(false) 跳过。
  void applyApproach(
    geometry_msgs::msg::Twist & cmd,
    const Eigen::Vector2d & pos,
    double yaw,
    const Eigen::Vector2d & goal)
  {
    if (!approach_enable_ || !approach_enabled_.load(std::memory_order_relaxed)) {
      return;
    }
    const double dx = goal.x() - pos.x();
    const double dy = goal.y() - pos.y();
    const double dist = std::hypot(dx, dy);
    if (dist <= goal_tolerance_) {
      cmd = geometry_msgs::msg::Twist{};
      return;
    }
    const double local_heading = normalizeAngle(std::atan2(dy, dx) - yaw);
    const double target_speed = std::min(approach_velocity_, dist * direct_approach_kp_);
    if (dist < direct_approach_distance_) {
      cmd.linear.x = target_speed * std::cos(local_heading);
      cmd.linear.y = target_speed * std::sin(local_heading);
      cmd.angular.z = 0.0;
    } else if (dist < approach_distance_) {
      const double speed = std::hypot(cmd.linear.x, cmd.linear.y);
      if (speed > approach_velocity_ && speed > 1e-6) {
        const double scale = approach_velocity_ / speed;
        cmd.linear.x *= scale;
        cmd.linear.y *= scale;
        cmd.angular.z *= scale;
      }
    }
  }

  // 进入恢复态。必须关掉接近段：它在距目标 0.5 m 内覆写 linear、容差内置零，
  // 会把倒车指令吃掉。
  void enterRecovery(NavState state, const Eigen::Vector2d & pos, const char * reason)
  {
    nav_state_ = state;
    ++recovery_attempts_;
    recovery_elapsed_ = 0.0;
    recovery_dwell_ = 0.0;
    safe_point_.reset();
    setApproachEnabled(false);
    // 恢复动作本身不该被判成「无进展」。
    progress_monitor_.reset();
    // 否决连击已经兑现成一次恢复，计数清零；否则退出恢复后第一拍再被否决
    // 就会立刻二次进入恢复，尝试次数被瞬间耗尽。
    veto_streak_ = 0.0;

    if (state == NavState::StuckReverse) {
      // 沿路径切向的反方向倒退；速度太小时退回车头反方向。
      const Eigen::Vector2d velocity = route_tracker_.velocity();
      const double vel_norm = velocity.norm();
      if (vel_norm > 0.05) {
        reverse_direction_ = -velocity / vel_norm;
      } else {
        reverse_direction_ = Eigen::Vector2d(-1.0, 0.0);
      }
      reverse_travelled_ = 0.0;
      reverse_last_pos_ = pos;
    }

    RCLCPP_WARN(
      get_logger(), "Entering %s recovery (attempt %d/%d): %s",
      state == NavState::StuckReverse ? "STUCK_REVERSE" : "HAZARD_RECOVERY",
      recovery_attempts_, recovery_max_attempts_, reason);
  }

  // 退出恢复。旧路径的执行生命周期已被打断，必须从当前位置强制重规划，
  // 而不是回到旧路径。
  void exitRecovery(const char * reason)
  {
    // 每次恢复退出只打一行，不在热路径上。
    RCLCPP_INFO(get_logger(), "Recovery finished: %s", reason);
    nav_state_ = NavState::Follow;
    recovery_attempts_ = 0;
    recovery_elapsed_ = 0.0;
    recovery_dwell_ = 0.0;
    safe_point_.reset();
    setApproachEnabled(true);
    progress_monitor_.reset();
    route_tracker_.on_path_replaced();
    veto_streak_ = 0.0;
    forceReplan();
  }

  // 三条「本拍无法正常跟踪」的路径共用的处理：跟踪丢失、QP 求解失败、
  // 预测轨迹被安全检查否决。
  //
  // 快路径保持原样：停车 + 请求重规划。单帧的坏代价图、偶发的 QP 抖动都
  // 应该由重规划解决，不该惊动恢复链。
  //
  // 慢路径是新增的：如果同一类否决连续持续超过 veto_recovery_time_，说明
  // 重规划解决不了（典型情况是车已经贴到障碍上、规划器的起点检查过不去），
  // 此时必须先物理脱困。原先这里只会无限循环「停车 → 重规划 → 起点不可行
  // → 继续停车」，靠 noProgress 兜底要多等 no_progress_timeout。
  //
  // 返回 true 表示已转入恢复态，调用方应立即 return。
  bool handleVeto(
    const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & costmap, const Eigen::Vector2d & pos,
    double dt, const char * reason)
  {
    publishStop();
    veto_streak_ += dt;

    if (!recovery_enabled_ || veto_streak_ < veto_recovery_time_) {
      requestReplan();
      return false;
    }

    // 已经在危险区就直接采样安全点，不必先白倒一段车。
    const bool hazardous = costmap ? mpc::isHazardous(*costmap, pos, hazard_policy_) : false;
    enterRecovery(hazardous ? NavState::HazardRecovery : NavState::StuckReverse, pos, reason);
    return true;
  }

  void failRecovery(const char * reason)
  {
    RCLCPP_ERROR(get_logger(), "Recovery failed, entering FAILED state: %s", reason);
    nav_state_ = NavState::Failed;
    safe_point_.reset();
    setApproachEnabled(true);
  }

  // 恢复期的通行判据只看 lethal：车已在膨胀圈里，用正常阈值会让所有方向
  // 立即被否，恢复无法启动。
  bool reversePathClear(
    const nav_msgs::msg::OccupancyGrid & grid, const Eigen::Vector2d & pos) const
  {
    const double remaining = std::max(recovery_reverse_distance_ - reverse_travelled_, 0.0);
    const int samples = 6;
    std::vector<Eigen::Vector2d> probe;
    probe.reserve(samples + 1);
    for (int i = 0; i <= samples; ++i) {
      const double fraction = static_cast<double>(i) / static_cast<double>(samples);
      probe.push_back(pos + reverse_direction_ * remaining * fraction);
    }

    LocalPathSafetyPolicy lethal_only;
    lethal_only.obstacle_threshold = hazard_policy_.lethal_cost;
    lethal_only.unknown_is_obstacle = hazard_policy_.unknown_is_hazard;
    lethal_only.check_steps = 0;  // 检查全部探针点。
    return isPathSafe(grid, probe, lethal_only);
  }

  void runStuckReverse(
    const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & costmap, const Eigen::Vector2d & pos,
    double yaw)
  {
    // 没有代价图时无法评估倒车路径是否安全。旧逻辑里 hazardous 取 true（按最坏
    // 情况处理），但 reversePathClear 被 `costmap &&` 短路跳过，最危险的“无图”
    // 场景反而会继续下发倒车速度。这里统一为：无图绝不盲倒车，立即停止并判失败，
    // 由重规划在代价图恢复后重新决策。
    if (!costmap) {
      publishStop();
      failRecovery("no costmap during reverse");
      return;
    }

    reverse_travelled_ += (pos - reverse_last_pos_).norm();
    reverse_last_pos_ = pos;

    const bool hazardous = mpc::isHazardous(*costmap, pos, hazard_policy_);

    if (reverse_travelled_ >= recovery_reverse_distance_) {
      if (!hazardous) {
        exitRecovery("reversed clear of obstacle");
      } else {
        // 倒退够了但仍在危险区，升级为采样安全点。
        enterRecovery(NavState::HazardRecovery, pos, "still hazardous after reversing");
      }
      return;
    }

    if (!reversePathClear(*costmap, pos)) {
      publishStop();
      enterRecovery(NavState::HazardRecovery, pos, "reverse path blocked by lethal obstacle");
      return;
    }

    publishWorldVelocity(reverse_direction_ * recovery_reverse_speed_, yaw);
  }

  void runHazardRecovery(
    const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & costmap, const Eigen::Vector2d & pos,
    double yaw, double dt)
  {
    if (!costmap) {
      publishStop();
      return;
    }

    if (!safe_point_) {
      safe_point_ = mpc::findSafePoint(*costmap, pos, hazard_policy_, safe_point_params_);
      if (!safe_point_) {
        publishStop();
        failRecovery("no reachable safe point around robot");
        return;
      }
    }

    const Eigen::Vector2d to_target = *safe_point_ - pos;
    const double distance = to_target.norm();

    if (distance > recovery_reach_tolerance_) {
      const double speed = std::min(recovery_kp_ * distance, recovery_max_speed_);
      publishWorldVelocity((to_target / distance) * speed, yaw);
      recovery_dwell_ = 0.0;
      return;
    }

    // 到点了：停下确认安全并保持一段时间，避免立刻又被判危险。
    publishStop();
    if (mpc::isHazardous(*costmap, pos, hazard_policy_)) {
      // 这个点其实不安全，换一个。
      safe_point_.reset();
      recovery_dwell_ = 0.0;
      return;
    }

    recovery_dwell_ += dt;
    if (recovery_dwell_ >= recovery_dwell_time_) {
      exitRecovery("reached and held safe point");
    }
  }

  bool costmapFresh(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & costmap) const
  {
    if (!costmap) {
      return false;
    }
    const rclcpp::Time stamp(costmap->header.stamp, get_clock()->get_clock_type());
    const double age = (now() - stamp).seconds();
    return std::isfinite(age) && age >= 0.0 && age <= std::max(0.0, local_costmap_timeout_);
  }

  void requestReplan()
  {
    const auto current = now();
    if (last_replan_request_ &&
      (current - *last_replan_request_).seconds() < std::max(0.0, replan_cooldown_))
    {
      return;
    }
    replan_pub_->publish(std_msgs::msg::Empty{});
    last_replan_request_ = current;
  }

  // 绕过冷却的重规划。恢复链退出是一次性事件：旧路径的执行生命周期已被
  // 打断，必须立刻从新位置重建路径，不能被冷却吞掉。
  void forceReplan()
  {
    replan_pub_->publish(std_msgs::msg::Empty{});
    last_replan_request_ = now();
  }

  // 发布当前位置和速度箭头。
  void publishMarkers(const Eigen::Vector2d & pos, double vx_w, double vy_w)
  {
    // 当前位置球体。
    visualization_msgs::msg::Marker sm;
    sm.header.frame_id = target_frame_;
    sm.header.stamp = now();
    sm.ns = "position";
    sm.id = 1;
    sm.type = visualization_msgs::msg::Marker::SPHERE;
    sm.action = visualization_msgs::msg::Marker::ADD;
    sm.scale.x = sm.scale.y = sm.scale.z = 0.3;
    sm.color.a = 1.0;
    sm.color.g = 1.0;
    sm.pose.position.x = pos.x();
    sm.pose.position.y = pos.y();
    sm.lifetime = rclcpp::Duration::from_seconds(0.1);
    state_marker_pub_->publish(sm);

    // 速度箭头。
    visualization_msgs::msg::Marker vm;
    vm.header.frame_id = target_frame_;
    vm.header.stamp = now();
    vm.ns = "mpc_velocity";
    vm.id = 0;
    vm.type = visualization_msgs::msg::Marker::ARROW;
    vm.action = visualization_msgs::msg::Marker::ADD;
    vm.scale.x = 0.08;  // 箭杆
    vm.scale.y = 0.16;  // 箭头宽
    vm.scale.z = 0.24;  // 箭头长
    vm.color.r = 0.9f;
    vm.color.g = 0.1f;
    vm.color.b = 0.1f;
    vm.color.a = 1.0f;
    vm.lifetime = rclcpp::Duration::from_seconds(0.1);
    geometry_msgs::msg::Point ps, pe;
    ps.x = pos.x();
    ps.y = pos.y();
    const double scale = std::hypot(vx_w, vy_w);
    if (scale > 1e-3) {
      pe.x = pos.x() + vx_w;
      pe.y = pos.y() + vy_w;
    } else {
      pe.x = pos.x();
      pe.y = pos.y();
    }
    vm.points.push_back(ps);
    vm.points.push_back(pe);
    vel_marker_pub_->publish(vm);
  }

  // 滚动前推速度序列。
  void publishPredict(const std::vector<Eigen::Vector2d> & vseq, const Eigen::Vector2d & pos)
  {
    if (predict_path_pub_->get_subscription_count() == 0) {
      return;
    }
    nav_msgs::msg::Path path;
    path.header.frame_id = target_frame_;
    path.header.stamp = now();
    Eigen::Vector2d p = pos;
    for (const auto & v : vseq) {
      p += v * predict_dt_;
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = p.x();
      ps.pose.position.y = p.y();
      const double yaw = (v.norm() > 1e-3) ? std::atan2(v.y(), v.x()) : 0.0;
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      ps.pose.orientation = tf2::toMsg(q);
      path.poses.push_back(ps);
    }
    predict_path_pub_->publish(path);
  }

  std::string path_topic_, odom_topic_, cmd_vel_topic_, target_frame_;
  std::string local_costmap_topic_, replan_topic_, robot_base_frame_;
  double control_fps_, expected_speed_, goal_tolerance_, max_track_error_;
  double blind_radius_, delay_time_, default_wz_, predict_dt_;
  bool approach_enable_{true};
  double approach_distance_{1.5};
  double approach_velocity_{0.5};
  double direct_approach_distance_{0.5};
  double direct_approach_kp_{1.0};
  std::atomic<bool> approach_enabled_{true};
  int steps_;
  bool use_delay_comp_;
  bool use_tf_pose_{true};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  double delay_comp_max_dt_;
  double local_costmap_timeout_, replan_cooldown_;
  LocalPathSafetyPolicy safety_policy_;

  // 隧道限速窗口。语义地图在订阅回调里写、在 rebuildSpeedProfile 的窗口查询里读，
  // 两者可能不同线程（单容器多线程执行器），用独立的 map_mutex_ 护住 receiver_ ——
  // 与控制状态的 mtx_ 分开，避免把地图订阅和控制环相互阻塞。
  bool tunnel_speed_window_enabled_{true};
  std::string semantic_map_topic_;
  SemanticMapReceiver receiver_;
  std::mutex map_mutex_;

  std::mutex mtx_;
  nav_msgs::msg::Odometry odom_;
  mpc::PathReference ref_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr local_costmap_;
  bool has_path_ = false;
  bool has_odom_ = false;
  bool path_changed_ = false;
  // 上一条被接受的路径几何，用来区分「真的换了路径」和「原样重发」。
  std::vector<Eigen::Vector2d> last_path_points_;
  // 目标点（路径终点）变化标记：FAILED 态只有收到新目标才清除。MPC 不订阅
  // /goal_pose，只能从路径终点推断目标。
  bool goal_changed_ = false;
  std::optional<Eigen::Vector2d> last_goal_;
  // 链路末端实际下发的速率，由 executed_cmd_sub_ 在订阅线程写入。这是
  // 「底盘真的收到了什么」的唯一可信来源：本节点发布的指令仍会被
  // rm_velocity_smoother 限幅或超时归零。
  double executed_speed_{0.0};
  std::optional<rclcpp::Time> last_executed_cmd_time_;
  // 云台收放的请求与实测，由各自的订阅回调写入。请求到位之前车不走 —— 判据见 control()。
  bool gimbal_lower_requested_{false};
  // 初值 false 是拦车判据的安全默认值：还没收到回传时按「没收好」处理。见 control()。
  bool gimbal_lowered_{false};
  // 只用来分辨停车日志该说「云台还在动」还是「回传压根没来」。不参与拦车判据 ——
  // 加进判据是多余的，因为 gimbal_lowered_ 的初值已经覆盖了「没收到过」这种情况。
  bool has_gimbal_state_{false};

  mpc::MpcSolver solver_;
  // 进度跟踪与失效检测只在控制线程里用，不需要加锁。
  mpc::RouteTracker route_tracker_;
  mpc::ProgressMonitor progress_monitor_;
  // 速度剖面：按曲率限侧向加速度，前/后向扫描保证加减速可达。
  // 路径变化时由 control() 调用 rebuild()；启用开关由 speed_profile.enable 控制。
  mpc::SpeedProfile speed_profile_;
  bool speed_profile_enabled_{true};

  // 恢复链 FSM。只在控制线程里读写，不需要加锁。
  NavState nav_state_{NavState::Follow};
  bool recovery_enabled_{true};
  mpc::HazardPolicy hazard_policy_;
  mpc::SafePointSearchParams safe_point_params_;
  // 指令链路可观测性。只在控制线程里读写。
  std::string executed_cmd_topic_;
  std::string gimbal_posture_topic_;
  std::string gimbal_posture_state_topic_;
  double executed_cmd_timeout_{0.3};
  double override_detect_time_{1.0};
  // 「本节点发速度但链路末端为零」的累计时长，超过 override_detect_time_ 报警。
  double override_time_{0.0};
  // 是否收到过任何 executed 指令。一直没有说明下游节点没起来。
  bool ever_received_executed_{false};
  double no_feedback_time_{0.0};
  bool feedback_warned_{false};
  // 三条失效路径（跟踪丢失/求解失败/安全否决）的连续否决时长。
  double veto_streak_{0.0};
  double veto_recovery_time_{1.5};
  double veto_before_recovery_{1.5};

  double recovery_reverse_speed_{0.3};
  double recovery_reverse_distance_{0.4};
  double recovery_max_speed_{0.4};
  double recovery_kp_{1.0};
  double recovery_reach_tolerance_{0.12};
  double recovery_dwell_time_{0.4};
  double recovery_max_duration_{10.0};
  int recovery_max_attempts_{3};
  double goal_change_threshold_{0.3};
  // 目标附近抑制失效检测的半径，到点静止不应进恢复链。
  double recovery_suppress_near_goal_{0.35};
  // 本次恢复动作的运行状态。
  Eigen::Vector2d reverse_direction_{Eigen::Vector2d::Zero()};
  double reverse_travelled_{0.0};
  Eigen::Vector2d reverse_last_pos_{Eigen::Vector2d::Zero()};
  std::optional<Eigen::Vector2d> safe_point_;
  double recovery_dwell_{0.0};
  double recovery_elapsed_{0.0};
  int recovery_attempts_{0};
  double last_published_speed_{0.0};
  std::optional<rclcpp::Time> last_control_time_;
  // 预分配缓冲区，避免 30Hz 控制循环中重复分配。
  std::vector<Eigen::Vector2d> predicted_positions_buffer_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr executed_cmd_sub_;
  rclcpp::Subscription<decision_interfaces::msg::GimbalPosture>::SharedPtr gimbal_posture_sub_;
  rclcpp::Subscription<decision_interfaces::msg::GimbalPostureState>::SharedPtr
    gimbal_posture_state_sub_;
  rclcpp::Subscription<decision_interfaces::msg::SemanticMap>::SharedPtr semantic_map_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_norm_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr predict_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vel_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr state_marker_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr replan_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr approach_enabled_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr approach_enabled_sub_;
  std::string approach_enabled_topic_;
  std::optional<rclcpp::Time> last_replan_request_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace navigation2

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(navigation2::RmMpcController)

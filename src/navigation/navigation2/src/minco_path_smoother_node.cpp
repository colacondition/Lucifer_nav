#include "grid_utils.hpp"
#include "minco/minco_optimizer.hpp"
#include "performance_monitor.hpp"
#include "semantic_map_consumer.hpp"
#include "shared_state.hpp"

#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

#include <decision_interfaces/msg/semantic_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

namespace navigation2
{

class RmMincoPathSmoother : public rclcpp::Node
{
public:
  explicit RmMincoPathSmoother(const rclcpp::NodeOptions & options)
  : Node("rm_minco_path_smoother", options)
  {
    input_path_topic_ = declare_parameter<std::string>("input_path_topic", "/plan_raw");
    output_path_topic_ = declare_parameter<std::string>("output_path_topic", "/plan");

    // MINCO 参数。几何平滑 + 数据保持 + 隧道轴向对齐是既有职责；障碍 soft 代价
    // 现在从进程内距离场（DistanceFieldRegistry，由 rm_local_costmap 维护）注入，
    // 替代历史版本误把机器人多边形 SDF 当障碍距离场的实现 —— 那版距离场中心固定在
    // map 原点、查询几乎恒 0，障碍代价从不生效，已删。距离场未就绪时障碍项静默失效。
    smooth_weight_ = declare_parameter<double>("smooth_weight", 1.0);
    data_weight_ = declare_parameter<double>("data_weight", 10.0);
    enable_optimization_ = declare_parameter<bool>("enable_optimization", true);
    // 隧道内偏离轴线的软代价权重。收不到语义地图或图里没隧道时自动失效。
    tunnel_axis_weight_ = declare_parameter<double>("tunnel_axis_weight", 5.0);
    // 障碍 soft 代价权重与安全距离。设 0 关闭；距离场未就绪时静默失效。
    obstacle_weight_ = declare_parameter<double>("obstacle_weight", 0.0);
    safe_dist_ = declare_parameter<double>("safe_dist", 0.2);
    semantic_map_topic_ = declare_parameter<std::string>(
      "semantic_map_topic", "/map_server/semantic_map");

    // 时间分配参数
    default_velocity_ = declare_parameter<double>("default_velocity", 1.0);
    min_segment_time_ = declare_parameter<double>("min_segment_time", 0.1);

    // 关断式性能观测：默认关。开 enable 后记录优化耗时 / 成功 / 失败原因到 CSV。
    {
      navigation2::PerformanceMonitor::Config cfg;
      cfg.enable = declare_parameter<bool>("performance.enable", false);
      cfg.csv_enable = declare_parameter<bool>("performance.csv_enable", false);
      cfg.csv_path = declare_parameter<std::string>(
        "performance.csv_path", "/tmp/minco_perf.csv");
      cfg.print_enable = declare_parameter<bool>("performance.print_enable", false);
      perf_monitor_.configure(cfg, get_logger());
    }

    // 初始化 MINCO 优化器
    minco_optimizer_ = std::make_unique<MincoOptimizer>();
    MincoOptimizer::Params params;
    params.smooth_weight = smooth_weight_;
    params.data_weight = data_weight_;
    params.tunnel_axis_weight = tunnel_axis_weight_;
    params.obstacle_weight = obstacle_weight_;
    params.safe_dist = safe_dist_;
    params.enable = enable_optimization_;
    minco_optimizer_->setParams(params);

    // 隧道轴线查询：给 MINCO 的对齐软代价用。语义地图收不到时 receiver_.map() 无效，
    // tunnelAxisAtPoint 恒返回 false，对齐项自动失效。
    minco_optimizer_->setTunnelAxisQuery(
      [this](const Eigen::Vector2d & pos, Eigen::Vector2d & axis) {
        std::lock_guard<std::mutex> lk(map_mutex_);
        return tunnelAxisAtPoint(receiver_.map(), pos, axis);
      });

    // 语义地图和代价地图同源同 QoS。收不到时对齐项静默失效（隧道被当普通空地），
    // 不报错。整帧不自洽时保留上一张好图。
    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
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

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      input_path_topic_, rclcpp::QoS(1).reliable(),
      [this](nav_msgs::msg::Path::ConstSharedPtr msg) {
        smoothAndPublish(*msg);
      });

    path_pub_ = create_publisher<nav_msgs::msg::Path>(output_path_topic_, rclcpp::QoS(1).reliable());

    RCLCPP_INFO(
      get_logger(), "rm_minco_path_smoother ready: %s -> %s",
      input_path_topic_.c_str(), output_path_topic_.c_str());
  }

private:
  void smoothAndPublish(const nav_msgs::msg::Path & input_path)
  {
    const auto perf_start = perf_monitor_.tick();
    if (input_path.poses.size() < 2) {
      RCLCPP_WARN(get_logger(), "Path too short for MINCO smoothing");
      navigation2::TrajectoryCache::instance().invalidate();
      path_pub_->publish(input_path);
      return;
    }

    // 提取路径点
    std::vector<Eigen::Vector2d> waypoints;
    waypoints.reserve(input_path.poses.size());
    for (const auto & pose : input_path.poses) {
      waypoints.emplace_back(pose.pose.position.x, pose.pose.position.y);
    }

    // 计算段时间（基于路径点间距离）
    std::vector<double> segment_times;
    segment_times.reserve(waypoints.size() - 1);
    for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
      double dist = (waypoints[i + 1] - waypoints[i]).norm();
      double time = std::max(dist / default_velocity_, min_segment_time_);
      segment_times.push_back(time);
    }

    // 障碍代价距离场：优化前读一次快照（shared_ptr 只读，跨节点 registry 有锁），
    // 优化回调里反复查同一快照，不在 L-BFGS 热循环里每点加锁。未就绪时障碍项失效。
    auto dist_field = navigation2::DistanceFieldRegistry::instance().latest();
    minco_optimizer_->setDistanceQuery(
      [dist_field](const Eigen::Vector2d & pos, double & d, Eigen::Vector2d & g) {
        if (!dist_field) {
          return false;
        }
        return navigation2::DistanceFieldRegistry::query(*dist_field, pos, d, g);
      });

    // MINCO 优化
    auto start_time = now();
    auto pieces = minco_optimizer_->optimize(waypoints, segment_times);
    auto duration = (now() - start_time).seconds();

    if (pieces.empty()) {
      RCLCPP_ERROR(get_logger(), "MINCO optimization failed, publishing original path");
      navigation2::PerformanceMonitor::Sample sample;
      sample.success = false;
      sample.failure_reason = "optimize_failed";
      perf_monitor_.stop(perf_start, sample);
      navigation2::TrajectoryCache::instance().invalidate();
      path_pub_->publish(input_path);
      return;
    }

    RCLCPP_DEBUG(get_logger(), "MINCO optimization took %.3f ms", duration * 1000.0);

    // 采样轨迹：一条 Path（给 RViz / rm_tunnel_posture 的兼容通道）+ 一份轨迹快照
    // （给 MPC 的 P/V/A 前馈）。MINCO 已经算出 P/V/A，旧实现只采位置就发布 Path，
    // 速度/加速度白算一遍；现在按全局时间等距采样，同时喂两条通道。
    nav_msgs::msg::Path output_path;
    output_path.header = input_path.header;

    const double sample_dt = 0.05;  // 每 0.05 秒采样一次，给 MPC 更平滑的参考
    double total_duration = 0.0;
    for (const auto & piece : pieces) {
      total_duration += piece.getDuration();
    }

    navigation2::TrajectorySnapshot snapshot;
    snapshot.dt = sample_dt;
    snapshot.total_duration = total_duration;
    snapshot.stamp = now();
    snapshot.valid = true;
    snapshot.samples.reserve(static_cast<size_t>(total_duration / sample_dt) + 4);

    double s = 0.0;
    double global_t = 0.0;
    double prev_x = 0.0;
    double prev_y = 0.0;
    bool have_prev = false;
    const double eps = 1e-9;

    for (const auto & piece : pieces) {
      const double dur = piece.getDuration();
      const int num = std::max(1, static_cast<int>(std::ceil(dur / sample_dt - eps)));
      for (int j = 0; j <= num; ++j) {
        const double t_local = std::min(j * sample_dt, dur);
        // 段首与上一段段尾是同一时间点，跳过重复采样。
        if (j == 0 && !snapshot.samples.empty()) {
          continue;
        }
        const Eigen::Vector2d pos = piece.getPos(t_local);
        const Eigen::Vector2d vel = piece.getVel(t_local);
        const Eigen::Vector2d acc = piece.getAcc(t_local);
        if (have_prev) {
          s += std::hypot(pos.x() - prev_x, pos.y() - prev_y);
        }
        prev_x = pos.x();
        prev_y = pos.y();
        have_prev = true;

        geometry_msgs::msg::PoseStamped pose;
        pose.header = input_path.header;
        pose.pose.position.x = pos.x();
        pose.pose.position.y = pos.y();
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;
        output_path.poses.push_back(pose);

        navigation2::TrajectorySample smp;
        smp.t = global_t + t_local;
        smp.s = s;
        smp.x = pos.x();
        smp.y = pos.y();
        smp.vx = vel.x();
        smp.vy = vel.y();
        smp.ax = acc.x();
        smp.ay = acc.y();
        snapshot.samples.push_back(smp);
      }
      global_t += dur;
    }
    snapshot.total_length = s;

    // 发布前硬校验：障碍代价开启且距离场就绪时，逐采样点查距离，任何一点 < safe_dist
    // 即拒绝这条轨迹（发布原始路径），防止 soft 代价没兜住的穿障解流到 MPC（对应
    // sentry 的 TrajectorySafetyChecker 发布前拒绝，但只做 2D 距离检查）。
    if (obstacle_weight_ > 0.0 && dist_field) {
      bool trajectory_safe = true;
      for (const auto & smp : snapshot.samples) {
        double d = 0.0;
        Eigen::Vector2d g;
        if (!navigation2::DistanceFieldRegistry::query(
            *dist_field, Eigen::Vector2d(smp.x, smp.y), d, g))
        {
          continue;  // 地图外不判，由 MPC 安全检查兜底
        }
        if (std::isfinite(d) && d < safe_dist_) {
          trajectory_safe = false;
          break;
        }
      }
      if (!trajectory_safe) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "MINCO trajectory rejected by safety check (distance < %.2f m)", safe_dist_);
        navigation2::PerformanceMonitor::Sample sample;
        sample.success = false;
        sample.failure_reason = "safety_reject";
        perf_monitor_.stop(perf_start, sample);
        navigation2::TrajectoryCache::instance().invalidate();
        path_pub_->publish(input_path);
        return;
      }
    }

    path_pub_->publish(output_path);
    navigation2::TrajectoryCache::instance().publish(std::move(snapshot));
    {
      navigation2::PerformanceMonitor::Sample sample;
      sample.success = true;
      perf_monitor_.stop(perf_start, sample);
    }
    RCLCPP_DEBUG(
      get_logger(), "Published MINCO path with %zu poses and %zu trajectory samples",
      output_path.poses.size(), navigation2::TrajectoryCache::instance().latest() ?
      navigation2::TrajectoryCache::instance().latest()->samples.size() : 0);
  }

  std::string input_path_topic_;
  std::string output_path_topic_;

  double smooth_weight_;
  double data_weight_;
  double tunnel_axis_weight_;
  double obstacle_weight_;
  double safe_dist_;
  bool enable_optimization_;
  std::string semantic_map_topic_;

  double default_velocity_;
  double min_segment_time_;

  std::unique_ptr<MincoOptimizer> minco_optimizer_;

  // 关断式性能观测。smoothAndPublish 与优化回调同线程（MutuallyExclusive 回调组），
  // 无需加锁；record 只在 enable 时计时。
  navigation2::PerformanceMonitor perf_monitor_;

  // 语义地图。轴线查询在优化回调里读，语义地图在订阅回调里写，两者可能不同线程
  // （单容器多线程执行器），用锁护住 receiver_。
  SemanticMapReceiver receiver_;
  std::mutex map_mutex_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<decision_interfaces::msg::SemanticMap>::SharedPtr semantic_map_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

}  // namespace navigation2

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(navigation2::RmMincoPathSmoother)

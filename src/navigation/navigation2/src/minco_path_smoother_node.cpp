#include "grid_utils.hpp"
#include "minco/minco_optimizer.hpp"
#include "minco_time_allocation.hpp"
#include "performance_monitor.hpp"
#include "sfc_corridor.hpp"
#include "semantic_map_consumer.hpp"
#include "shared_state.hpp"

#include <algorithm>
#include <chrono>
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
    // 隧道横向走廊软代价权重与影响区边距。借鉴 TDT-nav-kit 的 Item.corridor：
    // 路标点横向偏移超出 clear_width/2 - robot_radius 时加 w * 超出量²。这是几何
    // 边界而不是方向项 —— 轴向项管「走向正不正」，走廊项管「别贴壁」。影响区
    // 边距与 rm_global_costmap / rm_local_costmap 的 tunnel_margin_m 同一取值，
    // 三处必须一起改。设 0 关闭。
    tunnel_corridor_weight_ = declare_parameter<double>("tunnel_corridor_weight", 100.0);
    tunnel_corridor_margin_m_ = declare_parameter<double>("tunnel_corridor.margin_m", 0.20);
    tunnel_corridor_lateral_margin_m_ =
      declare_parameter<double>("tunnel_corridor.lateral_margin_m", 0.0);
    // 障碍 soft 代价权重与安全距离。设 0 关闭；距离场未就绪时静默失效。
    obstacle_weight_ = declare_parameter<double>("obstacle_weight", 0.0);
    safe_dist_ = declare_parameter<double>("safe_dist", 0.2);
    semantic_map_topic_ = declare_parameter<std::string>(
      "semantic_map_topic", "/map_server/semantic_map");

    // —— 全图级安全校验（借鉴 TDT-nav-kit 的 SFC 走廊）——
    // MINCO 现有的发布前校验只覆盖 rm_local_costmap 的 5x5 m 滚动窗口，5 m 以外
    // 的全局障碍对它不可见。这里从全局代价地图建 SfcCorridor，用「路标点为中心
    // 的无致命格方形半宽 >= robot_radius」做全图校验，两者互补。
    // global_check.enable=false 或收不到代价地图时退回局部校验，不报错。
    global_check_enable_ = declare_parameter<bool>("global_check.enable", true);
    global_check_robot_radius_ = declare_parameter<double>("global_check.robot_radius", 0.25);
    global_check_max_range_ = declare_parameter<double>("global_check.max_range", 2.5);
    global_check_topic_ = declare_parameter<std::string>(
      "global_check.costmap_topic", "/global_costmap/costmap");
    global_check_threshold_ = declare_parameter<int>("global_check.obstacle_threshold", 100);
    global_check_unknown_lethal_ = declare_parameter<bool>("global_check.unknown_is_lethal", false);

    // —— 有界迭代修复（借鉴 TDT-nav-kit 的碰撞迭代）——
    // 校验失败不再立刻整条拒绝回退折线，而是先插路标点/收紧约束重优化，预算用尽
    // 才回退。repair.max_attempts=0 完全关闭（回到历史行为）。
    repair_max_attempts_ = static_cast<int>(std::clamp<int64_t>(
      declare_parameter<int>("repair.max_attempts", 2), 0, 4));
    repair_waypoint_spacing_ = declare_parameter<double>("repair.waypoint_spacing", 0.15);

    // 固定时间分配参数。保持时间不进入 L-BFGS，避免扩大优化变量；在距离/速度基础上
    // 根据转角给相邻段增加过渡时间，给急弯更合理的动力学初值。
    default_velocity_ = declare_parameter<double>("default_velocity", 1.0);
    min_segment_time_ = declare_parameter<double>("min_segment_time", 0.1);
    turn_time_weight_ = declare_parameter<double>("turn_time_weight", 0.0);
    first_stage_max_iterations_ = static_cast<int>(std::max<int64_t>(
      1, declare_parameter<int>("first_stage.max_iterations", 800)));
    lbfgs_memory_size_ = static_cast<int>(std::clamp<int64_t>(
      declare_parameter<int>("lbfgs.memory_size", 32), 4, 128));
    two_stage_enable_ = declare_parameter<bool>("two_stage.enable", true);
    two_stage_max_speed_ = declare_parameter<double>("two_stage.max_speed", default_velocity_);
    two_stage_max_accel_ = declare_parameter<double>("two_stage.max_accel", 4.0);
    two_stage_max_scale_ = declare_parameter<double>("two_stage.max_scale", 1.5);
    two_stage_samples_per_piece_ = static_cast<int>(std::max<int64_t>(
      3, declare_parameter<int>("two_stage.samples_per_piece", 5)));
    two_stage_max_iterations_ = static_cast<int>(std::max<int64_t>(
      1, declare_parameter<int>("two_stage.max_iterations", 300)));
    // 第二阶段的 v/a 极值检测方式。true = 多项式求根（精确，默认）；
    // false = 保留旧的每段等距采样（性能敏感时回退，会漏掉段内极值）。
    two_stage_exact_check_ = declare_parameter<bool>("two_stage.exact_dynamics_check", true);

    // 轨迹快照/兼容 Path 的等时采样步长。MPC 用它做前馈查表，太小徒增消息
    // 尺寸、太大丢弯道细节；非正数或非有限值回退默认，避免除零。
    trajectory_sample_dt_ = declare_parameter<double>("trajectory_sample_dt", 0.05);
    if (!std::isfinite(trajectory_sample_dt_) || trajectory_sample_dt_ <= 0.0) {
      RCLCPP_WARN(
        get_logger(), "Invalid trajectory_sample_dt %.4f s, falling back to 0.05",
        trajectory_sample_dt_);
      trajectory_sample_dt_ = 0.05;
    }

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
    params.obstacle_normal_only = false;
    params.lbfgs_memory_size = lbfgs_memory_size_;
    params.max_iterations = first_stage_max_iterations_;
    params.enable = enable_optimization_;
    minco_optimizer_->setParams(params);

    // 隧道轴线查询：给 MINCO 的对齐软代价用。语义地图收不到时 receiver_.map() 无效，
    // tunnelAxisAtPoint 恒返回 false，对齐项自动失效。
    minco_optimizer_->setTunnelAxisQuery(
      [this](const Eigen::Vector2d & pos, Eigen::Vector2d & axis) {
        std::lock_guard<std::mutex> lk(map_mutex_);
        return tunnelAxisAtPoint(receiver_.map(), pos, axis);
      });

    // 隧道横向走廊查询：把 TunnelRegionGrid 的走廊几何桥接给 MINCO 的走廊项。
    // TunnelRegionGrid 与 receiver_ 一起在 map_mutex_ 下更新，查询同一把锁 ——
    // 平滑回调和语义地图回调同在 MutuallyExclusive 回调组，实际串行，显式加锁
    // 是为了让「读方拿的是同一张图的派生物」这一约束显式可见。
    minco_optimizer_->setTunnelCorridorQuery(
      [this](const Eigen::Vector2d & pos, TunnelCorridorFrame & frame) {
        std::lock_guard<std::mutex> lk(map_mutex_);
        TunnelRegionGrid::CorridorFrame geo;
        if (!tunnel_region_ ||
          !tunnel_region_->corridorFrameAtPoint(
            pos.x(), pos.y(), global_check_robot_radius_, tunnel_corridor_lateral_margin_m_, geo))
        {
          return false;
        }
        frame.centroid = geo.centroid;
        frame.dir = geo.dir;
        frame.half_len = geo.half_len;
        frame.half_width_inner = geo.half_width_inner;
        frame.lateral_outer = geo.lateral_outer;
        return true;
      });

    // 语义地图和代价地图同源同 QoS。收不到时对齐项静默失效（隧道被当普通空地），
    // 不报错。整帧不自洽时保留上一张好图。
    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    semantic_map_sub_ = create_subscription<decision_interfaces::msg::SemanticMap>(
      semantic_map_topic_, map_qos,
      [this](decision_interfaces::msg::SemanticMap::ConstSharedPtr msg) {
        std::unique_lock<std::mutex> lk(map_mutex_);
        try {
          if (receiver_.update(*msg)) {
            // 只有内容真的变了才重建派生物：rm_map_server 每秒重发同一张图，
            // TunnelRegionGrid 的 build 是逐本体格半径展开，重复做就是白烧 CPU。
            tunnel_region_ = std::make_unique<TunnelRegionGrid>(
              TunnelRegionGrid::build(receiver_.map(), tunnel_corridor_margin_m_));
            ++semantic_generation_;
          }
        } catch (const std::exception & ex) {
          RCLCPP_ERROR(get_logger(), "Rejected semantic map: %s", ex.what());
        }
      });

    // 全局代价地图：全图级安全校验的输入。与 rm_global_costmap 的发布 QoS 一致
    // （transient_local + reliable），迟加入也能拿到最后一张图。
    if (global_check_enable_) {
      global_costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        global_check_topic_, map_qos,
        [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
          // 全局代价地图 8 Hz 发布且几乎每帧变内容；SfcCorridor 内部有内容哈希，
          // 内容没变时 O(1) 返回，变了才重建积分图（O(n)，10 万格亚毫秒）。
          // 致命位图构建是 O(n) 的，但那部分本来就无法避免 —— 与其在这里加
          // 二级哈希，不如让 SfcCorridor 自己决定是否重算。
          SfcCorridorParams p;
          p.max_range = global_check_max_range_;
          p.robot_radius = global_check_robot_radius_;
          p.obstacle_threshold = global_check_threshold_;
          p.unknown_is_lethal = global_check_unknown_lethal_;
          std::lock_guard<std::mutex> lk(map_mutex_);
          sfc_corridor_.updateGrid(*msg, p);
        });
    }

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
  // 输入路径的几何指纹。用于 0b 的去重：全局规划器以 5-8 Hz 定频重发同一条
  // /plan_raw（路径没变时也重发），而 MINCO 优化是平滑器里最贵的一步
  // （10 m 路径约 35 ms）。指纹相同就复用上一次的输出，不再重跑 L-BFGS。
  //
  // 指纹除了路径本身还要带上语义地图代次：隧道路标点约束 / 轴向对齐项都读语义
  // 地图，地图更新后同一条路径的优化结果会变。障碍距离场不进指纹 —— 它 10 Hz
  // 连续变化，进了指纹去重就永远命中不了；而规划器本身消费同一张实时代价地图，
  // 它给出的路径没变就说明障碍格局没有发生会改变优化结果的变化，且 MPC 侧仍有
  // local_safety 兜底。
  struct InputFingerprint
  {
    bool valid = false;
    std::uint64_t path_hash = 0;
    std::uint64_t semantic_generation = 0;
    bool operator==(const InputFingerprint & other) const noexcept
    {
      return valid == other.valid && path_hash == other.path_hash &&
             semantic_generation == other.semantic_generation;
    }
  };

  static std::uint64_t hashPathGeometry(const nav_msgs::msg::Path & path)
  {
    // FNV-1a，与 global_planner_node 的 clearanceFieldHash 同一套约定。
    std::uint64_t hash = 1469598103934665603ULL;
    auto mix = [&hash](const void * data, std::size_t bytes) {
        const auto * bytes_ptr = static_cast<const unsigned char *>(data);
        for (std::size_t i = 0; i < bytes; ++i) {
          hash ^= bytes_ptr[i];
          hash *= 1099511628211ULL;
        }
      };
    const std::uint64_t count = path.poses.size();
    mix(&count, sizeof(count));
    for (const auto & pose : path.poses) {
      // 只比位置几何；header 时间戳每次必然不同，不能参与指纹。
      const double xy[2] = {pose.pose.position.x, pose.pose.position.y};
      mix(xy, sizeof(xy));
    }
    return hash;
  }

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

    // —— 去重 —— 语义地图读取放在锁内拿一次代次快照，锁只在读取期间持有，
    // 不会覆盖优化回调里轴线查询的加锁窗口（两者同在 MutuallyExclusive 回调组，
    // 实际串行，这里显式取快照是为了让锁的意图可读）。
    InputFingerprint fingerprint;
    {
      std::lock_guard<std::mutex> lk(map_mutex_);
      fingerprint.semantic_generation = semantic_generation_;
    }
    fingerprint.path_hash = hashPathGeometry(input_path);
    fingerprint.valid = true;
    if (fingerprint == last_fingerprint_ && last_output_valid_) {
      // 同一条路径 + 同一张语义地图：直接重发上一次的输出。快照不动 —— 它描述的
      // 就是这条路径，invalidate() 反而会让 MPC 在下一拍丢掉前馈。
      // 只重发 Path 不重发快照是有意的：快照是进程内单例的 latest()，本就还在。
      if (perf_monitor_.enabled()) {
        navigation2::PerformanceMonitor::Sample sample;
        sample.success = true;
        sample.failure_reason = "dedup_reuse";
        perf_monitor_.stop(perf_start, sample);
      }
      path_pub_->publish(last_output_path_);
      return;
    }

    // 固定段时间：距离项保证直线速度尺度，转角项给急弯两侧增加过渡时间。该计算为
    // O(N)，不新增优化变量、线程或逐周期缓存，保持现有 MINCO 热路径的规模。
    std::vector<double> segment_times = allocateMincoSegmentTimes(
      waypoints, default_velocity_, min_segment_time_, turn_time_weight_);

    // 障碍代价距离场：优化前读一次快照（shared_ptr 只读，跨节点 registry 有锁），
    // 优化回调里反复查同一快照，不在 L-BFGS 热循环里每点加锁。未就绪时障碍项失效。
    // 同一份快照供优化与发布前校验共用：修复过程不该因为距离场中途更新而改变判据。
    dist_field_for_check_ = navigation2::DistanceFieldRegistry::instance().latest();
    minco_optimizer_->setDistanceQuery(
      [snap = dist_field_for_check_](const Eigen::Vector2d & pos, double & d, Eigen::Vector2d & g) {
        if (!snap) {
          return false;
        }
        return navigation2::DistanceFieldRegistry::queryQuadratic(*snap, pos, d, g);
      });

    RCLCPP_DEBUG(get_logger(), "MINCO optimization start (%zu waypoints)", waypoints.size());

    // —— 优化 + 校验 + 有界修复 ——
    // 历史行为：优化一次，校验失败整条拒绝回退折线。借鉴 TDT-nav-kit 的碰撞迭代
    // （solve → 采样 → 查碰撞 → 修复 → 重解），失败先修复再重解，预算用尽才回退。
    // 修复手段交替使用 TDT 的两种方式：
    //   弱约束（第 1 次）：放大 data_weight，压缩优化器偏离前端路径的自由度；
    //   强约束（第 2 次）：在最差采样点附近插入路标点，给 L-BFGS 新的自由度。
    // repair.max_attempts=0 时 total_attempts==1，完全等价历史行为。
    std::vector<Eigen::Vector2d> repair_waypoints = waypoints;
    std::vector<double> repair_times = segment_times;
    double repair_data_weight = data_weight_;
    double duration = 0.0;
    std::vector<Piece<5, 2>> pieces;
    nav_msgs::msg::Path output_path;
    navigation2::TrajectorySnapshot snapshot;
    bool accepted = false;
    std::string reject_reason;

    const int total_attempts = 1 + std::max(0, repair_max_attempts_);
    for (int attempt = 0; attempt < total_attempts; ++attempt) {
      if (attempt > 0) {
        if ((attempt % 2) == 1) {
          // 弱约束：前端路径是 A* 在同一张代价地图上搜出来的，本身满足 clearance
          // 偏好；把数据项放大就是逼轨迹回到前端路径上。
          repair_data_weight *= 4.0;
          RCLCPP_WARN(
            get_logger(), "MINCO repair attempt %d: data_weight -> %.1f", attempt,
            repair_data_weight);
        } else {
          // 强约束：在最差采样点附近插一个路标点，让 L-BFGS 在那里多一个自由度。
          if (!insertRepairWaypoint(
              repair_waypoints, repair_times, accepted_samples_, repair_waypoint_spacing_))
          {
            RCLCPP_WARN(get_logger(), "MINCO repair: cannot insert waypoint, giving up");
            break;
          }
          RCLCPP_WARN(
            get_logger(), "MINCO repair attempt %d: waypoints -> %zu", attempt,
            repair_waypoints.size());
        }
      }
      accepted_samples_.clear();

      pieces = runOptimization(repair_waypoints, repair_times, repair_data_weight, duration);
      if (pieces.empty()) {
        reject_reason = "optimize_failed";
        break;
      }

      buildTrajectoryOutput(pieces, input_path, output_path, snapshot);
      accepted_samples_ = snapshotSamplePositions(snapshot);

      // 校验。global（全图 SFC）与 local（5x5 m 距离场）互补：前者覆盖全程但
      // 只有致命格，后者只在 5 m 窗口内但带膨胀梯度。任一不可用就跳过该项。
      reject_reason = evaluateTrajectory(snapshot);
      if (reject_reason.empty()) {
        accepted = true;
        break;
      }
      RCLCPP_WARN(
        get_logger(), "MINCO trajectory rejected (%s), attempt %d/%d", reject_reason.c_str(),
        attempt + 1, total_attempts);
    }

    if (!accepted) {
      if (reject_reason.empty()) {
        reject_reason = "optimize_failed";
      }
      RCLCPP_ERROR(
        get_logger(), "MINCO optimization failed (%s), publishing original path",
        reject_reason.c_str());
      navigation2::PerformanceMonitor::Sample sample;
      sample.success = false;
      sample.failure_reason = reject_reason;
      perf_monitor_.stop(perf_start, sample);
      navigation2::TrajectoryCache::instance().invalidate();
      // 失败也记指纹：下一条完全相同的输入同样会失败，直接重发同样的输入路径。
      last_fingerprint_ = fingerprint;
      last_output_path_ = input_path;
      last_output_valid_ = true;
      path_pub_->publish(input_path);
      return;
    }

    RCLCPP_DEBUG(get_logger(), "MINCO optimization took %.3f ms", duration * 1000.0);

    last_fingerprint_ = fingerprint;
    last_output_path_ = output_path;
    last_output_valid_ = true;
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

  // 一次完整的 MINCO 优化（第一阶段 + 确定性第二阶段）。segment_times 会被两阶段
  // 原地放大，调用方持有的是本次尝试自己的副本。
  std::vector<Piece<5, 2>> runOptimization(
    const std::vector<Eigen::Vector2d> & waypoints, std::vector<double> & segment_times,
    double data_weight, double & duration_out)
  {
    const auto start_time = now();
    MincoOptimizer::Params params;
    params.smooth_weight = smooth_weight_;
    params.data_weight = data_weight;
    params.tunnel_axis_weight = tunnel_axis_weight_;
    params.tunnel_corridor_weight = tunnel_corridor_weight_;
    params.obstacle_weight = obstacle_weight_;
    params.safe_dist = safe_dist_;
    params.obstacle_normal_only = false;
    params.lbfgs_memory_size = lbfgs_memory_size_;
    params.max_iterations = first_stage_max_iterations_;
    params.enable = enable_optimization_;
    minco_optimizer_->setParams(params);

    auto pieces = minco_optimizer_->optimize(waypoints, segment_times);

    // 确定性第二阶段：第一阶段轨迹求每段真实的最大速度/加速度（多项式求根，精确
    // 而不是采样），只增加违反动力学段的时间，再以原始路标点重跑一次较小迭代预算
    // 的 MINCO。第二阶段失败时保留第一阶段结果。
    //
    // 为什么不用采样：5 阶多项式的速度是 4 次函数，[0,T] 上最多 3 个内部极值，
    // 5 个等距样本（含两端）会漏掉靠近段中部的尖峰 —— 实测能把峰值低估 10-20%，
    // 恰好落在阈值附近时两阶段就不会触发，参考轨迹带着超速流到 MPC。MINCO 库的
    // Piece::getMaxVelRate()/getMaxAccRate() 已经是精确实现，这里直接用。
    //
    // 代价：每段各一次多项式求根（companion matrix 特征值），N 段 2N 次。0.18 m
    // 重采样下 10 m 路径约 55 段，合计毫秒级，且发生在 8 Hz 的平滑回调而不是
    // 30 Hz 的控制回调。two_stage.exact_dynamics_check 可关回采样行为。
    if (!pieces.empty() && two_stage_enable_ && pieces.size() == segment_times.size() &&
      two_stage_max_speed_ > 0.0 && two_stage_max_accel_ > 0.0)
    {
      std::vector<double> scales(pieces.size(), 1.0);
      bool needs_second_stage = false;
      for (size_t i = 0; i < pieces.size(); ++i) {
        double vmax = 0.0;
        double amax = 0.0;
        if (two_stage_exact_check_) {
          vmax = pieces[i].getMaxVelRate();
          amax = pieces[i].getMaxAccRate();
        } else {
          const double piece_duration = pieces[i].getDuration();
          for (int sample = 0; sample < two_stage_samples_per_piece_; ++sample) {
            const double ratio = static_cast<double>(sample) /
              static_cast<double>(two_stage_samples_per_piece_ - 1);
            const double t = ratio * piece_duration;
            vmax = std::max(vmax, pieces[i].getVel(t).norm());
            amax = std::max(amax, pieces[i].getAcc(t).norm());
          }
        }
        // 非有限值（求根数值异常/退化段）按不违例处理，交给发布前硬校验兜底，
        // 不要让一段坏数据把整条路径的时间分配放大。
        if (!std::isfinite(vmax) || !std::isfinite(amax)) {
          continue;
        }
        scales[i] = std::clamp(
          std::max({1.0, vmax / two_stage_max_speed_,
            std::sqrt(amax / two_stage_max_accel_)}), 1.0, two_stage_max_scale_);
        needs_second_stage = needs_second_stage || scales[i] > 1.001;
      }
      if (needs_second_stage) {
        // 一次固定三点平滑，防止相邻段时间突跳；只增不减。
        const auto raw_scales = scales;
        for (size_t i = 0; i < scales.size(); ++i) {
          const double left = raw_scales[i == 0 ? i : i - 1];
          const double right = raw_scales[i + 1 < raw_scales.size() ? i + 1 : i];
          scales[i] = std::clamp(
            0.25 * left + 0.5 * raw_scales[i] + 0.25 * right,
            1.0, two_stage_max_scale_);
          segment_times[i] *= scales[i];
        }
        // 精优化沿用本次尝试的权重（含修复时放大的 data_weight），只把障碍梯度
        // 限制在轨迹法向、并压缩迭代预算。
        MincoOptimizer::Params fine_params = params;
        fine_params.obstacle_normal_only = true;
        fine_params.max_iterations = two_stage_max_iterations_;
        minco_optimizer_->setParams(fine_params);
        auto fine_pieces = minco_optimizer_->optimize(waypoints, segment_times);
        if (!fine_pieces.empty()) {
          pieces = std::move(fine_pieces);
        }
        // 恢复第一阶段参数，下一条路径不会继承 fine 模式。
        fine_params.obstacle_normal_only = false;
        fine_params.max_iterations = first_stage_max_iterations_;
        minco_optimizer_->setParams(fine_params);
      }
    }

    // 放大后复核。历史缺陷：两阶段把段时间放大到 max_scale 为止就停，放大后的
    // 轨迹**不再检查**是否真的达标 —— 若 max_scale 封顶后仍超限，超速的参考轨迹
    // 会无声流到 MPC（MPC 拿它当前馈，跟不动就表现为跟踪误差/震荡，现场很难
    // 归因）。这里用同一个精确求根检查复核一次，只记日志不改行为：是否值得
    // 再做一轮「放大→重优化」应该由实车性能数据决定，而不是盲加一次 35-80 ms
    // 的重优化进 8 Hz 的平滑回调。
    if (!pieces.empty() && pieces.size() == segment_times.size() &&
      two_stage_max_speed_ > 0.0 && two_stage_max_accel_ > 0.0)
    {
      double worst_vel_ratio = 0.0;
      double worst_acc_ratio = 0.0;
      for (const auto & piece : pieces) {
        const double vmax = piece.getMaxVelRate();
        const double amax = piece.getMaxAccRate();
        if (std::isfinite(vmax)) {
          worst_vel_ratio = std::max(worst_vel_ratio, vmax / two_stage_max_speed_);
        }
        if (std::isfinite(amax)) {
          worst_acc_ratio = std::max(worst_acc_ratio,
            std::sqrt(amax / two_stage_max_accel_));
        }
      }
      const double worst = std::max(worst_vel_ratio, worst_acc_ratio);
      if (worst > 1.001) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "MINCO two-stage capped: trajectory still exceeds limits by %.1f%% "
          "(v-ratio %.2f, a-ratio %.2f, max_scale %.2f) — reference may be "
          "untrackable", (worst - 1.0) * 100.0, worst_vel_ratio, worst_acc_ratio,
          two_stage_max_scale_);
      }
    }

    duration_out = (now() - start_time).seconds();
    return pieces;
  }

  // 把 pieces 采样成兼容 Path（RViz / rm_tunnel_posture）+ P/V/A 快照（MPC 前馈）。
  void buildTrajectoryOutput(
    const std::vector<Piece<5, 2>> & pieces, const nav_msgs::msg::Path & input_path,
    nav_msgs::msg::Path & output_path, navigation2::TrajectorySnapshot & snapshot) const
  {
    output_path = nav_msgs::msg::Path();
    output_path.header = input_path.header;

    const double sample_dt = trajectory_sample_dt_;
    double total_duration = 0.0;
    for (const auto & piece : pieces) {
      total_duration += piece.getDuration();
    }

    snapshot = navigation2::TrajectorySnapshot();
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
  }

  static std::vector<Eigen::Vector2d> snapshotSamplePositions(
    const navigation2::TrajectorySnapshot & snapshot)
  {
    std::vector<Eigen::Vector2d> samples;
    samples.reserve(snapshot.samples.size());
    for (const auto & smp : snapshot.samples) {
      samples.emplace_back(smp.x, smp.y);
    }
    return samples;
  }

  // 发布前校验。返回空串表示通过，否则返回失败原因（进 PerformanceMonitor 与日志）。
  //
  // 两道互补的检查：
  //   global_clearance —— 全图 SFC（SfcCorridor，来自 rm_global_costmap）。覆盖整条
  //     轨迹，但只认致命格；MINCO 现有的距离场校验只覆盖 5x5 m 滚动窗口，5 m 以外
  //     的全局障碍对它不可见，这一道补上。SfcCorridor 未就绪（没收到代价地图）时
  //     跳过，与距离场未就绪时静默失效同一语义 —— 无法判定不等于判为不可行。
  //   safety_reject —— 局部距离场（历史行为，保留不动）。
  // 非 const：要锁 map_mutex_。本类不打算被并发调用（回调组串行），改成非 const
  // 比把 mutex 标 mutable 更能表达「校验会触碰受锁状态」。
  std::string evaluateTrajectory(const navigation2::TrajectorySnapshot & snapshot)
  {
    bool sfc_ready = false;
    {
      std::lock_guard<std::mutex> lk(map_mutex_);
      sfc_ready = sfc_corridor_.ready();
    }
    if (sfc_ready) {
      for (const auto & p : accepted_samples_) {
        double radius = 0.0;
        bool judged = false;
        {
          std::lock_guard<std::mutex> lk(map_mutex_);
          // 图外不判（理论上全局图覆盖整个场地，这里是防御），交给 MPC 兜底。
          judged = sfc_corridor_.insideMap(p.x(), p.y());
          if (judged) {
            radius = sfc_corridor_.clearanceRadius(p.x(), p.y());
          }
        }
        if (!judged) {
          continue;
        }
        if (radius < 0.0 || radius < global_check_robot_radius_) {
          return "global_clearance";
        }
      }
    }

    if (obstacle_weight_ > 0.0 && dist_field_for_check_) {
      for (const auto & smp : snapshot.samples) {
        double d = 0.0;
        Eigen::Vector2d g;
        if (!navigation2::DistanceFieldRegistry::query(
            *dist_field_for_check_, Eigen::Vector2d(smp.x, smp.y), d, g))
        {
          continue;  // 地图外不判，由 MPC 安全检查兜底
        }
        if (std::isfinite(d) && d < safe_dist_) {
          return "safety_reject";
        }
      }
    }
    return {};
  }

  // 强约束修复：在最偏离前端路径的采样点附近插入一个输入路标点，并同步对半拆分
  // 对应段时间。返回 false 表示插不了（点太少 / 偏离已经小到没有意义）。
  static bool insertRepairWaypoint(
    std::vector<Eigen::Vector2d> & waypoints, std::vector<double> & segment_times,
    const std::vector<Eigen::Vector2d> & samples, double min_deviation)
  {
    if (waypoints.size() < 3 || samples.size() < 3 ||
      waypoints.size() != segment_times.size() + 1)
    {
      return false;
    }
    // 每个采样点到「输入路标点折线」的距离，取最大者。输入折线是 A* 在同一张
    // 代价地图上搜出来的、满足 clearance 偏好的路径；平滑结果偏离它最远的位置
    // 就是需要额外自由度的位置。
    size_t worst = 0;
    double worst_dist = -1.0;
    for (size_t i = 0; i < samples.size(); ++i) {
      double best = std::numeric_limits<double>::infinity();
      for (size_t j = 0; j + 1 < waypoints.size(); ++j) {
        const Eigen::Vector2d ab = waypoints[j + 1] - waypoints[j];
        const double len2 = ab.squaredNorm();
        double t = 0.0;
        if (len2 > 1e-12) {
          t = std::clamp((samples[i] - waypoints[j]).dot(ab) / len2, 0.0, 1.0);
        }
        best = std::min(best, (samples[i] - (waypoints[j] + t * ab)).norm());
      }
      if (best > worst_dist) {
        worst_dist = best;
        worst = i;
      }
    }
    if (worst_dist <= min_deviation) {
      return false;
    }

    // 插到离最差采样点最近的输入路标点之前。
    size_t best_j = 0;
    double best_j_dist = std::numeric_limits<double>::infinity();
    for (size_t j = 0; j < waypoints.size(); ++j) {
      const double d = (samples[worst] - waypoints[j]).norm();
      if (d < best_j_dist) {
        best_j_dist = d;
        best_j = j;
      }
    }
    const size_t insert_at = std::clamp<size_t>(best_j, 1, waypoints.size() - 1);
    const Eigen::Vector2d midpoint = 0.5 * (waypoints[insert_at - 1] + waypoints[insert_at]);
    waypoints.insert(waypoints.begin() + static_cast<std::ptrdiff_t>(insert_at), midpoint);

    // 段时间同步拆分：被拆的那段对半分，其余不变。
    const double half_time = 0.5 * segment_times[insert_at - 1];
    segment_times[insert_at - 1] = half_time;
    segment_times.insert(
      segment_times.begin() + static_cast<std::ptrdiff_t>(insert_at - 1), half_time);
    return true;
  }

  std::string input_path_topic_;
  std::string output_path_topic_;

  double smooth_weight_;
  double data_weight_;
  double tunnel_axis_weight_;
  double tunnel_corridor_weight_;
  double tunnel_corridor_margin_m_;
  double tunnel_corridor_lateral_margin_m_;
  double obstacle_weight_;
  double safe_dist_;
  bool enable_optimization_;
  std::string semantic_map_topic_;

  // 全图级安全校验（SFC）参数。robot_radius 与代价地图的 robot_radius 同源。
  bool global_check_enable_;
  double global_check_robot_radius_;
  double global_check_max_range_;
  std::string global_check_topic_;
  int global_check_threshold_;
  bool global_check_unknown_lethal_;

  // 有界迭代修复。
  int repair_max_attempts_;
  double repair_waypoint_spacing_;

  double default_velocity_;
  double min_segment_time_;
  double turn_time_weight_;
  int first_stage_max_iterations_;
  int lbfgs_memory_size_;
  bool two_stage_enable_;
  double two_stage_max_speed_;
  double two_stage_max_accel_;
  double two_stage_max_scale_;
  int two_stage_samples_per_piece_;
  int two_stage_max_iterations_;
  bool two_stage_exact_check_{true};
  double trajectory_sample_dt_{0.05};

  std::unique_ptr<MincoOptimizer> minco_optimizer_;

  // 关断式性能观测。smoothAndPublish 与优化回调同线程（MutuallyExclusive 回调组），
  // 无需加锁；record 只在 enable 时计时。
  navigation2::PerformanceMonitor perf_monitor_;

  // 语义地图。轴线查询在优化回调里读，语义地图在订阅回调里写，两者可能不同线程
  // （单容器多线程执行器），用锁护住 receiver_。
  SemanticMapReceiver receiver_;
  std::mutex map_mutex_;
  // 语义地图代次：地图每次**内容**更新 +1。与路径指纹一起构成去重键。
  std::uint64_t semantic_generation_{0};
  // 隧道影响区（本体 + margin），给横向走廊约束提供几何。与 receiver_ 同锁更新。
  // unique_ptr 而不是值成员：build() 逐本体格做半径展开，没隧道时返回空对象，
  // 用空指针可以让查询入口一次判掉。
  std::unique_ptr<TunnelRegionGrid> tunnel_region_;

  // 全图级安全校验的距离场。写：全局代价地图回调；读：smoothAndPublish。
  // 三个订阅同在默认 MutuallyExclusive 回调组，实际串行；加锁是为了让
  // 「读方拿到的派生物与语义地图/代价地图一致」这一约束显式可见。
  SfcCorridor sfc_corridor_;
  // 本轮优化的距离场快照（优化与校验共用，避免修复中途换判据）。
  std::shared_ptr<const navigation2::DistanceFieldSnapshot> dist_field_for_check_;
  // 本轮已接受轨迹的采样点缓存（校验与修复都用）。
  std::vector<Eigen::Vector2d> accepted_samples_;

  // 去重缓存。只在 smoothAndPublish（MutuallyExclusive 回调组）里读写，
  // 与语义地图订阅同组串行，无需额外锁。
  InputFingerprint last_fingerprint_;
  nav_msgs::msg::Path last_output_path_;
  bool last_output_valid_{false};

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<decision_interfaces::msg::SemanticMap>::SharedPtr semantic_map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

}  // namespace navigation2

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(navigation2::RmMincoPathSmoother)

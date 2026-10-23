#include <dirent.h>
#include <execinfo.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pthread.h>
#include <signal.h>
#include <sys/syscall.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <stdexcept>
#include <thread>

#include "ground_segmentation/ground_segmentation.h"

namespace linefit_ground_segmentation
{
namespace
{

// ---- 卡死看门狗 ----------------------------------------------------------
// 2026-08 线上出现过一次：scanCallback 整体卡死 10s+（/segmentation/obstacle
// 停发 → 代价图 stale → MPC 恢复链 → 导航 FAILED），SIGINT/SIGTERM 都杀不掉，
// launch 只能 SIGKILL。回调是单线程 executor 跑的，卡住后节点没有任何自证
// 能力。这里加一个独立看门狗线程：
//   * scanCallback 每过一个阶段就推一次心跳/阶段号；
//   * 心跳停顿 > 3s → 打印卡住的阶段、给 executor 线程发 SIGUSR2 打 backtrace、
//     并 dump 本进程全部线程的内核等待通道（wchan），下次复现即可直接定位；
//   * 输出节流（见 watchdogLoop）：首次停顿立刻全量 dump，之后每 10s 一行状态、
//     完整 backtrace 至多每 60s 一次，避免卡死时刷屏。
// 所有诊断输出都走 write/fprintf(stderr)，尽量少用可能被卡住的锁。
std::atomic<int> g_seg_stage{0};
std::atomic<uint64_t> g_seg_heartbeat{0};
std::atomic<bool> g_seg_watchdog_stop{true};

void stuckBacktraceHandler(int)
{
  // backtrace + backtrace_symbols_fd 是 async-signal-safe 的（backtrace_symbols_fd
  // 直接写 fd，不 malloc），可以在卡住的线程上安全执行。看门狗会给进程内所有
  // 线程发 SIGUSR2，这里带上 tid 才能区分谁是谁。
  char header[128];
  const int n = snprintf(
    header, sizeof(header), "\n[ground_segmentation] === backtrace of thread %ld ===\n",
    static_cast<long>(syscall(SYS_gettid)));
  if (n > 0) {
    (void)!write(STDERR_FILENO, header, static_cast<size_t>(n));
  }
  void * frames[64];
  const int n_frames = backtrace(frames, 64);
  backtrace_symbols_fd(frames, n_frames, STDERR_FILENO);
  static const char footer[] = "[ground_segmentation] === end of backtrace ===\n";
  (void)!write(STDERR_FILENO, footer, sizeof(footer) - 1);
}

// 给本进程所有线程（除自己）发 SIGUSR2：每个线程在信号里打印自己的栈。
// executor 线程卡在哪、池 worker/任务线程卡在哪，一张图全出来。
void signalAllThreads()
{
  DIR * dir = opendir("/proc/self/task");
  if (!dir) {
    return;
  }
  const long self = static_cast<long>(syscall(SYS_gettid));
  const pid_t pid = static_cast<pid_t>(syscall(SYS_getpid));
  struct dirent * entry;
  while ((entry = readdir(dir)) != nullptr) {
    if (entry->d_name[0] == '.') {
      continue;
    }
    const long tid = atol(entry->d_name);
    if (tid == 0 || tid == self) {
      continue;
    }
    syscall(SYS_tgkill, pid, tid, SIGUSR2);
  }
  closedir(dir);
}

void dumpThreadWchan()
{
  // 按 tid 列本进程所有线程的 comm + 内核等待通道（futex/sys_poll/…），
  // 配合 executor 线程的 backtrace 判断卡在锁、网络还是其他 syscall。
  DIR * dir = opendir("/proc/self/task");
  if (!dir) {
    return;
  }
  fprintf(stderr, "[ground_segmentation] thread states (tid comm wchan):\n");
  struct dirent * entry;
  while ((entry = readdir(dir)) != nullptr) {
    if (entry->d_name[0] == '.') {
      continue;
    }
    char path[512];
    char buf[256];
    snprintf(path, sizeof(path), "/proc/self/task/%s/comm", entry->d_name);
    buf[0] = '\0';
    if (FILE * f = fopen(path, "r")) {
      if (fgets(buf, sizeof(buf), f) == nullptr) {
        buf[0] = '\0';
      }
      fclose(f);
    }
    buf[strcspn(buf, "\n")] = '\0';
    snprintf(path, sizeof(path), "/proc/self/task/%s/wchan", entry->d_name);
    char wchan[256] = "?";
    if (FILE * f = fopen(path, "r")) {
      if (fgets(wchan, sizeof(wchan), f) == nullptr) {
        snprintf(wchan, sizeof(wchan), "?");
      }
      fclose(f);
    }
    wchan[strcspn(wchan, "\n")] = '\0';
    fprintf(stderr, "  tid %-6s %-20s %s\n", entry->d_name, buf, wchan);
  }
  closedir(dir);
}

void watchdogLoop()
{
  // 卡死时的输出节流。旧实现每 0.5s 打一行提示，而且「停顿→恢复→再停顿」的
  // 抖动型卡死每次都会重新 dump 一遍全部线程栈（一次 dump ≈ 170 行）：实测一次
  // 坏跑刷出 4 万+ 行 launch 日志，既把别的节点日志冲掉，又恰好在最缺 CPU 的
  // 时候白烧 CPU（signalAllThreads 会给每个线程发信号打栈）。
  //
  // 新策略：
  //   * 首次停顿 >3s：立刻 dump 全部线程 backtrace + wchan；
  //   * 停顿期间每 10s 打一行状态（成本 1 行），能看出卡了多久、卡点变没变；
  //   * 完整 backtrace 至多每 60s 重打一次（栈通常不变，没必要 10s 一遍）。
  // last_status / last_dump 跨「恢复」不重置，否则每 10s 来一帧的抖动型卡死会
  // 退化成每帧一次全量 dump —— 那正是旧实现刷屏的成因。
  constexpr double kStallThresholdSec = 3.0;
  constexpr double kStatusIntervalSec = 10.0;
  constexpr double kDumpIntervalSec = 60.0;
  uint64_t last_heartbeat = g_seg_heartbeat.load();
  int last_stage = g_seg_stage.load();
  auto stalled_since = std::chrono::steady_clock::now();
  // 初始值往前各推一个间隔，保证「首次卡死立刻 dump 并打状态」。
  auto last_status = stalled_since - std::chrono::seconds(10);
  auto last_dump = stalled_since - std::chrono::seconds(60);
  bool stalled = false;
  while (!g_seg_watchdog_stop.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    const uint64_t hb = g_seg_heartbeat.load();
    if (hb == last_heartbeat) {
      const auto now = std::chrono::steady_clock::now();
      if (!stalled) {
        stalled = true;
        stalled_since = now;
      }
      const double secs = std::chrono::duration<double>(now - stalled_since).count();
      if (secs > kStallThresholdSec) {
        const bool want_status =
          std::chrono::duration<double>(now - last_status).count() >= kStatusIntervalSec;
        const bool want_dump =
          std::chrono::duration<double>(now - last_dump).count() >= kDumpIntervalSec;
        if (want_status || want_dump) {
          fprintf(
            stderr,
            "[ground_segmentation] WATCHDOG: scanCallback stalled %.1fs at stage %d"
            " (last moving stage %d)%s\n",
            secs, g_seg_stage.load(), last_stage,
            want_dump ? "; dumping backtraces of all threads" : "");
          if (want_dump) {
            signalAllThreads();
            dumpThreadWchan();
            last_dump = now;
          }
          last_status = now;
        }
      }
    } else {
      last_heartbeat = hb;
      last_stage = g_seg_stage.load();
      stalled = false;
    }
  }
}

void bumpStage(int stage)
{
  g_seg_stage.store(stage, std::memory_order_relaxed);
  g_seg_heartbeat.fetch_add(1, std::memory_order_relaxed);
}

}  // namespace

class SegmentationNode : public rclcpp::Node
{
public:
  SegmentationNode(const rclcpp::NodeOptions & node_options);
  ~SegmentationNode() override;
  void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  GroundSegmentationParams params_;
  std::shared_ptr<GroundSegmentation> segmenter_;
  std::string gravity_aligned_frame_;
  // 可选处理坐标系：权威云在 odom，但 LineFit 的 r_min/r_max 与径向 bins 必须以当前
  // 传感器/虚拟云台为原点。设为 base_link 时按消息 stamp 把 odom 云搬回局部系，输出
  // header 同步改为 base_link，costmap 再按同一 stamp 变回 map，避免移动后点距 odom
  // 原点超过 r_max 而全部消失。
  std::string processing_frame_;
  // 雷达安装高度不再由 yaml 手填，而是从 URDF 外参的 TF 里查出来，见
  // resolveSensorHeight()。空字符串表示关闭该行为、退回 sensor_height 参数。
  std::string sensor_height_frame_;
  // sensor_height_frame_ 原点离地的高度。默认 0：实测 RMUL.pcd 的地面主峰在
  // -0.17，正好等于外参里的雷达安装高度 0.175，说明 base_link 原点基本就在地面
  // 上。底盘换结构、base_link 原点抬到轮轴上时才需要填。
  double base_to_ground_z_{0.0};
  bool sensor_height_resolved_{false};
  // TF 查询连续失败计数：失败几次后锁定参数兜底值，避免每个点云回调都空等
  // 0.2s 超时，把分割节拍拖垮。
  int sensor_height_tf_failures_{0};
  std::thread watchdog_thread_;

private:
  void resolveSensorHeight(const std::string & cloud_frame);
};

SegmentationNode::SegmentationNode(const rclcpp::NodeOptions & node_options)
: Node("ground_segmentation", node_options)
{
  gravity_aligned_frame_ = this->declare_parameter("gravity_aligned_frame", "gravity_aligned");
  processing_frame_ = this->declare_parameter("processing_frame", std::string(""));

  params_.visualize = this->declare_parameter("visualize", params_.visualize);
  params_.n_bins = this->declare_parameter("n_bins", params_.n_bins);
  params_.n_segments = this->declare_parameter("n_segments", params_.n_segments);
  params_.max_dist_to_line = this->declare_parameter("max_dist_to_line", params_.max_dist_to_line);
  params_.max_slope = this->declare_parameter("max_slope", params_.max_slope);
  params_.min_slope = this->declare_parameter("min_slope", params_.min_slope);
  params_.long_threshold = this->declare_parameter("long_threshold", params_.long_threshold);
  params_.max_long_height = this->declare_parameter("max_long_height", params_.max_long_height);
  params_.max_start_height = this->declare_parameter("max_start_height", params_.max_start_height);
  // sensor_height 只是收不到 TF 时的兜底值。真值从 sensor_height_frame ->
  // 点云 frame 的 TF 里取（见 resolveSensorHeight），这样雷达离地高度在整个仓库
  // 里只有 URDF 外参一处来源：config/{reality,simulation}/measurement_params_*.yaml。
  // 以前这个值要跟外参手工同步，实车就同步失败了 —— 外参 z 填 0.0、
  // 这里填 0.59、xacro 的 default 又是 0.49，三个数互相矛盾。
  params_.sensor_height = this->declare_parameter("sensor_height", params_.sensor_height);
  sensor_height_frame_ = this->declare_parameter("sensor_height_frame", std::string("base_link"));
  base_to_ground_z_ = this->declare_parameter("base_to_ground_z", base_to_ground_z_);
  params_.line_search_angle =
    this->declare_parameter("line_search_angle", params_.line_search_angle);
  params_.n_threads = this->declare_parameter("n_threads", params_.n_threads);
  // Params that need to be squared.
  const double r_min = this->declare_parameter("r_min", std::sqrt(params_.r_min_square));
  const double r_max = this->declare_parameter("r_max", std::sqrt(params_.r_max_square));
  const double max_fit_error =
    this->declare_parameter("max_fit_error", std::sqrt(params_.max_error_square));
  if (!std::isfinite(r_min) || r_min < 0.0) {
    throw std::invalid_argument("r_min must be finite and greater than or equal to zero");
  }
  if (!std::isfinite(r_max) || r_max <= r_min) {
    throw std::invalid_argument("r_max must be finite and greater than r_min");
  }
  if (!std::isfinite(max_fit_error) || max_fit_error < 0.0) {
    throw std::invalid_argument("max_fit_error must be finite and greater than or equal to zero");
  }
  // 结构性参数校验：n_bins/n_segments <= 0 会让库里的 bin_step/segment_step 除零、
  // segments_ 为空，随后 operator[] 越界（UB）。放到建段器之前抛异常，让配置错误
  // 在启动时炸掉而不是在点云回调里写坏内存。
  if (params_.n_bins <= 0) {
    throw std::invalid_argument("n_bins must be greater than zero");
  }
  if (params_.n_segments <= 0) {
    throw std::invalid_argument("n_segments must be greater than zero");
  }
  if (params_.n_threads <= 0) {
    throw std::invalid_argument("n_threads must be greater than zero");
  }
  if (!std::isfinite(params_.line_search_angle) || params_.line_search_angle < 0.0) {
    throw std::invalid_argument(
      "line_search_angle must be finite and greater than or equal to zero");
  }
  if (!std::isfinite(params_.sensor_height) || params_.sensor_height <= 0.0) {
    throw std::invalid_argument("sensor_height must be finite and greater than zero");
  }
  params_.r_min_square = r_min * r_min;
  params_.r_max_square = r_max * r_max;
  params_.max_error_square = max_fit_error * max_fit_error;
  segmenter_ = std::make_shared<GroundSegmentation>(params_);
  std::string ground_topic, obstacle_topic, input_topic;
  ground_topic = this->declare_parameter("ground_output_topic", "ground_cloud");
  obstacle_topic = this->declare_parameter("obstacle_output_topic", "obstacle_cloud");
  input_topic = this->declare_parameter("input_topic", "input_cloud");
  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic, rclcpp::SensorDataQoS(),
    std::bind(&SegmentationNode::scanCallback, this, std::placeholders::_1));
  // 发布用 RELIABLE：pointcloud_to_laserscan（cloud_in 重映射到
  // /segmentation/obstacle）和 RViz 都用 RELIABLE 订阅，而 SensorDataQoS
  // 是 best-effort —— best-effort 发布者不会给 RELIABLE 订阅者投递任何消息，
  // 建图模式下 slam_toolbox 一直吃到空扫描、地图永远建不出来（线上 2026-08
  // 日志里的 "requesting incompatible QoS ... RELIABILITY_QOS_POLICY" 就是
  // 这个订阅者）。RELIABLE 发布对 best-effort 订阅（全局/局部代价图、MPC 的
  // ESDF）是兼容的，一改全通。
  const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
  ground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(ground_topic, output_qos);
  obstacle_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(obstacle_topic, output_qos);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  sensor_height_resolved_ = sensor_height_frame_.empty();
  RCLCPP_INFO(this->get_logger(), "Segmentation node initialized");

  // 卡死看门狗：独立线程监视 scanCallback 心跳，停顿 >3s 时打栈/wchan。
  struct sigaction sa
  {
  };
  sa.sa_handler = stuckBacktraceHandler;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = SA_RESTART;
  sigaction(SIGUSR2, &sa, nullptr);
  g_seg_watchdog_stop.store(false);
  watchdog_thread_ = std::thread(watchdogLoop);
}

SegmentationNode::~SegmentationNode()
{
  g_seg_watchdog_stop.store(true);
  if (watchdog_thread_.joinable()) {
    watchdog_thread_.join();
  }
}

// 把 sensor_height 换成从 TF 查出来的真值：
//   sensor_height = z(sensor_height_frame -> 点云 frame) + base_to_ground_z
// 前一项就是 URDF 外参里的雷达安装高度，后一项是 base_link 原点离地的高度
// （底盘几何，换雷达位置不会变）。这样「雷达装多高」在整个仓库里只有
// measurement_params_{real,sim}.yaml 一处来源，real 和 sim 走同一条逻辑。
// 只在第一次成功时重建 segmenter：外参是静态的，查到一次就够。
void SegmentationNode::resolveSensorHeight(const std::string & cloud_frame)
{
  if (sensor_height_resolved_ || cloud_frame.empty()) {
    return;
  }

  geometry_msgs::msg::TransformStamped tf_stamped;
  try {
    // TimePointZero 不阻塞，但 TF 数据完全缺失时也会无限等 —— 给 0.2s 上限，
    // 查不到就走参数兜底，绝不让回调卡在这里。
    tf_stamped = tf_buffer_->lookupTransform(
      sensor_height_frame_, cloud_frame, tf2::TimePointZero, tf2::durationFromSec(0.2));
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Cannot look up %s -> %s for sensor_height, keep the "
      "configured fallback %.3f: %s",
      sensor_height_frame_.c_str(), cloud_frame.c_str(), params_.sensor_height, ex.what());
    // TF 长期缺失时不能在每个点云回调里都空等 0.2s：5 次失败后锁定兜底值，
    // 等外部重新发布外参后需重启节点才会再次尝试解析。
    if (++sensor_height_tf_failures_ >= 5) {
      RCLCPP_ERROR(
        this->get_logger(),
        "sensor_height TF lookup failed %d times in a row; "
        "locking to configured fallback %.3f to keep the segmentation "
        "cadence. Restart the node after fixing the TF tree.",
        sensor_height_tf_failures_, params_.sensor_height);
      sensor_height_resolved_ = true;
    }
    return;
  }
  sensor_height_tf_failures_ = 0;

  const double mount_z = tf_stamped.transform.translation.z;
  const double resolved = mount_z + base_to_ground_z_;
  if (!std::isfinite(resolved) || resolved <= 0.0) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Resolved sensor_height %.3f (mount %.3f + base_to_ground "
      "%.3f) is not a positive finite number. Check the URDF "
      "extrinsic in measurement_params_*.yaml. Keep %.3f.",
      resolved, mount_z, base_to_ground_z_, params_.sensor_height);
    sensor_height_resolved_ = true;
    return;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "sensor_height resolved from TF: %.3f (mount %.3f above %s + "
    "%.3f base_to_ground_z), configured fallback was %.3f",
    resolved, mount_z, sensor_height_frame_.c_str(), base_to_ground_z_, params_.sensor_height);
  params_.sensor_height = resolved;
  segmenter_ = std::make_shared<GroundSegmentation>(params_);
  sensor_height_resolved_ = true;
}

void SegmentationNode::scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  bumpStage(1);
  const bool publish_ground =
    ground_pub_->get_subscription_count() + ground_pub_->get_intra_process_subscription_count() > 0;
  const bool publish_obstacle = obstacle_pub_->get_subscription_count() +
                                  obstacle_pub_->get_intra_process_subscription_count() >
                                0;
  if (!publish_ground && !publish_obstacle) {
    return;
  }

  // 外参是静态 TF，开机后头几帧可能还没进 buffer，所以在回调里查、查到即锁定。
  resolveSensorHeight(msg->header.frame_id);

  bumpStage(2);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg, cloud);
  pcl::PointCloud<pcl::PointXYZ> cloud_transformed;
  std::string output_frame = msg->header.frame_id;

  bumpStage(3);
  std::vector<int> labels;

  bool is_original_pc = true;
  if (!processing_frame_.empty() && processing_frame_ != msg->header.frame_id) {
    try {
      const auto tf_stamped = tf_buffer_->lookupTransform(
        processing_frame_, msg->header.frame_id, msg->header.stamp, tf2::durationFromSec(0.05));
      const Eigen::Quaterniond q(
        tf_stamped.transform.rotation.w, tf_stamped.transform.rotation.x,
        tf_stamped.transform.rotation.y, tf_stamped.transform.rotation.z);
      const Eigen::Vector3d t(
        tf_stamped.transform.translation.x, tf_stamped.transform.translation.y,
        tf_stamped.transform.translation.z);
      Eigen::Affine3d tf = Eigen::Affine3d::Identity();
      tf.translate(t);
      tf.rotate(q);
      output_frame = processing_frame_;
      pcl::transformPointCloud(cloud, cloud_transformed, tf);
      is_original_pc = false;
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Drop point cloud: cannot transform processing frame %s <- %s: %s",
        processing_frame_.c_str(), msg->header.frame_id.c_str(), ex.what());
      return;
    }
  }

  // processing_frame 已经是重力对齐系时无需第二次旋转。否则只旋转、不平移。
  if (
    is_original_pc && !gravity_aligned_frame_.empty() &&
    gravity_aligned_frame_ != msg->header.frame_id) {
    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
      // 给查询加 50ms 上限：Timeout 0（缺省）意味着「等多久都行」，一旦这条
      // TF 链某帧缺失，单线程回调会永久卡死、节点再也杀不掉（2026-08 线上
      // 正是这个签名：障碍点云停发 10s+、SIGTERM 无效、只能 SIGKILL）。
      tf_stamped = tf_buffer_->lookupTransform(
        gravity_aligned_frame_, msg->header.frame_id, msg->header.stamp,
        tf2::durationFromSec(0.05));
      // Remove translation part.
      tf_stamped.transform.translation.x = 0;
      tf_stamped.transform.translation.y = 0;
      tf_stamped.transform.translation.z = 0;
      Eigen::Affine3d tf;
      tf.translate(Eigen::Vector3d(0, 0, 0));
      tf.rotate(Eigen::Quaterniond(
        tf_stamped.transform.rotation.w, tf_stamped.transform.rotation.x,
        tf_stamped.transform.rotation.y, tf_stamped.transform.rotation.z));
      // tf::transformMsgToEigen(tf_stamped.transform, tf);
      pcl::transformPointCloud(cloud, cloud_transformed, tf);
      is_original_pc = false;
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Drop point cloud: cannot transform into required gravity frame: %s", ex.what());
      return;
    }
  }

  // Trick to avoid PC copy if we do not transform.
  const pcl::PointCloud<pcl::PointXYZ> & cloud_proc = is_original_pc ? cloud : cloud_transformed;

  bumpStage(4);
  const auto segment_start = std::chrono::steady_clock::now();
  segmenter_->segment(cloud_proc, &labels);
  const double segment_ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - segment_start)
      .count();
  if (segment_ms > 100.0) {
    // 正常单帧只有几毫秒；持续变慢说明有问题在累积，卡死前就能看到趋势。
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "segment() took %.1f ms for %zu points (normal is <10 ms)", segment_ms, cloud.size());
  }
  bumpStage(5);
  pcl::PointCloud<pcl::PointXYZ> ground_cloud, obstacle_cloud;
  if (publish_ground) {
    ground_cloud.reserve(cloud.size());
  }
  if (publish_obstacle) {
    obstacle_cloud.reserve(cloud.size());
  }
  // labels 对应 cloud_proc；输出也必须取 cloud_proc。旧代码变换到 processing_frame 后
  // 却仍复制原始 cloud 点，再把 header 改成 processing_frame，导致 odom 数值被伪装成
  // base_link 点：启动时那一片障碍会永久粘在车上随机器人移动。
  for (size_t i = 0; i < cloud_proc.size(); ++i) {
    // NaN/Inf 点不是有效观测：linefit 里它们的 label 恒为 0（障碍），旧实现会原样
    // 进 /segmentation/obstacle。下游（代价地图高度带 / 距离变换 / laserscan）
    // 拿到 NaN 会把整帧观测算成 NaN，轻则丢帧重则生成幽灵障碍。这里在出口统一
    // 剔掉，与库内「非有限点不参与分割」保持一致。
    const auto & p = cloud_proc[i];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      continue;
    }
    // label 约定：0=障碍、1=地面、2=忽略（r_min/r_max 外或非有限点）。
    // 范围外点不参与地面/障碍判定，也不能当障碍发布给 costmap/建图。
    if (labels[i] == 2) {
      continue;
    }
    if (labels[i] == 1) {
      if (publish_ground) {
        ground_cloud.push_back(p);
      }
    } else if (publish_obstacle) {
      obstacle_cloud.push_back(p);
    }
  }
  if (publish_ground) {
    sensor_msgs::msg::PointCloud2 ground_msg;
    pcl::toROSMsg(ground_cloud, ground_msg);
    ground_msg.header = msg->header;
    ground_msg.header.frame_id = output_frame;
    ground_pub_->publish(ground_msg);
  }
  if (publish_obstacle) {
    sensor_msgs::msg::PointCloud2 obstacle_msg;
    pcl::toROSMsg(obstacle_cloud, obstacle_msg);
    obstacle_msg.header = msg->header;
    obstacle_msg.header.frame_id = output_frame;
    obstacle_pub_->publish(obstacle_msg);
  }
  bumpStage(6);
}

}  // namespace linefit_ground_segmentation

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(linefit_ground_segmentation::SegmentationNode)

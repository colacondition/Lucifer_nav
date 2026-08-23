#ifndef NAVIGATION2__SHARED_STATE_HPP_
#define NAVIGATION2__SHARED_STATE_HPP_

// 进程内共享状态：给同容器（nav_container_mt）内多个组件传递高频数据的免序列化通道。
//
// 为什么存在：Lucifer 的 navigation2 全部组件编进同一个 shared library、跑在同一个
// 固定线程容器里（同进程）。sentry 里 planner 直接拿 ROGMap 的 MapQueryInterface*
// 查内存地图，避免为每次优化序列化整张 OccupancyGrid；Lucifer 的 latch 话题是
// transient_local、不能开 intra-process，但同进程同库意味着可以走「单例 + 快照」。
//
// 纪律（对齐 Lucifer 既有线程模型，不新起线程、不破坏回调组串行化）：
// - 写者（如 rm_local_costmap / rm_minco_path_smoother）在自己的 MutuallyExclusive
//   回调组里调用 publish()；读者（如 rm_mpc_controller）在自己的回调组里 latest()。
//   跨节点读写不在同一回调组，因此单例内部必须加锁。
// - 用 std::shared_ptr 交换快照（copy-on-write）：加锁只在替换指针的瞬间，读者拿到
//   的 shared_ptr 在整个使用期都有效，写者不会阻塞读者做逐格拷贝。
// - 快照是「最近一次」语义，不是队列：高频数据只消费最新一帧，避免排队累积延迟，
//   与 odom/costmap 的 KeepLast(1) + volatile 纪律一致。
// - 所有查询都返回明确的结果状态；读者必须能接受「快照不存在 / 已过期」，退回
//   它原有的 ROS 话题订阅，不把进程内通道当唯一真源。

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/time.hpp>

namespace navigation2
{

// ---- MINCO 轨迹快照：smoother 采样 P/V/A 后写入，MPC 作为参考前馈 ----
// 采样在时间上等距（dt），同时记录累积弧长 s。MPC 的 route_tracker 维护弧长进度，
// 用 s 直接二分定位到采样点，取 P/V 作为参考 —— 这样保留弧长域跟踪逻辑不动，
// 只把「弧长 → 参考点」的查表从 Path 的 pos_by_arc + tangent×speed 换成轨迹的
// P/V/A 真值（MINCO 已经算出来，之前发布 Path 时被丢掉了）。速度/加速度是世界系分量。
struct TrajectorySample
{
  double t{0.0};
  double s{0.0};
  double x{0.0};
  double y{0.0};
  double vx{0.0};
  double vy{0.0};
  double ax{0.0};
  double ay{0.0};
};

struct TrajectorySnapshot
{
  std::vector<TrajectorySample> samples;
  double dt{0.1};
  double total_duration{0.0};
  double total_length{0.0};
  rclcpp::Time stamp;
  bool valid{false};

  // 按弧长 s 线性插值取参考点（位置 + 速度 + 加速度）。s 超出轨迹时 clamp 到末端，
  // 返回 false 表示轨迹为空。MPC 用它替代 Path 弧长采样。
  // 采样在时间上等距，累积弧长 s 一般不是等距（MINCO 变速），所以按 s 二分定位，
  // 不能用 s/span*(n-1) 当弧长均匀索引。
  bool queryAtArc(double s, TrajectorySample & out) const
  {
    if (!valid || samples.size() < 2) {
      return false;
    }
    if (s <= samples.front().s) {
      out = samples.front();
      return true;
    }
    if (s >= samples.back().s) {
      out = samples.back();
      return true;
    }
    const auto it = std::lower_bound(
      samples.begin(), samples.end(), s,
      [](const TrajectorySample & sample, double key) { return sample.s < key; });
    if (it == samples.begin()) {
      out = *it;
      return true;
    }
    if (it == samples.end()) {
      out = samples.back();
      return true;
    }
    const auto & p1 = *it;
    const auto & p0 = *(it - 1);
    const double ds = p1.s - p0.s;
    const double a = (ds > 1e-12) ? ((s - p0.s) / ds) : 0.0;
    out.t = p0.t + a * (p1.t - p0.t);
    out.s = s;
    out.x = p0.x + a * (p1.x - p0.x);
    out.y = p0.y + a * (p1.y - p0.y);
    out.vx = p0.vx + a * (p1.vx - p0.vx);
    out.vy = p0.vy + a * (p1.vy - p0.vy);
    out.ax = p0.ax + a * (p1.ax - p0.ax);
    out.ay = p0.ay + a * (p1.ay - p0.ay);
    return true;
  }
};

class TrajectoryCache
{
public:
  static TrajectoryCache & instance()
  {
    static TrajectoryCache s;
    return s;
  }

  void publish(TrajectorySnapshot snapshot)
  {
    auto next = std::make_shared<TrajectorySnapshot>(std::move(snapshot));
    std::lock_guard<std::mutex> lock(mutex_);
    latest_ = std::move(next);
  }

  // 优化失败 / 安全拒绝时把快照标成无效。MPC 用 total_length 5% 容差匹配 Path，
  // 若不失效，下一帧折线 Path 可能仍对上旧的 MINCO P/V。
  // 与 publish() 同一把锁、同一份 shared_ptr 交换，不新起线程、不排队。
  void invalidate()
  {
    auto next = std::make_shared<TrajectorySnapshot>();
    next->valid = false;
    std::lock_guard<std::mutex> lock(mutex_);
    latest_ = std::move(next);
  }

  std::shared_ptr<const TrajectorySnapshot> latest() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_;
  }

private:
  TrajectoryCache() = default;
  mutable std::mutex mutex_;
  std::shared_ptr<const TrajectorySnapshot> latest_;
};

// ---- 2D 有符号距离场快照：local costmap 维护，smoother/MPC 查询 ----
// distance[i] 为第 i 个栅格到最近障碍的有符号距离（米）：正=障碍外、负=障碍内、
// 0=障碍表面。与 sentry 的 Signed ESDF 语义一致，但落在 Lucifer 已有的 2D 栅格上，
// 供 MINCO 障碍代价（外推）与恢复链（梯度脱困）共用，避免各自再订一份 OccupancyGrid。
struct DistanceFieldSnapshot
{
  double resolution{0.05};
  double origin_x{0.0};
  double origin_y{0.0};
  int width{0};
  int height{0};
  std::vector<float> distance;
  rclcpp::Time stamp;
  bool valid{false};
};

class DistanceFieldRegistry
{
public:
  static DistanceFieldRegistry & instance()
  {
    static DistanceFieldRegistry s;
    return s;
  }

  void publish(DistanceFieldSnapshot snapshot)
  {
    auto next = std::make_shared<DistanceFieldSnapshot>(std::move(snapshot));
    std::lock_guard<std::mutex> lock(mutex_);
    latest_ = std::move(next);
  }

  std::shared_ptr<const DistanceFieldSnapshot> latest() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_;
  }

  // 世界坐标 → 有符号距离 + 梯度。返回 false 表示查询点在地图外或快照无效。
  // grad 用中心差分（查询点不在格点上时），单位长度方向指向远离障碍。
  static bool query(
    const DistanceFieldSnapshot & field, const Eigen::Vector2d & pos,
    double & dist, Eigen::Vector2d & grad)
  {
    if (!field.valid || field.distance.empty() || field.width <= 0 || field.height <= 0) {
      return false;
    }
    const double gx = (pos.x() - field.origin_x) / field.resolution;
    const double gy = (pos.y() - field.origin_y) / field.resolution;
    const int cx = static_cast<int>(std::floor(gx));
    const int cy = static_cast<int>(std::floor(gy));
    if (cx < 0 || cy < 0 || cx + 1 >= field.width || cy + 1 >= field.height) {
      return false;
    }
    const double fx = gx - static_cast<double>(cx);
    const double fy = gy - static_cast<double>(cy);
    const auto & d = field.distance;
    const auto idx = [&field](int x, int y) { return static_cast<size_t>(y) * field.width + x; };
    const double d00 = d[idx(cx, cy)];
    const double d10 = d[idx(cx + 1, cy)];
    const double d01 = d[idx(cx, cy + 1)];
    const double d11 = d[idx(cx + 1, cy + 1)];
    // 双线性插值。
    dist = (1.0 - fx) * (1.0 - fy) * d00 + fx * (1.0 - fy) * d10 +
           (1.0 - fx) * fy * d01 + fx * fy * d11;
    // 中心差分梯度（世界尺度）。
    grad.x() = ((1.0 - fy) * (d10 - d00) + fy * (d11 - d01)) / field.resolution;
    grad.y() = ((1.0 - fx) * (d01 - d00) + fx * (d11 - d10)) / field.resolution;
    return true;
  }

  // 二次可分离插值查询，给轨迹优化热路径使用。以离查询点最近的格点为中心取 3x3
  // 邻域，先沿 x 再沿 y 拟合二次函数，并解析求导。相比双线性插值，梯度跨 cell
  // 边界更平滑，也能缓和对称窄通道中的梯度突变；仅需固定 3x3 标量读取，不分配内存。
  // 地图边缘不足 3x3 时回退到 query()。
  static bool queryQuadratic(
    const DistanceFieldSnapshot & field, const Eigen::Vector2d & pos,
    double & dist, Eigen::Vector2d & grad)
  {
    if (!field.valid || field.distance.empty() || field.width <= 0 || field.height <= 0 ||
      !std::isfinite(field.resolution) || field.resolution <= 0.0)
    {
      return false;
    }

    const double gx = (pos.x() - field.origin_x) / field.resolution;
    const double gy = (pos.y() - field.origin_y) / field.resolution;
    const int cx = static_cast<int>(std::floor(gx + 0.5));
    const int cy = static_cast<int>(std::floor(gy + 0.5));
    if (cx - 1 < 0 || cy - 1 < 0 || cx + 1 >= field.width || cy + 1 >= field.height) {
      return query(field, pos, dist, grad);
    }

    const double ux = gx - static_cast<double>(cx);
    const double uy = gy - static_cast<double>(cy);
    const auto & d = field.distance;
    const auto idx = [&field](int x, int y) {
        return static_cast<size_t>(y) * static_cast<size_t>(field.width) +
               static_cast<size_t>(x);
      };
    const auto quadratic = [](double fm, double f0, double fp, double u) {
        return f0 + 0.5 * u * (fp - fm) + 0.5 * u * u * (fp - 2.0 * f0 + fm);
      };
    const auto derivative = [](double fm, double f0, double fp, double u) {
        return 0.5 * (fp - fm) + u * (fp - 2.0 * f0 + fm);
      };

    double row_value[3];
    double row_dx[3];
    for (int row = -1; row <= 1; ++row) {
      const double fm = d[idx(cx - 1, cy + row)];
      const double f0 = d[idx(cx, cy + row)];
      const double fp = d[idx(cx + 1, cy + row)];
      row_value[row + 1] = quadratic(fm, f0, fp, ux);
      row_dx[row + 1] = derivative(fm, f0, fp, ux);
    }

    dist = quadratic(row_value[0], row_value[1], row_value[2], uy);
    const double dx_grid = quadratic(row_dx[0], row_dx[1], row_dx[2], uy);
    const double dy_grid = derivative(row_value[0], row_value[1], row_value[2], uy);
    grad.x() = dx_grid / field.resolution;
    grad.y() = dy_grid / field.resolution;
    return std::isfinite(dist) && grad.allFinite();
  }

private:
  DistanceFieldRegistry() = default;
  mutable std::mutex mutex_;
  std::shared_ptr<const DistanceFieldSnapshot> latest_;
};

}  // namespace navigation2

#endif  // NAVIGATION2__SHARED_STATE_HPP_

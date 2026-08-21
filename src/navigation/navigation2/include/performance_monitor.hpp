#ifndef NAVIGATION2__PERFORMANCE_MONITOR_HPP_
#define NAVIGATION2__PERFORMANCE_MONITOR_HPP_

// 关断式性能观测：把「每拍耗时 / 频率 / 失败原因」落 CSV 的轻量设施。
//
// 为什么照搬 sentry 的设计、但收敛到 Lucifer 的纪律：
// - sentry 每个热路径模块都有可关断的 CSV（search/opt/solve 耗时 + failure_reason），
//   这比平均耗时更重要 —— 动态避障慢不是「优化不够快」，而是
//   「障碍入图 → ESDF → 重规划开始 → 新轨迹被接受」这条时间线。
// - 但这里不引入独立线程、不引入全局锁聚合。每个使用者（节点）持有一个实例，
//   所有 record() 调用都发生在该节点自己的 MutuallyExclusive 回调组里，天然串行，
//   无需加锁。这与 Lucifer 现有的回调组纪律一致。
//
// 性能纪律（不破坏热路径）：
// - enable=false 时，tick()/stop()/record() 都退化为一个 bool 判断，零分配。
// - CSV 按 csv_flush_every_n 批量 flush，避免逐行同步刷盘成为新的实时性瓶颈。
// - 时间源用 std::chrono::steady_clock（单调时钟），不做 ROS Time 换算。
//
// 用法：
//   PerformanceMonitor pm;
//   pm.configure(PerformanceMonitor::Config{.enable=true, .csv_path=...}, logger);
//   auto t = pm.tick();
//   ... hot path ...
//   pm.stop(t, PerformanceSample{.success=true, .failure_reason="", .extra...});
//   // 析构或 close() 时 flush 剩余行并关闭文件。

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include <rclcpp/logging.hpp>

namespace navigation2
{

class PerformanceMonitor
{
public:
  struct Config
  {
    // 总开关。关闭时所有调用都是空操作。
    bool enable{false};
    // 是否打印窗口摘要到终端（节流）。
    bool print_enable{false};
    // 是否写逐次 CSV。
    bool csv_enable{false};
    // CSV 输出路径。空则用默认值。
    std::string csv_path{};
    // 每 N 次 flush 一次文件。
    int csv_flush_every_n{20};
    // 窗口摘要的节流周期（秒）。
    double print_period_s{5.0};
    // 写入 CSV 每行的附加标识（run_id/scenario），按逗号转义。
    std::vector<std::string> csv_prefix_fields{};
  };

  // 一次观测样本。字段语义由使用者定义，这里只负责序列化。
  struct Sample
  {
    bool success{true};
    std::string failure_reason{};
    // 阶段耗时（毫秒）。使用者自行填充，列头由使用者给出。
    std::vector<double> stage_ms{};
    std::vector<std::string> stage_names{};
    // 附加数值字段（命令/参考速度等），列头由使用者给出。
    std::vector<double> extra{};
    std::vector<std::string> extra_names{};
  };

  PerformanceMonitor() = default;
  ~PerformanceMonitor() { close(); }

  PerformanceMonitor(const PerformanceMonitor &) = delete;
  PerformanceMonitor & operator=(const PerformanceMonitor &) = delete;

  void configure(const Config & config, rclcpp::Logger logger)
  {
    config_ = config;
    logger_ = logger;
    if (config_.enable && config_.csv_enable) {
      openCsv(config_.csv_path);
    }
  }

  bool enabled() const { return config_.enable; }

  // 返回起始时间点。enable=false 时返回一个默认值（不会进 stop()）。
  std::chrono::steady_clock::time_point tick() const
  {
    if (!config_.enable) {
      return {};
    }
    return std::chrono::steady_clock::now();
  }

  // 结束一段观测。sample.stage_ms 由使用者自行填充（调用方在 tick/stop 之间计时）。
  void stop(const std::chrono::steady_clock::time_point & start, const Sample & sample)
  {
    if (!config_.enable || start == std::chrono::steady_clock::time_point{}) {
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    const double cycle_ms =
      std::chrono::duration<double, std::milli>(now - start).count();
    record(cycle_ms, sample);
  }

  void close()
  {
    if (csv_file_) {
      flush();
      std::fclose(csv_file_);
      csv_file_ = nullptr;
    }
  }

private:
  static bool sanitize(const std::string & v) noexcept
  {
    return v.find(',') == std::string::npos && v.find('\n') == std::string::npos &&
           v.find('\r') == std::string::npos;
  }

  void openCsv(const std::string & path)
  {
    const std::string p = path.empty() ? "/tmp/lucifer_perf.csv" : path;
    csv_file_ = std::fopen(p.c_str(), "w");
    if (!csv_file_) {
      RCLCPP_ERROR(logger_, "PerformanceMonitor: cannot open CSV %s", p.c_str());
      return;
    }
    writeHeader();
    // 写头部后立即 flush，避免崩溃时头部也丢失。
    std::fflush(csv_file_);
  }

  void writeHeader()
  {
    if (!csv_file_) {
      return;
    }
    // 固定列头：cycle_ms,success,<prefix...>,stage_ms,failure_reason。
    // stage_ms 列把样本的 stage_ms 用分号拼成单个单元格，避免流式 CSV 的动态列头
    // 无法回写（文件是顺序追加的，列头只能写一次）。
    std::fprintf(csv_file_, "cycle_ms,success");
    for (const auto & f : config_.csv_prefix_fields) {
      if (sanitize(f)) {
        std::fprintf(csv_file_, ",%s", f.c_str());
      }
    }
    std::fprintf(csv_file_, ",stage_ms,failure_reason\n");
  }

  void record(double cycle_ms, const Sample & sample)
  {
    if (csv_file_) {
      std::fprintf(csv_file_, "%.3f,%d", cycle_ms, sample.success ? 1 : 0);
      for (const auto & f : config_.csv_prefix_fields) {
        std::fprintf(csv_file_, ",%s", sanitize(f) ? f.c_str() : "_");
      }
      if (!sample.stage_ms.empty()) {
        std::fprintf(csv_file_, ",");
        for (size_t i = 0; i < sample.stage_ms.size(); ++i) {
          if (i) {
            std::fprintf(csv_file_, ";");
          }
          std::fprintf(csv_file_, "%.3f", sample.stage_ms[i]);
        }
      } else {
        std::fprintf(csv_file_, ",");
      }
      std::fprintf(csv_file_, ",%s", sanitize(sample.failure_reason) ? sample.failure_reason.c_str() : "_");
      std::fprintf(csv_file_, "\n");
      if (++csv_rows_since_flush_ >= config_.csv_flush_every_n) {
        flush();
      }
    }

    if (config_.print_enable) {
      maybePrint(sample);
    }
  }

  void maybePrint(const Sample & sample)
  {
    const auto now = std::chrono::steady_clock::now();
    const double since = std::chrono::duration<double>(now - last_print_).count();
    if (since < config_.print_period_s) {
      return;
    }
    last_print_ = now;
    if (sample.success) {
      RCLCPP_INFO(logger_, "perf: success");
    } else {
      RCLCPP_WARN(logger_, "perf: fail(%s)", sample.failure_reason.c_str());
    }
  }

  void flush()
  {
    if (csv_file_) {
      std::fflush(csv_file_);
      csv_rows_since_flush_ = 0;
    }
  }

  Config config_;
  rclcpp::Logger logger_{rclcpp::get_logger("PerformanceMonitor")};
  std::FILE * csv_file_{nullptr};
  int csv_rows_since_flush_{0};
  std::chrono::steady_clock::time_point last_print_{};
};

}  // namespace navigation2

#endif  // NAVIGATION2__PERFORMANCE_MONITOR_HPP_

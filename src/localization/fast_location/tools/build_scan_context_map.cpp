// 离线 Scan Context 建库工具：把先验 PCD 按平面网格采成一批传感器位置描述子，
// 存成 fast_location::ScanContextDatabase 的 versioned 二进制（.scdb）。
//
// 用法：
//   build_scan_context_map --input map.pcd --output map.scdb
//     [--step 1.0] [--max-radius 30] [--rings 30] [--sectors 90]
//     [--map-voxel 0.2] [--map-sensor-z 0] [--min-range 1.0]
//
// 注意：.scdb 与建库参数（rings/sectors/max_radius/map_sensor_z 等）强绑定，
// 加载端必须用同一套配置建查询描述子，否则距离没有意义（config 存在文件头里）。

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <string>
#include <vector>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "fast_location/scan_context.hpp"

namespace
{

using PointCloud = pcl::PointCloud<pcl::PointXYZI>;

// 与 scan_context.cpp 里 build() 的条目内存上限保持一致（约 256MB 描述子）。
constexpr std::size_t kMaxDescriptorFloats = std::size_t{64} << 20;

struct Options
{
  std::string input;
  std::string output;
  fast_location::ScanContextConfig config;
};

void printUsage(const char * program)
{
  std::printf(
    "Usage: %s --input map.pcd --output map.scdb [options]\n"
    "\n"
    "Options:\n"
    "  --input <path>         先验点云 PCD（必填）\n"
    "  --output <path>        输出 .scdb（必填）\n"
    "  --step <m>             XY 网格步长（默认 1.0）\n"
    "  --max-radius <m>       描述子最大半径（默认 30）\n"
    "  --rings <n>            环数（默认 30）\n"
    "  --sectors <n>          扇区数（默认 90）\n"
    "  --map-voxel <m>        建库前地图体素降采样（默认 0.2，<=0 关闭）\n"
    "  --map-sensor-z <m>     地图 z 原点相对传感器的偏移（默认 0）\n"
    "  --min-range <m>        描述子最小半径（默认 1.0）\n"
    "  --help                 打印本帮助\n",
    program);
}

bool parseFloat(const char * text, float & value)
{
  if (text == nullptr || *text == '\0') {
    return false;
  }
  char * end = nullptr;
  errno = 0;
  const float parsed = std::strtof(text, &end);
  if (end == text || *end != '\0' || errno == ERANGE || !std::isfinite(parsed)) {
    return false;
  }
  value = parsed;
  return true;
}

bool parseInt(const char * text, int & value)
{
  if (text == nullptr || *text == '\0') {
    return false;
  }
  char * end = nullptr;
  errno = 0;
  const long parsed = std::strtol(text, &end, 10);
  if (end == text || *end != '\0' || errno == ERANGE ||
    parsed < std::numeric_limits<int>::min() || parsed > std::numeric_limits<int>::max())
  {
    return false;
  }
  value = static_cast<int>(parsed);
  return true;
}

// 取 "--flag value" 里的 value；缺少时返回 false 并打印错误。
bool takeValue(int argc, char ** argv, int & index, std::string & value)
{
  if (index + 1 >= argc) {
    std::fprintf(stderr, "missing value for %s\n", argv[index]);
    return false;
  }
  value = argv[++index];
  return true;
}

// 解析失败返回 false；--help 时 help=true 且返回 true。
bool parseOptions(int argc, char ** argv, Options & options, bool & help)
{
  help = false;
  for (int index = 1; index < argc; ++index) {
    const std::string arg = argv[index];
    if (arg == "--help" || arg == "-h") {
      help = true;
      return true;
    }
    std::string value;
    if (arg == "--input") {
      if (!takeValue(argc, argv, index, value)) {
        return false;
      }
      options.input = value;
    } else if (arg == "--output") {
      if (!takeValue(argc, argv, index, value)) {
        return false;
      }
      options.output = value;
    } else if (arg == "--step") {
      if (!takeValue(argc, argv, index, value) || !parseFloat(value.c_str(), options.config.grid_step)) {
        std::fprintf(stderr, "invalid --step: %s\n", value.c_str());
        return false;
      }
    } else if (arg == "--max-radius") {
      if (!takeValue(argc, argv, index, value) ||
        !parseFloat(value.c_str(), options.config.max_radius))
      {
        std::fprintf(stderr, "invalid --max-radius: %s\n", value.c_str());
        return false;
      }
    } else if (arg == "--rings") {
      if (!takeValue(argc, argv, index, value) || !parseInt(value.c_str(), options.config.rings)) {
        std::fprintf(stderr, "invalid --rings: %s\n", value.c_str());
        return false;
      }
    } else if (arg == "--sectors") {
      if (!takeValue(argc, argv, index, value) ||
        !parseInt(value.c_str(), options.config.sectors))
      {
        std::fprintf(stderr, "invalid --sectors: %s\n", value.c_str());
        return false;
      }
    } else if (arg == "--map-voxel") {
      if (!takeValue(argc, argv, index, value) ||
        !parseFloat(value.c_str(), options.config.map_voxel))
      {
        std::fprintf(stderr, "invalid --map-voxel: %s\n", value.c_str());
        return false;
      }
    } else if (arg == "--map-sensor-z") {
      if (!takeValue(argc, argv, index, value) ||
        !parseFloat(value.c_str(), options.config.map_sensor_z))
      {
        std::fprintf(stderr, "invalid --map-sensor-z: %s\n", value.c_str());
        return false;
      }
    } else if (arg == "--min-range") {
      if (!takeValue(argc, argv, index, value) ||
        !parseFloat(value.c_str(), options.config.min_range))
      {
        std::fprintf(stderr, "invalid --min-range: %s\n", value.c_str());
        return false;
      }
    } else {
      std::fprintf(stderr, "unknown option: %s\n", arg.c_str());
      return false;
    }
  }
  return true;
}

}  // namespace

int main(int argc, char ** argv)
{
  Options options;
  bool help = false;
  if (!parseOptions(argc, argv, options, help)) {
    printUsage(argv[0]);
    return 2;
  }
  if (help) {
    printUsage(argv[0]);
    return 0;
  }
  if (options.input.empty() || options.output.empty()) {
    std::fprintf(stderr, "--input and --output are required\n");
    printUsage(argv[0]);
    return 2;
  }

  const auto & config = options.config;
  const std::size_t cells =
    static_cast<std::size_t>(config.rings) * static_cast<std::size_t>(config.sectors);
  if (cells == 0) {
    std::fprintf(stderr, "--rings/--sectors must be positive\n");
    return 2;
  }
  const std::size_t max_entries = std::max<std::size_t>(1, kMaxDescriptorFloats / cells);

  // 1) 读先验 PCD（PointXYZI 容器；纯 xyz 的 small_glim PCD 也能读）。
  PointCloud raw_map;
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(options.input, raw_map) != 0) {
    std::fprintf(stderr, "failed to read PCD: %s\n", options.input.c_str());
    return 1;
  }
  if (raw_map.empty()) {
    std::fprintf(stderr, "PCD has no points: %s\n", options.input.c_str());
    return 1;
  }

  // 2) 体素降采样（建库只关心几何轮廓；map_voxel<=0 时关闭）。
  PointCloud map;
  if (config.map_voxel > 0.0f) {
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
    voxel_filter.setInputCloud(raw_map.makeShared());
    voxel_filter.setLeafSize(config.map_voxel, config.map_voxel, config.map_voxel);
    voxel_filter.filter(map);
  } else {
    map = raw_map;
  }
  if (map.empty()) {
    std::fprintf(stderr, "map is empty after voxel downsampling\n");
    return 1;
  }

  // 3) 平面包围盒 → 网格采样位置（与 ScanContextDatabase::build 的约定一致）。
  float min_x = std::numeric_limits<float>::infinity();
  float max_x = -std::numeric_limits<float>::infinity();
  float min_y = std::numeric_limits<float>::infinity();
  float max_y = -std::numeric_limits<float>::infinity();
  for (const auto & point : map.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      continue;
    }
    min_x = std::min(min_x, point.x);
    max_x = std::max(max_x, point.x);
    min_y = std::min(min_y, point.y);
    max_y = std::max(max_y, point.y);
  }
  if (!std::isfinite(min_x) || max_x < min_x || max_y < min_y) {
    std::fprintf(stderr, "map has no finite planar points\n");
    return 1;
  }

  const double nx = std::floor((max_x - min_x) / static_cast<double>(config.grid_step)) + 1.0;
  const double ny = std::floor((max_y - min_y) / static_cast<double>(config.grid_step)) + 1.0;
  const double position_count = nx * ny;
  if (!(position_count >= 1.0)) {
    std::fprintf(stderr, "--step is invalid\n");
    return 2;
  }
  if (position_count > static_cast<double>(max_entries)) {
    std::fprintf(
      stderr,
      "grid would create %.0f positions (> %zu safety limit); "
      "increase --step (e.g. %.1f) or reduce --rings/--sectors\n",
      position_count, max_entries,
      static_cast<double>(config.grid_step) * std::sqrt(
        position_count / static_cast<double>(max_entries)));
    return 1;
  }

  std::printf(
    "input=%s raw_points=%zu downsampled_points=%zu\n"
    "descriptor: %d rings x %d sectors, max_radius=%.1f min_range=%.2f "
    "map_sensor_z=%.2f map_voxel=%.2f\n"
    "grid: step=%.2f x=[%.2f, %.2f] y=[%.2f, %.2f] positions=%zu\n",
    options.input.c_str(), raw_map.size(), map.size(),
    config.rings, config.sectors, config.max_radius, config.min_range,
    config.map_sensor_z, config.map_voxel, config.grid_step,
    min_x, max_x, min_y, max_y, static_cast<std::size_t>(position_count));

  // 4) 逐位置建描述子；prepare 只做一次降采样/索引准备，每个位置单独计时。
  fast_location::ScanContextDatabase database;
  if (!database.prepare(map, config)) {
    std::fprintf(stderr, "failed to prepare Scan Context database (invalid config?)\n");
    return 1;
  }

  const auto build_start = std::chrono::steady_clock::now();
  const float epsilon = config.grid_step * 1e-4f;
  std::size_t position_index = 0;
  std::size_t skipped = 0;
  for (float x = min_x; x <= max_x + epsilon; x += config.grid_step) {
    for (float y = min_y; y <= max_y + epsilon; y += config.grid_step) {
      const auto position_start = std::chrono::steady_clock::now();
      const bool added = database.addAtPosition(Eigen::Vector2f(x, y));
      const auto position_end = std::chrono::steady_clock::now();
      const double elapsed_ms =
        std::chrono::duration<double, std::milli>(position_end - position_start).count();
      ++position_index;
      if (added) {
        std::printf(
          "[%6zu/%6zu] pos=(%7.2f, %7.2f) %8.3f ms entries=%zu\n",
          position_index, static_cast<std::size_t>(position_count), x, y, elapsed_ms,
          database.size());
      } else {
        ++skipped;
        std::printf(
          "[%6zu/%6zu] pos=(%7.2f, %7.2f) %8.3f ms skipped (no points in radius)\n",
          position_index, static_cast<std::size_t>(position_count), x, y, elapsed_ms);
      }
      std::fflush(stdout);
    }
  }
  const auto build_end = std::chrono::steady_clock::now();
  const double total_seconds =
    std::chrono::duration<double>(build_end - build_start).count();

  if (database.empty()) {
    std::fprintf(stderr, "no descriptor built: check map/extrinsics/--max-radius\n");
    return 1;
  }

  // 5) 保存 versioned 二进制。
  if (!database.save(options.output)) {
    std::fprintf(stderr, "failed to write database: %s\n", options.output.c_str());
    return 1;
  }

  std::printf(
    "done: entries=%zu skipped=%zu elapsed=%.3f s (%.2f ms/entry) -> %s\n",
    database.size(), skipped, total_seconds,
    1000.0 * total_seconds / static_cast<double>(database.size()),
    options.output.c_str());
  return 0;
}

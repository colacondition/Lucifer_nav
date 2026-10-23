#include "fast_location/scan_context.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <unordered_map>
#include <utility>

namespace fast_location
{

// 降采样后的地图 + 2D 邻域索引。cell_size 取 max(1, max_radius/8)，查询时扫过覆盖
// 圆形包围盒的格子；每个位置只碰附近点，避免每次建库条目都全图遍历。
// 定义在 .cpp 里：头文件只前置声明，公开 API 不暴露实现类型。
struct ScanContextPreparedMap
{
  std::vector<Eigen::Vector3f> points;
  std::unordered_map<std::int64_t, std::vector<int>> cells;
  float cell_size{1.0f};
};

namespace
{

constexpr float kPi = 3.14159265358979323846f;

// 二进制格式：magic + version，加载时严格比对。
constexpr char kMagic[8] = {'L', 'S', 'C', 'D', 'B', '0', '0', '1'};
constexpr std::uint32_t kFormatVersion = 1;

// 描述子几何上限：rings × sectors 过大等于单条目几 MB，一次查询/建库就会失控。
constexpr int kMaxRings = 4096;
constexpr int kMaxSectors = 4096;
constexpr std::size_t kMaxDescriptorCells = std::size_t{1} << 20;  // 1Mi 格 ≈ 4MB/条目

// 单库描述子 float 总量上限（约 256MB）：build() 的网格位置数超过它就自动加大
// grid_step，避免一块大地图 + 1m 网格把内存吃光。
constexpr std::size_t kMaxDescriptorFloats = std::size_t{64} << 20;

// load() 时允许的最大数据量，防止损坏/恶意文件让进程直接分配天量内存。
constexpr std::size_t kMaxLoadFloats = std::size_t{256} << 20;
constexpr std::size_t kMaxEntriesPerFile = std::size_t{1} << 24;

// 2D 邻域索引的格子坐标偏移，保证坐标非负后能拼进一个 int64。
constexpr std::int64_t kCellBias = std::int64_t{1} << 30;

bool isFinitePoint(const pcl::PointXYZI & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

// 把角度压到 [-pi, pi]。
float normalizeAngle(float angle)
{
  while (angle > kPi) {
    angle -= 2.0f * kPi;
  }
  while (angle <= -kPi) {
    angle += 2.0f * kPi;
  }
  return angle;
}

std::size_t descriptorCells(const ScanContextConfig & config)
{
  return static_cast<std::size_t>(config.rings) * static_cast<std::size_t>(config.sectors);
}

// 校验并规范化配置；非法配置返回 false（不静默接受会造成除零/巨量内存）。
bool validateConfig(ScanContextConfig & config)
{
  if (config.rings < 1 || config.rings > kMaxRings) {
    return false;
  }
  if (config.sectors < 1 || config.sectors > kMaxSectors) {
    return false;
  }
  if (descriptorCells(config) > kMaxDescriptorCells) {
    return false;
  }
  if (!std::isfinite(config.max_radius) || !(config.max_radius > 0.0f)) {
    return false;
  }
  if (!std::isfinite(config.min_range) || config.min_range < 0.0f) {
    return false;
  }
  // 必须留出有效环带；min_range == max_radius 等于什么都编码不到。
  if (!(config.min_range < config.max_radius)) {
    return false;
  }
  if (!std::isfinite(config.map_sensor_z) || !std::isfinite(config.query_sensor_z)) {
    return false;
  }
  if (!std::isfinite(config.grid_step) || !(config.grid_step > 0.0f)) {
    return false;
  }
  if (!std::isfinite(config.map_voxel) || config.map_voxel < 0.0f) {
    return false;
  }
  if (!std::isfinite(config.distance_threshold) || config.distance_threshold < 0.0f) {
    return false;
  }
  if (config.sector_key_candidates < 1) {
    config.sector_key_candidates = 1;
  }
  return true;
}

// 描述子缓冲区：image 初值 -inf，便于逐点取 max z；收尾时把没点的格子归 0。
struct DescriptorBuffer
{
  std::vector<float> image;
  std::vector<float> sector_key;
};

// 把一个相对传感器的点累加进极坐标高度图（ring = 水平距离，sector = 方位角）。
// 返回该点是否落在 [min_range, max_radius] 有效环带内。
bool accumulatePoint(
  float x, float y, float z, const ScanContextConfig & config, std::vector<float> & image)
{
  const float radius = std::hypot(x, y);
  if (!(radius >= config.min_range) || radius > config.max_radius) {
    return false;
  }

  const float ring_gap = config.max_radius / static_cast<float>(config.rings);
  const float sector_gap = 2.0f * kPi / static_cast<float>(config.sectors);
  int ring = static_cast<int>(std::floor(radius / ring_gap));
  ring = std::clamp(ring, 0, config.rings - 1);

  float theta = std::atan2(y, x);
  if (theta < 0.0f) {
    theta += 2.0f * kPi;
  }
  int sector = static_cast<int>(std::floor(theta / sector_gap));
  sector = std::clamp(sector, 0, config.sectors - 1);

  const std::size_t index =
    static_cast<std::size_t>(ring) * static_cast<std::size_t>(config.sectors) +
    static_cast<std::size_t>(sector);
  image[index] = std::max(image[index], z);
  return true;
}

// 空格子归 0 + 计算每扇区跨环平均（sector key）。sector key 是查询前的粗排特征：
// 它比完整描述子便宜一个环数，且 yaw 只体现为循环列移。
void finalizeDescriptor(const ScanContextConfig & config, DescriptorBuffer & descriptor)
{
  const std::size_t cells = descriptorCells(config);
  for (std::size_t index = 0; index < cells; ++index) {
    if (!std::isfinite(descriptor.image[index])) {
      descriptor.image[index] = 0.0f;
    }
  }

  const int sectors = config.sectors;
  const int rings = config.rings;
  descriptor.sector_key.assign(static_cast<std::size_t>(sectors), 0.0f);
  for (int sector = 0; sector < sectors; ++sector) {
    double sum = 0.0;
    for (int ring = 0; ring < rings; ++ring) {
      sum += descriptor.image[static_cast<std::size_t>(ring) * sectors + sector];
    }
    descriptor.sector_key[sector] = static_cast<float>(sum / rings);
  }
}

// 用 base 系点云（已相对传感器）构造查询描述子。
bool makeQueryDescriptor(
  const pcl::PointCloud<pcl::PointXYZI> & base_cloud,
  const ScanContextConfig & config,
  DescriptorBuffer & descriptor)
{
  descriptor.image.assign(descriptorCells(config), -std::numeric_limits<float>::infinity());
  descriptor.sector_key.clear();

  bool any = false;
  for (const auto & point : base_cloud.points) {
    if (!isFinitePoint(point)) {
      continue;
    }
    any = accumulatePoint(
      point.x, point.y, point.z - config.query_sensor_z, config, descriptor.image) || any;
  }
  if (!any) {
    return false;
  }
  finalizeDescriptor(config, descriptor);
  return true;
}

// 2D 平面格子坐标 → int64 key（两维各 31 位，带偏移避免负数）。
std::int64_t packCell(std::int64_t ix, std::int64_t iy)
{
  const std::int64_t px = std::clamp<std::int64_t>(ix, -kCellBias, kCellBias - 1) + kCellBias;
  const std::int64_t py = std::clamp<std::int64_t>(iy, -kCellBias, kCellBias - 1) + kCellBias;
  return (px << 31) | py;
}

// 3D 体素坐标 → int64 key；每维 21 位，换算成米在 0.2m 体素下覆盖 ±200km。
constexpr std::int64_t kVoxelBias = std::int64_t{1} << 20;
std::int64_t packVoxel(std::int64_t ix, std::int64_t iy, std::int64_t iz)
{
  const auto clampIndex = [](std::int64_t value) {
    return std::clamp<std::int64_t>(value, -kVoxelBias, kVoxelBias - 1) + kVoxelBias;
  };
  return (clampIndex(ix) << 42) | (clampIndex(iy) << 21) | clampIndex(iz);
}

// 用 map_radius 内的地图点构造某个采样位置的描述子。
bool makeMapDescriptor(
  const ScanContextPreparedMap & prepared,
  const Eigen::Vector2f & origin,
  const ScanContextConfig & config,
  DescriptorBuffer & descriptor)
{
  descriptor.image.assign(descriptorCells(config), -std::numeric_limits<float>::infinity());
  descriptor.sector_key.clear();

  const float radius = config.max_radius;
  const float cell_size = prepared.cell_size;
  const std::int64_t min_ix =
    static_cast<std::int64_t>(std::floor((origin.x() - radius) / cell_size));
  const std::int64_t max_ix =
    static_cast<std::int64_t>(std::floor((origin.x() + radius) / cell_size));
  const std::int64_t min_iy =
    static_cast<std::int64_t>(std::floor((origin.y() - radius) / cell_size));
  const std::int64_t max_iy =
    static_cast<std::int64_t>(std::floor((origin.y() + radius) / cell_size));

  bool any = false;
  for (std::int64_t ix = min_ix; ix <= max_ix; ++ix) {
    for (std::int64_t iy = min_iy; iy <= max_iy; ++iy) {
      const auto found = prepared.cells.find(packCell(ix, iy));
      if (found == prepared.cells.end()) {
        continue;
      }
      for (const int point_index : found->second) {
        const Eigen::Vector3f & point = prepared.points[static_cast<std::size_t>(point_index)];
        any = accumulatePoint(
          point.x() - origin.x(), point.y() - origin.y(),
          point.z() - config.map_sensor_z, config, descriptor.image) || any;
      }
    }
  }

  if (!any) {
    return false;
  }
  finalizeDescriptor(config, descriptor);
  return true;
}

// 两个 sector key 在循环列移下的最大余弦相似度（查询粗排用）。没有可对齐的有效
// 扇区时返回 -inf，排序时自然排到最后。
float sectorKeySimilarity(
  const std::vector<float> & query_key, const std::vector<float> & candidate_key, int sectors)
{
  const std::size_t count = static_cast<std::size_t>(sectors);
  if (query_key.size() != count || candidate_key.size() != count) {
    return -std::numeric_limits<float>::infinity();
  }

  std::vector<float> query_norm(count, 0.0f);
  std::vector<float> candidate_norm(count, 0.0f);
  for (std::size_t sector = 0; sector < count; ++sector) {
    const double q = query_key[sector];
    const double c = candidate_key[sector];
    query_norm[sector] = static_cast<float>(std::sqrt(q * q));
    candidate_norm[sector] = static_cast<float>(std::sqrt(c * c));
  }

  float best_similarity = -std::numeric_limits<float>::infinity();
  for (int shift = 0; shift < sectors; ++shift) {
    double sum = 0.0;
    int engaged = 0;
    for (int sector = 0; sector < sectors; ++sector) {
      int query_sector = sector + shift;
      if (query_sector >= sectors) {
        query_sector -= sectors;
      }
      if (!(query_norm[static_cast<std::size_t>(query_sector)] > 0.0f) ||
        !(candidate_norm[static_cast<std::size_t>(sector)] > 0.0f))
      {
        continue;
      }
      sum += static_cast<double>(query_key[static_cast<std::size_t>(query_sector)]) *
        static_cast<double>(candidate_key[static_cast<std::size_t>(sector)]) /
        (static_cast<double>(query_norm[static_cast<std::size_t>(query_sector)]) *
        static_cast<double>(candidate_norm[static_cast<std::size_t>(sector)]));
      ++engaged;
    }
    if (engaged == 0) {
      continue;
    }
    best_similarity = std::max(
      best_similarity, static_cast<float>(sum / static_cast<double>(engaged)));
  }
  return best_similarity;
}

struct DescriptorMatch
{
  float distance{std::numeric_limits<float>::max()};
  int shift{0};
};

// 完整描述子距离：对每个循环列移，算有效列（两侧都有点）的余弦相似度均值，
// 取最优列移。距离 = 1 - 平均余弦 ∈ [0, 2]，越小越像。
//
// 列移符号约定（yaw 的符号就是从这里来的）：
//   query 列 (sector + shift) 对齐 candidate 列 sector。
//   地图点在地图系角度 θ，查询传感器朝向 ψ（map_from_base 的 yaw），同一点在 base
//   系角度为 θ - ψ，因此查询列 = 地图列 - ψ/(2π/sectors)。令 query = candidate + shift：
//     shift = -ψ/(2π/sectors)  ⇒  ψ = -shift * 2π/sectors。
// 即 shift 是"把查询图转多少列才能对齐地图图"的相反数。
bool matchDescriptor(
  const DescriptorBuffer & query,
  const std::vector<float> & candidate_image,
  const ScanContextConfig & config,
  DescriptorMatch & match)
{
  const int rings = config.rings;
  const int sectors = config.sectors;
  const std::size_t cells = descriptorCells(config);
  if (query.image.size() != cells || candidate_image.size() != cells) {
    return false;
  }

  // 每扇区列向量的 L2 范数，列移不改变它，只算一次。
  std::vector<float> query_norm(static_cast<std::size_t>(sectors), 0.0f);
  std::vector<float> candidate_norm(static_cast<std::size_t>(sectors), 0.0f);
  for (int sector = 0; sector < sectors; ++sector) {
    double query_sum = 0.0;
    double candidate_sum = 0.0;
    for (int ring = 0; ring < rings; ++ring) {
      const std::size_t index = static_cast<std::size_t>(ring) * sectors + sector;
      const double query_value = query.image[index];
      const double candidate_value = candidate_image[index];
      query_sum += query_value * query_value;
      candidate_sum += candidate_value * candidate_value;
    }
    query_norm[static_cast<std::size_t>(sector)] =
      static_cast<float>(std::sqrt(query_sum));
    candidate_norm[static_cast<std::size_t>(sector)] =
      static_cast<float>(std::sqrt(candidate_sum));
  }

  float best_distance = std::numeric_limits<float>::max();
  int best_shift = 0;
  for (int shift = 0; shift < sectors; ++shift) {
    double similarity_sum = 0.0;
    int engaged = 0;
    for (int sector = 0; sector < sectors; ++sector) {
      int query_sector = sector + shift;
      if (query_sector >= sectors) {
        query_sector -= sectors;
      }
      const float query_norm_value = query_norm[static_cast<std::size_t>(query_sector)];
      const float candidate_norm_value = candidate_norm[static_cast<std::size_t>(sector)];
      // 空列（该扇区两侧任一侧完全没点）不参与平均，否则空地会淹没几何差异。
      if (!(query_norm_value > 0.0f) || !(candidate_norm_value > 0.0f)) {
        continue;
      }
      double dot = 0.0;
      for (int ring = 0; ring < rings; ++ring) {
        const std::size_t query_index =
          static_cast<std::size_t>(ring) * sectors + query_sector;
        const std::size_t candidate_index =
          static_cast<std::size_t>(ring) * sectors + sector;
        dot += static_cast<double>(query.image[query_index]) *
          static_cast<double>(candidate_image[candidate_index]);
      }
      similarity_sum += dot /
        (static_cast<double>(query_norm_value) *
        static_cast<double>(candidate_norm_value));
      ++engaged;
    }
    if (engaged == 0) {
      continue;
    }
    const float distance = static_cast<float>(1.0 - similarity_sum / engaged);
    // 用严格小于：完全并列（对称场景）时取最小列移，保证结果确定。
    if (distance < best_distance) {
      best_distance = distance;
      best_shift = shift;
    }
  }

  if (!std::isfinite(best_distance)) {
    return false;
  }
  match.distance = std::clamp(best_distance, 0.0f, 2.0f);
  match.shift = best_shift;
  return true;
}

// 列移 → 传感器在 map 系的 yaw（符号推导见 matchDescriptor 上方注释）。
float yawFromShift(int shift, int sectors)
{
  return normalizeAngle(
    -static_cast<float>(shift) * 2.0f * kPi / static_cast<float>(sectors));
}

template<typename T>
bool writePod(std::ostream & stream, const T & value)
{
  stream.write(reinterpret_cast<const char *>(&value), sizeof(T));
  return static_cast<bool>(stream);
}

template<typename T>
bool readPod(std::istream & stream, T & value)
{
  stream.read(reinterpret_cast<char *>(&value), sizeof(T));
  return static_cast<bool>(stream);
}

bool writeConfig(std::ostream & stream, const ScanContextConfig & config)
{
  return writePod(stream, config.rings) &&
         writePod(stream, config.sectors) &&
         writePod(stream, config.max_radius) &&
         writePod(stream, config.min_range) &&
         writePod(stream, config.map_sensor_z) &&
         writePod(stream, config.query_sensor_z) &&
         writePod(stream, config.grid_step) &&
         writePod(stream, config.map_voxel) &&
         writePod(stream, config.distance_threshold) &&
         writePod(stream, static_cast<std::uint32_t>(config.sector_key_candidates));
}

bool readConfig(std::istream & stream, ScanContextConfig & config)
{
  std::uint32_t key_candidates = 0;
  if (!(readPod(stream, config.rings) &&
    readPod(stream, config.sectors) &&
    readPod(stream, config.max_radius) &&
    readPod(stream, config.min_range) &&
    readPod(stream, config.map_sensor_z) &&
    readPod(stream, config.query_sensor_z) &&
    readPod(stream, config.grid_step) &&
    readPod(stream, config.map_voxel) &&
    readPod(stream, config.distance_threshold) &&
    readPod(stream, key_candidates)))
  {
    return false;
  }
  config.sector_key_candidates = static_cast<std::size_t>(key_candidates);
  return true;
}

}  // namespace

namespace
{

std::shared_ptr<ScanContextPreparedMap> prepareMap(
  const pcl::PointCloud<pcl::PointXYZI> & map, const ScanContextConfig & config)
{
  auto prepared = std::make_shared<ScanContextPreparedMap>();

  // 1) 体素降采样：每个体素取质心（与 PCL VoxelGrid 语义一致），只保留有限点。
  if (config.map_voxel > 0.0f) {
    struct VoxelAccumulator
    {
      Eigen::Vector3d sum{Eigen::Vector3d::Zero()};
      std::uint32_t count{0};
    };
    std::unordered_map<std::int64_t, VoxelAccumulator> voxels;
    voxels.reserve(map.size() / 2 + 1);
    for (const auto & point : map.points) {
      if (!isFinitePoint(point)) {
        continue;
      }
      const std::int64_t ix =
        static_cast<std::int64_t>(std::floor(point.x / config.map_voxel));
      const std::int64_t iy =
        static_cast<std::int64_t>(std::floor(point.y / config.map_voxel));
      const std::int64_t iz =
        static_cast<std::int64_t>(std::floor(point.z / config.map_voxel));
      VoxelAccumulator & accumulator = voxels[packVoxel(ix, iy, iz)];
      accumulator.sum += Eigen::Vector3d(point.x, point.y, point.z);
      ++accumulator.count;
    }
    prepared->points.reserve(voxels.size());
    for (const auto & [key, accumulator] : voxels) {
      (void)key;
      if (accumulator.count == 0) {
        continue;
      }
      const Eigen::Vector3d center =
        accumulator.sum / static_cast<double>(accumulator.count);
      prepared->points.emplace_back(center.cast<float>());
    }
  } else {
    prepared->points.reserve(map.size());
    for (const auto & point : map.points) {
      if (!isFinitePoint(point)) {
        continue;
      }
      prepared->points.emplace_back(point.x, point.y, point.z);
    }
  }

  if (prepared->points.empty()) {
    return nullptr;
  }

  // 2) 2D 邻域索引：格边长按 max_radius 缩放，使每次查询的扫描格子数有界（≈17×17）。
  prepared->cell_size = std::max(1.0f, config.max_radius / 8.0f);
  for (std::size_t index = 0; index < prepared->points.size(); ++index) {
    const Eigen::Vector3f & point = prepared->points[index];
    const std::int64_t ix =
      static_cast<std::int64_t>(std::floor(point.x() / prepared->cell_size));
    const std::int64_t iy =
      static_cast<std::int64_t>(std::floor(point.y() / prepared->cell_size));
    prepared->cells[packCell(ix, iy)].push_back(static_cast<int>(index));
  }
  return prepared;
}

}  // namespace

bool ScanContextDatabase::build(
  const pcl::PointCloud<pcl::PointXYZI> & map, const ScanContextConfig & config)
{
  if (!prepare(map, config)) {
    return false;
  }

  // 用降采样后的点云求平面包围盒，再按 grid_step 撒采样位置。
  float min_x = std::numeric_limits<float>::infinity();
  float max_x = -std::numeric_limits<float>::infinity();
  float min_y = std::numeric_limits<float>::infinity();
  float max_y = -std::numeric_limits<float>::infinity();
  for (const auto & point : prepared_map_->points) {
    min_x = std::min(min_x, point.x());
    max_x = std::max(max_x, point.x());
    min_y = std::min(min_y, point.y());
    max_y = std::max(max_y, point.y());
  }
  if (!std::isfinite(min_x) || max_x < min_x || max_y < min_y) {
    releasePreparedMap();
    clearEntries();
    configured_ = false;
    return false;
  }

  // 条目数上限保护：大地图 + 细网格会让描述子占用失控，超限时自动加大 grid_step
  // （与 global_search 的候选数上限保护同一思路），牺牲分辨率保住内存。
  const std::size_t cells = descriptorCells(config_);
  const std::size_t max_entries = std::max<std::size_t>(1, kMaxDescriptorFloats / cells);
  const auto estimated_positions = [&](float step) {
    const double nx = std::floor((max_x - min_x) / step) + 1.0;
    const double ny = std::floor((max_y - min_y) / step) + 1.0;
    return nx * ny;
  };
  float step = config_.grid_step;
  while (estimated_positions(step) > static_cast<double>(max_entries)) {
    const double scale = std::sqrt(
      estimated_positions(step) / static_cast<double>(max_entries));
    const float grown = step * static_cast<float>(std::max(scale, 1.05));
    if (!(grown > step) || !std::isfinite(grown)) {
      break;  // 浮点饱和兜底，避免死循环。
    }
    step = grown;
  }

  const float epsilon = step * 1e-4f;
  bool any = false;
  for (float x = min_x; x <= max_x + epsilon; x += step) {
    for (float y = min_y; y <= max_y + epsilon; y += step) {
      any = addAtPosition(Eigen::Vector2f(x, y)) || any;
    }
  }

  releasePreparedMap();
  if (!any) {
    clearEntries();
    configured_ = false;
    return false;
  }
  return true;
}

bool ScanContextDatabase::buildAtPositions(
  const pcl::PointCloud<pcl::PointXYZI> & map,
  const ScanContextConfig & config,
  const std::vector<Eigen::Vector2f> & positions)
{
  if (!prepare(map, config)) {
    return false;
  }

  bool any = false;
  for (const auto & position : positions) {
    any = addAtPosition(position) || any;
  }

  releasePreparedMap();
  if (!any) {
    clearEntries();
    configured_ = false;
    return false;
  }
  return true;
}

bool ScanContextDatabase::prepare(
  const pcl::PointCloud<pcl::PointXYZI> & map, const ScanContextConfig & config)
{
  clearEntries();
  releasePreparedMap();
  configured_ = false;

  ScanContextConfig normalized = config;
  if (!validateConfig(normalized)) {
    return false;
  }
  auto prepared = prepareMap(map, normalized);
  if (!prepared) {
    return false;
  }

  config_ = normalized;
  configured_ = true;
  prepared_map_ = std::move(prepared);
  return true;
}

bool ScanContextDatabase::addAtPosition(const Eigen::Vector2f & position)
{
  if (!configured_ || !prepared_map_ || !position.allFinite()) {
    return false;
  }

  DescriptorBuffer descriptor;
  if (!makeMapDescriptor(*prepared_map_, position, config_, descriptor)) {
    return false;
  }

  Entry entry;
  entry.position = position;
  entry.image = std::move(descriptor.image);
  entry.sector_key = std::move(descriptor.sector_key);
  entries_.push_back(std::move(entry));
  return true;
}

bool ScanContextDatabase::save(const std::string & path) const
{
  if (path.empty() ||
    config_.sector_key_candidates >
    static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max()) ||
    entries_.size() > static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max()))
  {
    return false;
  }
  ScanContextConfig validated_config = config_;
  if (!validateConfig(validated_config)) {
    return false;
  }

  const std::size_t cells = descriptorCells(validated_config);
  std::ofstream stream(path, std::ios::binary | std::ios::trunc);
  if (!stream) {
    return false;
  }

  stream.write(kMagic, sizeof(kMagic));
  if (!writePod(stream, kFormatVersion) || !writeConfig(stream, validated_config)) {
    return false;
  }
  if (!writePod(stream, static_cast<std::uint32_t>(entries_.size()))) {
    return false;
  }
  for (const auto & entry : entries_) {
    if (entry.image.size() != cells) {
      return false;
    }
    if (!writePod(stream, entry.position.x()) || !writePod(stream, entry.position.y())) {
      return false;
    }
    stream.write(
      reinterpret_cast<const char *>(entry.image.data()),
      static_cast<std::streamsize>(entry.image.size() * sizeof(float)));
    if (!stream) {
      return false;
    }
  }

  stream.flush();
  return static_cast<bool>(stream);
}

bool ScanContextDatabase::load(const std::string & path)
{
  if (path.empty()) {
    return false;
  }
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    return false;
  }

  char magic[sizeof(kMagic)] = {};
  stream.read(magic, sizeof(magic));
  if (!stream || std::memcmp(magic, kMagic, sizeof(kMagic)) != 0) {
    return false;
  }
  std::uint32_t version = 0;
  if (!readPod(stream, version) || version != kFormatVersion) {
    return false;
  }

  ScanContextConfig loaded_config;
  if (!readConfig(stream, loaded_config) || !validateConfig(loaded_config)) {
    return false;
  }

  std::uint32_t entry_count = 0;
  if (!readPod(stream, entry_count)) {
    return false;
  }
  const std::size_t cells = descriptorCells(loaded_config);
  const std::size_t count = static_cast<std::size_t>(entry_count);
  if (count > kMaxEntriesPerFile || count * cells > kMaxLoadFloats) {
    return false;
  }

  // 先读进临时容器，全部成功再覆盖原内容，load 失败不破坏已有数据库。
  std::vector<Entry> loaded_entries;
  loaded_entries.reserve(count);
  for (std::size_t entry_index = 0; entry_index < count; ++entry_index) {
    Entry entry;
    float x = 0.0f;
    float y = 0.0f;
    if (!readPod(stream, x) || !readPod(stream, y)) {
      return false;
    }
    entry.position = Eigen::Vector2f(x, y);
    if (!entry.position.allFinite()) {
      return false;
    }

    entry.image.resize(cells);
    stream.read(
      reinterpret_cast<char *>(entry.image.data()),
      static_cast<std::streamsize>(entry.image.size() * sizeof(float)));
    if (!stream) {
      return false;
    }

    // sector key 不落盘，加载时由 image 重算（少存 sectors 个 float，也不怕格式漂移）。
    const int sectors = loaded_config.sectors;
    const int rings = loaded_config.rings;
    entry.sector_key.assign(static_cast<std::size_t>(sectors), 0.0f);
    for (int sector = 0; sector < sectors; ++sector) {
      double sum = 0.0;
      for (int ring = 0; ring < rings; ++ring) {
        const float value = entry.image[static_cast<std::size_t>(ring) * sectors + sector];
        if (!std::isfinite(value)) {
          return false;
        }
        sum += value;
      }
      entry.sector_key[static_cast<std::size_t>(sector)] =
        static_cast<float>(sum / rings);
    }
    loaded_entries.push_back(std::move(entry));
  }

  entries_ = std::move(loaded_entries);
  config_ = loaded_config;
  configured_ = true;
  releasePreparedMap();
  return true;
}

bool ScanContextDatabase::empty() const
{
  return entries_.empty();
}

std::size_t ScanContextDatabase::size() const
{
  return entries_.size();
}

const ScanContextConfig & ScanContextDatabase::config() const
{
  return config_;
}

std::vector<ScanContextCandidate> ScanContextDatabase::query(
  const pcl::PointCloud<pcl::PointXYZI> & scan_odom,
  const Eigen::Matrix4f & odom_from_base,
  int top_k) const
{
  std::vector<ScanContextCandidate> candidates;
  if (entries_.empty() || scan_odom.empty() || top_k <= 0 ||
    !odom_from_base.array().isFinite().all())
  {
    return candidates;
  }

  // 点云从 odom 系转到 base 系：odom 已重力对齐，这里只做刚体变换，不做 roll/pitch 修正
  // （摇臂/坡道上地平面倾斜会退化描述子，见头文件已知限制）。
  const Eigen::Matrix4f base_from_odom = odom_from_base.inverse();
  pcl::PointCloud<pcl::PointXYZI> base_cloud;
  base_cloud.reserve(scan_odom.size());
  for (const auto & point : scan_odom.points) {
    if (!isFinitePoint(point)) {
      continue;
    }
    const Eigen::Vector4f transformed =
      base_from_odom * Eigen::Vector4f(point.x, point.y, point.z, 1.0f);
    if (!std::isfinite(transformed.x()) || !std::isfinite(transformed.y()) ||
      !std::isfinite(transformed.z()))
    {
      continue;
    }
    pcl::PointXYZI base_point;
    base_point.x = transformed.x();
    base_point.y = transformed.y();
    base_point.z = transformed.z();
    base_point.intensity = point.intensity;
    base_cloud.push_back(base_point);
  }
  if (base_cloud.empty()) {
    return candidates;
  }

  DescriptorBuffer query_descriptor;
  if (!makeQueryDescriptor(base_cloud, config_, query_descriptor)) {
    return candidates;
  }

  // 1) 用 sector key 在全部条目上做便宜粗排：只对相似度最高的若干条目算完整距离。
  //    大图上这一步决定查询耗时（粗排 O(条目数×sectors²)，精排只有 O(64×rings×sectors²)）。
  const int sectors = config_.sectors;
  std::vector<std::pair<float, std::size_t>> key_ranking;
  key_ranking.reserve(entries_.size());
  for (std::size_t index = 0; index < entries_.size(); ++index) {
    key_ranking.emplace_back(
      sectorKeySimilarity(query_descriptor.sector_key, entries_[index].sector_key, sectors),
      index);
  }
  std::sort(
    key_ranking.begin(), key_ranking.end(),
    [](const std::pair<float, std::size_t> & left,
      const std::pair<float, std::size_t> & right) {
      if (left.first != right.first) {
        return left.first > right.first;
      }
      return left.second < right.second;  // 同分按建库顺序，保证确定性
    });

  const std::size_t key_limit = std::min(
    entries_.size(),
    std::max<std::size_t>(config_.sector_key_candidates, static_cast<std::size_t>(top_k)));

  // 2) 完整描述子距离 + yaw，过滤距离阈值后按距离升序返回。
  struct RankedCandidate
  {
    float distance{std::numeric_limits<float>::max()};
    int shift{0};
    std::size_t index{0};
  };
  std::vector<RankedCandidate> ranked;
  ranked.reserve(key_limit);
  DescriptorMatch match;
  for (std::size_t rank = 0; rank < key_limit; ++rank) {
    const std::size_t index = key_ranking[rank].second;
    if (!matchDescriptor(query_descriptor, entries_[index].image, config_, match)) {
      continue;
    }
    if (!(match.distance <= config_.distance_threshold)) {
      continue;
    }
    ranked.push_back({match.distance, match.shift, index});
  }
  std::sort(
    ranked.begin(), ranked.end(),
    [](const RankedCandidate & left, const RankedCandidate & right) {
      if (left.distance != right.distance) {
        return left.distance < right.distance;
      }
      return left.index < right.index;  // 同分按建库顺序，保证确定性
    });

  const std::size_t result_count =
    std::min(static_cast<std::size_t>(top_k), ranked.size());
  candidates.reserve(result_count);
  for (std::size_t rank = 0; rank < result_count; ++rank) {
    const RankedCandidate & best = ranked[rank];
    ScanContextCandidate candidate;
    candidate.position = entries_[best.index].position;
    candidate.yaw_rad = yawFromShift(best.shift, sectors);
    candidate.distance = best.distance;
    candidates.push_back(candidate);
  }
  return candidates;
}

void ScanContextDatabase::clearEntries()
{
  std::vector<Entry>().swap(entries_);
}

void ScanContextDatabase::releasePreparedMap()
{
  prepared_map_.reset();
}

}  // namespace fast_location

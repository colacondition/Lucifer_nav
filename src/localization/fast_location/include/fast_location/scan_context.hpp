#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace fast_location
{

// Scan Context 描述子配置。描述子是一个 rings × sectors 的极坐标高度矩阵：
// 格子取该环/扇区内点的最大 z（相对传感器），没有点的格子记 0。
// 它只编码"传感器周围一圈几何"，因此对 yaw 旋转不敏感（用列移对齐），
// 位置由建库时的采样位置给出，不依赖里程计。
struct ScanContextConfig
{
  int rings{30};
  int sectors{90};
  // 只编码水平距离落在 [min_range, max_radius] 的点；min_range 用来滤掉近地/车身点。
  float max_radius{30.0f};
  float min_range{1.0f};
  // 地图点云 z 原点相对传感器的偏移：相对 z = point.z - map_sensor_z。
  // Lucifer 的先验 PCD 的 z=0 在开机雷达平面（与 small_glim 建图一致），取 0；
  // 若换成以地面为 z=0 的老图，则填雷达安装高度。
  float map_sensor_z{0.0f};
  // 查询点云 z 原点相对传感器的偏移：scan 已转到 base 系时一般取 0。
  float query_sensor_z{0.0f};
  // 离线建库的 XY 网格步长（米）。
  float grid_step{1.0f};
  // 建库前对地图做体素降采样（米）；<=0 表示不降采样。
  float map_voxel{0.2f};
  // 描述子距离（1 - 有效列平均余弦，越小越像）超过该值的候选不返回。
  float distance_threshold{0.6f};
  // 查询优化：先按 sector key（每扇区平均高度）粗排，只对前这么多条目算完整描述子距离。
  // 建库条目少时自然退化为全量比较。
  std::size_t sector_key_candidates{64};
};

// 一个重定位候选：传感器在 map 系的平面位置/朝向 + 描述子距离。
// yaw 约定：yaw_rad 是传感器在 map 系的朝向（map_from_base 的 yaw），
// 由扇区列移估计，符号见 src/scan_context.cpp 中 yawFromShift 的推导。
struct ScanContextCandidate
{
  Eigen::Vector2f position{Eigen::Vector2f::Zero()};
  float yaw_rad{0.0f};
  float distance{0.0f};
};

// 内部实现类型（建库时降采样后的地图 + 平面邻域索引），定义在 .cpp 里。
// 公开头文件只前置声明，调用方不要直接使用。
struct ScanContextPreparedMap;

// 离线 Scan Context 数据库：地图按网格采样位置 → 每个位置一个描述子；
// 查询时按描述子距离召回 top-k 个位置候选（只给位置 + yaw 初值，精配准交给上游 GICP）。
// 线程安全：build/prepare/load 之后只读 query 可并发；本类内部不加锁。
class ScanContextDatabase
{
public:
  ScanContextDatabase() = default;

  // 用地图点云重建数据库：把地图按地图 z 语义编码，再按 XY 包围盒以 grid_step
  // 网格采样传感器位置（起点为 min_x/min_y，含上边界，与 global_search 的 AABB
  // 网格约定一致）。地图为空/配置非法/所有位置都没点返回 false 并清空。
  // 位置数超过描述子内存上限（约 256MB）时自动加大 grid_step，避免一次建库 OOM。
  bool build(const pcl::PointCloud<pcl::PointXYZI> & map, const ScanContextConfig & config);

  // 在显式给定的传感器位置上建库（单测/自定义采样用）。语义同 build()。
  bool buildAtPositions(
    const pcl::PointCloud<pcl::PointXYZI> & map,
    const ScanContextConfig & config,
    const std::vector<Eigen::Vector2f> & positions);

  // 离线增量建库：prepare 清空条目、保存配置并准备地图（体素降采样 + 平面邻域索引），
  // 之后用 addAtPosition 逐个位置追加。工具用它打印每个位置的耗时。
  bool prepare(const pcl::PointCloud<pcl::PointXYZI> & map, const ScanContextConfig & config);
  // 在 prepare 准备的地图上追加一个位置；位置 max_radius 内没有有效点则不入库返回 false。
  bool addAtPosition(const Eigen::Vector2f & position);

  // versioned 二进制格式：magic + version + config + 条目数 + float buffer，
  // 不依赖 PCL IO / boost。load 失败时保留原有内容。
  bool save(const std::string & path) const;
  bool load(const std::string & path);

  bool empty() const;
  std::size_t size() const;
  const ScanContextConfig & config() const;

  // 查询：scan_odom 是 odom 系点云，odom_from_base 是当前雷达在 odom 系的位姿。
  // 先用 odom_from_base 的逆把点云转到 base 系（odom 已重力对齐，本模块不做
  // roll/pitch 修正），再算描述子。返回按 distance 升序、最多 top_k 个候选。
  // 空库/空扫描/非法位姿/top_k<=0 返回空。
  std::vector<ScanContextCandidate> query(
    const pcl::PointCloud<pcl::PointXYZI> & scan_odom,
    const Eigen::Matrix4f & odom_from_base,
    int top_k) const;

private:
  // 一个地图采样位置的描述子。image 为 ring-major（长度 rings*sectors），
  // 空格子为 0；sector_key 是每扇区跨环平均，用于查询前的粗排。
  struct Entry
  {
    Eigen::Vector2f position{Eigen::Vector2f::Zero()};
    std::vector<float> image;
    std::vector<float> sector_key;
  };

  // 降采样后的地图 + 2D 邻域索引，定义在 .cpp 里避免头文件暴露实现细节。
  void clearEntries();
  void releasePreparedMap();

  ScanContextConfig config_;
  bool configured_{false};
  std::vector<Entry> entries_;
  std::shared_ptr<ScanContextPreparedMap> prepared_map_;
};

// 已知限制：
// 1) 描述子只编码"相对传感器的高度轮廓"，前提是 odom / 先验 PCD 已重力对齐；坡道或
//    车体侧倾未补偿时地平面会在环上摊开，距离变大（需要外部先把点云整平）。
// 2) 对称场地（正方形房间、走廊）可能出现多个列移距离接近，yaw 只是初值，必须交给
//    上游 GICP 精化 + margin/时序验证兜底，不能单独作为定位结果。
// 3) .scdb 与建库参数（rings/sectors/max_radius/map_sensor_z/map_voxel 等）强绑定，
//    加载端必须用同一份 config 构造查询描述子；版本号不匹配/文件损坏时 load 返回 false。

}  // namespace fast_location

#ifndef NAVIGATION2__SFC_CORRIDOR_HPP_
#define NAVIGATION2__SFC_CORRIDOR_HPP_

// 方形安全走廊（Square Safety Corridor）生成器。
//
// 算法来源：TDT-nav-kit 的 SfcSquare（MIT License，
// Copyright (c) 2026 Rongxuan Ye and BoLin Li），按 Lucifer 的地图语义与线程
// 纪律重写。三处有意的差异，移植时不要"改回去"：
//
// 1. origin 符号统一。TDT 原实现 isOutside/lineInObsticle 用 (x - origin)、
//    _getBoundSquare2/valueAt 用 (x + origin)，origin 非零时走廊和碰撞检测
//    用的是两套坐标系。Lucifer 的地图 origin 非零（RMUC 是 [-2.48, -8.65]），
//    这里全程只有一个约定：cell = floor((world - origin) / resolution)。
//
// 2. 查询复杂度从 O(r²) 降到 O(log r)。TDT 对每个路径点从中心逐格向外扩张，
//    2.5 m / 0.05 m = 50 格时单点最多 10⁴ 次格查询，85 个路标点一次重规划就是
//    10⁶ 次。这里把「以该格为中心的最大无致命格正方形」转成积分图上的区间和
//    为零判定，建图 O(n) 一次、单点查询 O(log r)。8 Hz 重规划下两者差一个
//    数量级以上。
//
// 3. max_range 必须是有限正数。TDT 默认 FLT_MAX，static_cast<int>(round(
//    FLT_MAX / mapping)) 是未定义行为，随后可能变成超长循环；这里在
//    updateRaw() 里直接拒绝非法参数并保持未就绪状态，宁可不生成走廊也不 UB。
//
// 语义：本类回答的是「以这个点为中心、半宽多大的正方形内一个致命格都没有」，
// 不是「这个点离最近障碍多远」（那是 EDT 的事，见 distance_transform.hpp）。
// 两者的差别在方形贴着障碍但中心离障碍尚远时：clearanceRadius 给出的是
// 方形不受污染的半宽，它同时受四轴最近的致命格约束。
//
// 线程安全：不可并发使用。设计上由持有者串行化 —— smoother / planner 的
// 订阅与定时器都在各自的 MutuallyExclusive 回调组里，天然满足。不内建互斥锁
// 是有意的：热路径加锁会把无争用的临界区变成新的延迟源。

#include <cstdint>
#include <vector>

#include <Eigen/Core>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace navigation2
{

// 轴对齐矩形，世界坐标（map 系），单位米。[x0,y0] 是最小角，[x1,y1] 是最大角。
struct SfcBox
{
  double x0{0.0};
  double y0{0.0};
  double x1{0.0};
  double y1{0.0};

  // 退化 = 没有可用空间（半宽 <= 0）。调用方应把它当作「此处不能放松约束」，
  // 而不是「此处无障碍」。
  bool degenerate() const noexcept { return !(x1 > x0 && y1 > y0); }
  bool contains(double x, double y) const noexcept
  {
    return x >= x0 && x <= x1 && y >= y0 && y <= y1;
  }
  double halfWidth() const noexcept { return std::min(x1 - x0, y1 - y0) * 0.5; }
};

struct SfcCorridorParams
{
  // 单个走廊的最大半宽（米）。必须为有限正数，否则整个生成器保持未就绪。
  double max_range{2.5};
  // 车体半径（米）。与 rm_global_costmap / rm_local_costmap 的 robot_radius 同源：
  // clearanceRadius >= robot_radius 才表示圆形车体在该点无碰。
  double robot_radius{0.25};
  // 代价 >= 此值的格视为致命。与全局规划器的 obstacle_threshold 同一取值，
  // 两边不一致会把 A* 走过的膨胀格判成不可行。
  int obstacle_threshold{100};
  // 未知格（-1）是否算致命。与全局规划器的 allow_unknown 互补：
  // allow_unknown=true 时这里应填 false，否则规划出的穿未知区路径会被本类整条否掉。
  bool unknown_is_lethal{false};
};

class SfcCorridor
{
public:
  SfcCorridor() = default;

  // 从 OccupancyGrid 重建。内容哈希没变时直接复用上一张图（不重算积分图）。
  // 网格尺寸变化时内部缓冲一次性重分配，之后同尺寸更新零分配。
  void updateGrid(
    const nav_msgs::msg::OccupancyGrid & grid, const SfcCorridorParams & params);

  // 测试与离线工具用的原始入口：直接吃致命位图，跳过 ROS 消息适配。
  // lethal[i] != 0 表示格 i 致命，行主序（y * width + x）。
  void updateRaw(
    int width, int height, double resolution, double origin_x, double origin_y,
    const std::vector<std::uint8_t> & lethal, const SfcCorridorParams & params);

  bool ready() const noexcept { return width_ > 0 && height_ > 0 && resolution_ > 0.0; }

  // 世界坐标是否落在网格覆盖范围内。范围外无法判定，调用方应跳过而不是当致命。
  bool insideMap(double x, double y) const noexcept;

  // 该点所在格是否致命。范围外返回 false（用 insideMap 先判断）。
  bool pointLethal(double x, double y) const noexcept;

  // 以该点所在格为中心的最大无致命格正方形的半宽（米）。
  // 上限受三重约束：max_range、地图边界、致命格。
  // 返回 -1.0 表示无法判定（未就绪或图外）。
  double clearanceRadius(double x, double y) const noexcept;

  // clearanceRadius 减去 robot_radius 后的可用区域（车体中心可安全放置的范围）。
  // clearance < robot_radius 时返回退化 box。
  SfcBox usableBoxAt(double x, double y) const noexcept;

  // 批量校验：所有采样点都在图内且 clearanceRadius >= robot_radius 时返回 true。
  // 这是发布前硬校验的全图版 —— MINCO 现有的校验只覆盖局部 5x5 m 滚动窗口，
  // 5 m 以外的全局障碍对它不可见。
  bool pathClear(
    const std::vector<Eigen::Vector2d> & samples, double robot_radius) const noexcept;

  const SfcCorridorParams & params() const noexcept { return params_; }

private:
  // 格 [x0,y0]..[x1,y1]（含端点）内的致命格数量。调用方保证 0 <= x0 <= x1 < width。
  std::int64_t lethalCount(int x0, int y0, int x1, int y1) const noexcept;

  int width_{0};
  int height_{0};
  double resolution_{0.0};
  double origin_x_{0.0};
  double origin_y_{0.0};
  SfcCorridorParams params_;

  // 致命位图（width*height）。u8 而不是 bit：查询走积分图，位图只在建图时读。
  std::vector<std::uint8_t> lethal_;
  // 积分图 ((width+1)*(height+1))，prefix_[(y)*(w+1)+(x)] = [0,y)x[0,x) 的致命数。
  // i32 足够：168k 格最多 168k，远小于 2^31。
  std::vector<std::int32_t> prefix_;
  std::uint64_t content_hash_{0};
  bool has_content_{false};
};

}  // namespace navigation2

#endif  // NAVIGATION2__SFC_CORRIDOR_HPP_

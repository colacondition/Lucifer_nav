#ifndef NAVIGATION2__SEMANTIC_MAP_CONSUMER_HPP_
#define NAVIGATION2__SEMANTIC_MAP_CONSUMER_HPP_

// 语义地图的消费端适配层。
//
// 为什么单独一层：语义地图的栅格是 rm_map_server 定的（RMUC 是 0.05 m/格，origin
// [-2.48, -8.65]），而消费端各有自己的栅格 —— 局部代价地图是 0.02 m/格、跟车滚动
// 的窗口。两边格号不通用，每个消费端都要做一次「世界坐标 → 语义格」的换算。写三遍
// 就会有三种边界处理，所以收在这里。
//
// 膨胀由 rm_map_server 离线做一次，这里只做反序列化 + 查询，不重算。

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <decision_interfaces/msg/semantic_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "semantic_map.hpp"

namespace navigation2
{

// 消息 → 语义地图。通道长度与 width*height 不一致时抛 std::runtime_error，
// 调用方应当 catch 并丢弃这一帧而不是拿半张图去规划。
SemanticMap semanticMapFromMsg(const decision_interfaces::msg::SemanticMap & msg);

// 接收端缓存：内容没变就不重建。
//
// rm_map_server 每秒重发同一张图给漏收的订阅者兜底，但重建一次 RMUC 大小的方向场
// 要做 16 万次三角函数，而且会让消费端的派生缓存（比如逐格膨胀半径）跟着失效重算。
// 这些节点和 MPC 跑在同一个 component_container_mt 的执行器上，每秒白花几毫秒会直接
// 变成控制抖动。
//
// 只比通道内容，不比 header —— 重发时时间戳必然不同，比上就永远「变了」。
class SemanticMapReceiver
{
public:
  // 返回 true 表示地图内容确实变了（首次收到也算），此时调用方应当刷新自己的派生
  // 缓存。消息不自洽时抛 std::runtime_error，且不改动已持有的地图 —— 宁可继续用上
  // 一张好图，也不要因为一帧坏消息把隧道语义整个丢掉。
  bool update(const decision_interfaces::msg::SemanticMap & msg);

  const SemanticMap & map() const noexcept { return map_; }

private:
  SemanticMap map_;
  decision_interfaces::msg::SemanticMap last_msg_;
  bool has_map_{false};
};

// 世界坐标落在哪条隧道的本体内；不在任何隧道本体内返回 nullptr。
// 返回的指针指向 map 内部，生命周期跟随 map。
const TunnelSpec * tunnelSpecAtPoint(const SemanticMap & map, double world_x, double world_y);

// 世界坐标处的隧道轴线。落在隧道本体内时写出单位轴线向量并返回 true；否则返回
// false（不改 axis）。给 MINCO 的对齐软代价用 —— 它按世界坐标采样，不走栅格格号。
//
// 隧道双向：轴线只表朝向，正反等价，对齐判据在调用方用 |cos θ|。
bool tunnelAxisAtPoint(const SemanticMap & map, const Eigen::Vector2d & point, Eigen::Vector2d & axis);

// 每格的膨胀半径上限，按 grid 的格号索引。非隧道格是 default_radius。
//
// 隧道格必须单独给：净宽 W 的洞里，半径 r 的车两侧各只剩 (W/2 - r) 的余量，用全局
// 的 0.5 m 膨胀会把整条通道涂成高代价，A* 直接绕开这条唯一的通路。壁面格本身仍是
// 致命的，所以压小半径不会让车撞墙 —— 只是不再为洞内预留横向余量，这在窄洞里本来
// 就不存在。
//
// map 无效时返回空 vector，表示「无逐格上限」。
std::vector<float> makeInflationRadiusLimit(
  const nav_msgs::msg::OccupancyGrid & grid, const SemanticMap & map, double default_radius,
  double robot_radius);

// 隧道影响区：本体格向外扩 margin_m 的一圈，摊平成语义栅格大小的查表。
//
// 为什么本体格不够：门楣/顶板前沿的点云落点和洞口正前方的格子都在本体格边界外
// 一到两格。只认本体格时，这排点被标成致命格横在洞口上、洞口格又吃到两侧墙的
// 全量膨胀，洞口整体被封死 —— 车停在洞口，云台绿灯也进不去。扩一圈边距后，
// 顶板豁免和膨胀上限都覆盖到洞口，能不能过只由壁面自己的致命格决定。
//
// 预先摊平的理由与 TunnelAxisGrid 相同：查询跑在点云逐点循环里（每帧上万次），
// 每点做半径搜索不可接受；语义地图一张只变一次，构建一次之后查询只是一次查表。
class TunnelRegionGrid
{
public:
  // 本体格向外做圆形膨胀 margin_m。map 无效或没有隧道时返回空表。
  static TunnelRegionGrid build(const SemanticMap & map, double margin_m);

  bool empty() const noexcept { return spec_index_.empty(); }

  // 世界坐标落在任一隧道的影响区（本体 + 边距）内时返回该隧道的通行参数，
  // 否则返回 nullptr。多条隧道的边距重叠时取距离最近的那条。
  const TunnelSpec * specNearPoint(double world_x, double world_y) const noexcept;

  // 点是否落在该点所属隧道的「保护走廊」内。
  //
  // 走廊 = 轴线段沿方向 ±(half_len + run_up)、横向 ±(clear_width/2 + margin) 的
  // 矩形。几何推导：build() 时按 tunnel id 收集本体格做主成分 (PCA) 取轴向与
  // 端点极值。影响区内的点若在走廊外，则不再享受一刀切放行 —— 这正是
  // 「洞内真实障碍可见化」的开口：敌方实体停在走廊侧带或洞口环带上时会被
  // 正常多帧确认为障碍；走廊内部仍维持先验放行（顶板投影与两侧壁基 clutter
  // 不能封死通道）。判据依旧与高度无关。
  bool pointOutsideCorridor(double world_x, double world_y) const noexcept;

  // 世界坐标处可用于「路径硬约束」的走廊几何。
  //
  // 与 pointOutsideCorridor 的保护走廊不同：保护走廊用的是 clear_width/2 + margin
  // （往外扩，为了把洞口的门楣点云也罩进去），这里是给平滑器/规划器约束**车体
  // 中心**用的，横向半宽是 clear_width/2 - robot_radius + lateral_margin（往里
  // 收，保证圆形车体不擦壁）。两者方向相反，绝不能混用。
  //
  // 关于「半宽恰好为 0」：这不是退化，是**真实地图的常态**。RMUL 两条隧道的
  // clear_width 都是 0.5 m，而代价地图的 robot_radius 是 0.25 m —— 车体直径
  // 正好等于净宽，物理余量为零。此时的正确约束不是「放弃约束」，而是
  // 「把车压在轴线上」（半宽 0 的走廊就是轴线本身）。所以这里对负值钳到 0 而
  // 不是返回 false：钳到 0 让约束生效，返回 false 会让整个特性在真实地图上
  // 静默失效。
  //
  // 纵向不设界：路径要穿过隧道，纵向越界是正常的（车要从洞两头出去），
  // 只有横向偏移才是要压住的对象。
  struct CorridorFrame
  {
    Eigen::Vector2d centroid{0.0, 0.0};  // 本体格质心（世界系）
    Eigen::Vector2d dir{1.0, 0.0};       // 单位轴向（无向）
    double half_len{0.0};                // 纵向半长：|along| <= half_len 时算洞内
    double half_width_inner{0.0};        // 横向半宽（物理），可为 0（= 压轴线）
    // 影响区的横向外沿：clear_width/2 + margin_m。|lat| 超过它就不再属于这条隧道
    // 的影响区，惩罚项据此把代价平滑地封顶，而不是在边界上跳变。
    double lateral_outer{0.0};
  };

  // 返回 false 只表示「这条隧道的轴不可用」：不在任何隧道影响区内、轴表为空、
  // 或轴退化（half_len <= 0，单格/共线秩亏）。此时调用方应跳过，不要编造约束。
  bool corridorFrameAtPoint(
    double world_x, double world_y, double robot_radius, double lateral_margin,
    CorridorFrame & out) const noexcept;

private:
  struct CorridorAxis
  {
    Eigen::Vector2d centroid{0.0, 0.0};   // 本体格质心（世界系）
    Eigen::Vector2d dir{1.0, 0.0};        // 单位主方向
    double half_len{0.0};                 // 质心到两端的本体长度的一半
    std::uint8_t spec_id{0};
  };

  GridGeometry geometry_;
  // 每格所属隧道的 id + 1（0 = 不在任何影响区内）。
  std::vector<std::uint8_t> spec_index_;
  // 自持一份 TunnelSpec 拷贝，查询结果的生命周期跟随本对象而不是构建时的那张图。
  std::vector<TunnelSpec> tunnels_;
  // 每条隧道一条 PCA 轴（index = spec_id - 1）。id 无本体格时为空向量。
  std::vector<CorridorAxis> axes_;
  // 构建时的边距（m）：走廊横向半宽 = clear_width/2 + margin_m_。
  double margin_m_{0.0};
};

// 带影响区的版本：区内（本体 + 边距）的格子吃 clearance 上限，让洞口不再被两侧
// 墙的膨胀涂满。region 为空时退回上面只认本体格的版本。
std::vector<float> makeInflationRadiusLimit(
  const nav_msgs::msg::OccupancyGrid & grid, const SemanticMap & map, double default_radius,
  double robot_radius, const TunnelRegionGrid & region);

// 按消费端栅格格号索引的隧道轴线表，给栅格搜索用。
//
// 为什么要预先摊平成数组：A* 每弹出一格要问 8 次「这一步允许吗」，每次都做
// 「格号 → 世界坐标 → 语义格」的换算会把这个换算跑进百万次量级，而语义地图一张
// 只变一次。摊平之后每次查询只是两次数组读。
//
// 存单位向量而不是角度：判据是 |cos θ| = |axis·step|，点积不需要三角函数。
class TunnelAxisGrid
{
public:
  // grid 的每一格取其中心点落进的语义格。语义分辨率（0.05 m）通常比消费端粗，
  // 多个消费端格映射到同一个语义格是正常的。
  static TunnelAxisGrid build(
    const nav_msgs::msg::OccupancyGrid & grid, const SemanticMap & map);

  // 空表示这张图上没有隧道，调用方可以整段跳过轴向判定。
  bool empty() const noexcept { return axis_x_.empty(); }
  std::size_t size() const noexcept { return axis_x_.size(); }

  // 该格是否是隧道本体（车压在隧道上），而非膨胀圈。
  bool isTunnelCell(std::size_t index) const noexcept;

  // 从 from 走一步到 to 的轴向对齐度 |cos θ|，θ 是步进方向与隧道轴线的夹角。
  //
  // 两端都不在隧道本体内时返回 1.0（无约束）。只要有一端在隧道内就要算 —— 进洞的
  // 那一步尤其重要，斜着切进洞口正是要加代价的情形。隧道是直的，所以两端都在洞内
  // 时轴线相同，取哪端都一样。
  //
  // 隧道双向，所以取绝对值：正着进和倒着进都合法（见 SemanticMap::axisAlignment）。
  double stepAlignment(std::size_t from, std::size_t to, int step_x, int step_y) const noexcept;

private:
  std::vector<float> axis_x_;
  std::vector<float> axis_y_;
};

}  // namespace navigation2

#endif  // NAVIGATION2__SEMANTIC_MAP_CONSUMER_HPP_

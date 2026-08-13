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

private:
  GridGeometry geometry_;
  // 每格所属隧道的 id + 1（0 = 不在任何影响区内）。
  std::vector<std::uint8_t> spec_index_;
  // 自持一份 TunnelSpec 拷贝，查询结果的生命周期跟随本对象而不是构建时的那张图。
  std::vector<TunnelSpec> tunnels_;
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

#ifndef NAVIGATION2__SEMANTIC_MAP_HPP_
#define NAVIGATION2__SEMANTIC_MAP_HPP_

// 语义导航地图：地形标签 + 方向场。
//
// 为什么需要它：占据栅格只回答「这一格能不能站」，但隧道是一种「能站，但必须
// 摆对姿态才能进」的地形。顶板和侧壁在点云里和墙没有区别，纯几何的代价地图会
// 把整个洞口封死；反过来，如果只是把隧道格标成空闲，规划器会斜着切进洞口然后
// 卡在入口。两种失败都不是参数能调出来的，必须有先验语义。
//
// 数据布局与 pgm 的关系：msgpack 是唯一真源。terrain 通道里 OBSTACLE 的格子
// 就是障碍，rm_map_server 由它生成 /map，所以语义和占据不可能不同步。
//
// 方向场的两层语义（沿用 HWSentryNav26 的约定）：
//   模长 > 0.95  → 地形本体，车体正位于该地形上，方向约束是硬的。
//   模长 ≤ 0.9   → 膨胀圈，只携带「附近有方向地形」的弱信息，用于软塑形。
// 中间的 (0.9, 0.95] 是空档，用来让「本体」判定不受量化误差影响。
//
// 隧道方向是无向的：direction 存轴线朝向，正反等价，对齐判据用 |cos θ| 而不是
// cos θ。这是隧道与台阶的根本差异 —— 台阶的方向是「上行」，有正反之分。

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace navigation2
{

// 地形标签。3~6 预留给以后的坡和台阶，UNKNOWN 固定在 7。
//
// UNKNOWN 必须是独立标签而不是「当成 FLAT」：现有 RMUC.pgm 有约 2.6 万个从未
// 扫到的格子，map yaml 里专门把 free_thresh 压到 0.196 才让它们吃到 A* 的
// unknown_cost 惩罚。转成离散标签后若丢掉这个区分，规划器会以正常代价穿越从
// 未观测的区域。
enum class TerrainType : std::uint8_t
{
  FLAT = 0,
  OBSTACLE = 1,
  TUNNEL = 2,
  RESERVED_3 = 3,
  RESERVED_4 = 4,
  RESERVED_5 = 5,
  RESERVED_6 = 6,
  UNKNOWN = 7,
};

constexpr std::size_t kTerrainLabelCount = 8;

// 携带方向场的标签。目前只有隧道，留成函数是为了以后加坡/台阶时只改这一处。
constexpr bool isDirectionalLabel(std::uint8_t label)
{
  return label == static_cast<std::uint8_t>(TerrainType::TUNNEL);
}

// 方向场模长阈值：超过它算地形本体。
constexpr double kTerrainBodyMagnitudeThreshold = 0.95;
// 膨胀圈模长上限。量化成 uint8 后是 229/255。
constexpr double kMaxInflatedMagnitude = 0.9;

const char * terrainLabelName(std::uint8_t label);

// 隧道的物理尺寸与通行参数。一张地图里可以有多条隧道，用 id 关联到栅格。
struct TunnelSpec
{
  // 净高（m）。底盘变形后的车体高度必须小于它。
  double clear_height{0.0};
  // 净宽（m）。决定隧道格的膨胀半径 —— 用全局的 inflation_radius 会把窄通道填满。
  double clear_width{0.0};
  // 入口前必须完成变形的距离（m）。
  double run_up{0.5};
  // 洞内速度窗（m/s）。
  double velocity_min{0.0};
  double velocity_max{0.0};
};

// 离线膨胀参数。物理距离而非格数 —— 换分辨率时不用重新调参。
struct InflationParams
{
  double resolution{0.05};
  // 满代价半径：该半径内代价保持 255，方向场本体模长保持 1.0。
  double full_cost_radius_m{0.10};
  // 截断半径：超过它不再膨胀。
  double cutoff_radius_m{0.30};
  // 指数衰减率（1/m）。
  double decay_rate_per_m{24.0};
  // 非本体格的方向模长上限，必须在 (0, 0.9]。
  double non_body_magnitude_cap{kMaxInflatedMagnitude};
};

// 栅格几何。与 nav_msgs/OccupancyGrid 的约定一致：origin 是栅格 (0,0) 的角点，
// 覆盖范围是半开区间 [origin, origin + size * resolution)。
struct GridGeometry
{
  int width{0};
  int height{0};
  double resolution{0.0};
  // 参考工程把地图原点固定在栅格原点所以没有这个字段；Lucifer 的地图 origin
  // 非零（RMUC 是 [-2.48, -8.65]），照抄会让整张图偏移 8 米。
  Eigen::Vector2d origin{Eigen::Vector2d::Zero()};

  bool valid() const noexcept;
  std::size_t cellCount() const noexcept;
  bool containsCell(int x, int y) const noexcept;
  bool containsPoint(const Eigen::Vector2d & point) const noexcept;
  std::size_t index(int x, int y) const noexcept;
  std::optional<Eigen::Vector2i> containingCell(const Eigen::Vector2d & point) const noexcept;
  Eigen::Vector2d cellCenter(int x, int y) const noexcept;
  bool sameAs(const GridGeometry & other) const noexcept;
};

// 从 msgpack 读出来的原始语义地图，未经膨胀。
struct SemanticMapData
{
  GridGeometry geometry;
  // 每格的地形标签。
  std::vector<std::uint8_t> terrain;
  // 每格的方向编码：0~255 → 0~2π。仅对 isDirectionalLabel 的格子有意义。
  std::vector<std::uint8_t> direction;
  // 按隧道 id 索引的通行参数。空表示地图里没有隧道。
  std::vector<TunnelSpec> tunnels;
  // 每格所属的隧道 id + 1（0 表示不属于任何隧道）。为空时所有隧道格共用
  // tunnels[0]。
  std::vector<std::uint8_t> tunnel_ids;

  bool valid() const noexcept;
};

// 膨胀后的语义地图。方向场以连续场的形式存储（而非角度+模长），因为下游优化器
// 要的是可导的向量场，每次采样都做一次 cos/sin 反解是浪费。
class SemanticMap
{
public:
  struct DirectionSample
  {
    Eigen::Vector2d value{Eigen::Vector2d::Zero()};
    // 1/m；列分别是对 map x / y 的偏导。
    Eigen::Matrix2d jacobian{Eigen::Matrix2d::Zero()};
  };

  struct CostSample
  {
    double value{0.0};
    // cost/m。
    Eigen::Vector2d gradient{Eigen::Vector2d::Zero()};
  };

  // 双线性权重按标签聚合。标签 L 的权重是采样模板中携带 L 的格子的权重之和，
  // 于是 sum_L w_L * f(L) 形式的惩罚在格子边界上连续 —— 这是让 A* 的离散判定
  // 和 MINCO 的连续代价用同一套语义的前提。
  struct LabelWeights
  {
    std::array<double, kTerrainLabelCount> weights{};
    std::array<Eigen::Vector2d, kTerrainLabelCount> dweights{};

    LabelWeights() { dweights.fill(Eigen::Vector2d::Zero()); }
  };

  SemanticMap() = default;

  // 对原始地图做离线膨胀：障碍生成连续代价场，隧道轴线按距离衰减向外传播。
  static SemanticMap inflate(const SemanticMapData & data, const InflationParams & params);

  // 从已膨胀好的通道重建。给消费端用：膨胀由 rm_map_server 做一次，其余节点收到
  // 通道后直接装回来，不重算。
  //
  // 用 angle + magnitude 两个 uint8 通道而不是直接传向量：这是发布格式，量化到
  // 每格 2 字节让整张 RMUC 图的方向场只占 320 KB。重建时反解成向量，之后所有
  // 采样都走向量，不再有三角函数。
  //
  // 几何非法或任一通道长度与 width*height 不一致时抛 std::runtime_error。不返回
  // 空图是有意的：长度对不上说明两端对栅格的理解不一致，此时格号可能整体错位，
  // 静默降级成「这张图没有隧道」会让顶板照旧封住洞口而日志里什么都没有。
  static SemanticMap fromChannels(
    const GridGeometry & geometry, std::vector<std::uint8_t> terrain,
    const std::vector<std::uint8_t> & direction_angle,
    const std::vector<std::uint8_t> & direction_magnitude, std::vector<std::uint8_t> cost,
    std::vector<TunnelSpec> tunnels, std::vector<std::uint8_t> tunnel_ids);

  bool valid() const noexcept { return geometry_.valid() && !terrain_.empty(); }
  const GridGeometry & geometry() const noexcept { return geometry_; }
  const std::vector<std::uint8_t> & terrain() const noexcept { return terrain_; }
  const std::vector<std::uint8_t> & cost() const noexcept { return cost_; }
  const std::vector<TunnelSpec> & tunnels() const noexcept { return tunnels_; }
  const std::vector<std::uint8_t> & tunnelIds() const noexcept { return tunnel_ids_; }

  std::uint8_t terrainAtCell(int x, int y) const noexcept;
  std::uint8_t costAtCell(int x, int y) const noexcept;
  Eigen::Vector2d directionAtCell(int x, int y) const noexcept;
  double magnitudeAtCell(int x, int y) const noexcept;
  // 车体是否正位于该地形上（而非只是在它附近）。
  bool isTerrainBodyCell(int x, int y) const noexcept;
  bool isTunnelBodyCell(int x, int y) const noexcept;
  // 该格所属隧道的通行参数；不在隧道本体内时返回 nullptr。
  const TunnelSpec * tunnelSpecAtCell(int x, int y) const noexcept;

  // 越界时复制边界格的值，梯度在越界轴上归零。调用方若需要跨边界连续的惩罚，
  // 自己叠一层距离斜坡。
  CostSample sampleCost(const Eigen::Vector2d & point) const noexcept;
  DirectionSample sampleDirection(const Eigen::Vector2d & point) const noexcept;
  LabelWeights sampleLabelWeights(const Eigen::Vector2d & point) const noexcept;

  // 轴线对齐度 |cos θ|，θ 是 heading 与该点轴线方向的夹角。隧道是双向的，所以
  // 取绝对值 —— 正着进和倒着进都合法。不在方向场覆盖范围内时返回 1.0（无约束）。
  double axisAlignment(const Eigen::Vector2d & point, const Eigen::Vector2d & heading) const
    noexcept;

private:
  GridGeometry geometry_;
  std::vector<std::uint8_t> terrain_;
  std::vector<std::uint8_t> cost_;
  std::vector<Eigen::Vector2d> direction_;
  std::vector<std::uint8_t> tunnel_ids_;
  std::vector<TunnelSpec> tunnels_;
};

// 从 msgpack 文件读原始语义地图。文件损坏或不自洽时抛 std::runtime_error。
SemanticMapData loadSemanticMap(const std::string & path);

}  // namespace navigation2

#endif  // NAVIGATION2__SEMANTIC_MAP_HPP_

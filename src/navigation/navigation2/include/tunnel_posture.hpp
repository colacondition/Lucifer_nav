#ifndef NAVIGATION2__TUNNEL_POSTURE_HPP_
#define NAVIGATION2__TUNNEL_POSTURE_HPP_

// 「什么时候该让电控收云台」的判定。
//
// 导航侧只负责这一个布尔量：车即将穿洞或正在洞里。收云台本身、收到多低、到位反馈
// 都归电控。所以这里不需要任何车体参数 —— 不需要知道云台多高、底盘离地多少。
//
// 判定靠「是否真要穿洞」：规划路径穿过隧道本体（will_cross）才收。贴着洞口路过
// （并不进洞）不收 —— 这正是跟旧「附近有没有隧道」纯几何判据的区别。唯一例外是
// 车已经进到隧道本体内：此时无论路径怎么判都强制收，兜住路径过期/重规划漏发的安全
// 缺口（车在洞里而云台立着，撞顶板的代价不可逆）。

#include <optional>
#include <vector>

#include "semantic_map.hpp"

namespace navigation2
{

struct TunnelProximity
{
  // 到最近的隧道本体格中心的距离（m）。
  double distance{0.0};
  // 该格所属隧道的通行参数。指向 map 内部，生命周期跟随 map。
  const TunnelSpec * spec{nullptr};
};

// 搜索框内离该点最近的隧道本体格。图里没有隧道本体格时返回 nullopt。
//
// 不在函数内做圆形过滤，阈值比较留给调用方：run_up 是逐隧道的，只有找到是哪条
// 隧道之后才知道该跟什么比。搜索框至少覆盖包含该点的那一格，所以「车已经在洞里」
// 一定能被找到。
std::optional<TunnelProximity> nearestTunnelBody(
  const SemanticMap & map, double world_x, double world_y, double search_radius);

// 路径是否穿过任何一条隧道本体：任意路径点落在隧道本体格内即返回 true。
// 这是「收云台」的真正判据 —— 只有要穿洞才需要收，贴着洞口路过（不进洞）不该收。
bool pathCrossesTunnel(const SemanticMap & map, const std::vector<Eigen::Vector2d> & points);

// 带滞回的收云台判定。
//
// 滞回是必需的：判定跑在 10~20 Hz 上，边界附近若无滞回，标志位会在真假之间抖动，
// 电控那边就变成云台反复抬落。抬起的门槛比落下的高一档。
//
// will_cross 是「是否要穿洞」的判据（来自规划路径是否穿过隧道本体）。false 时即使
// 车靠近洞口也不收 —— 这正是跟旧「附近有没有隧道」纯几何判据的区别。唯一例外是
// 车已经进到隧道本体内：此时无论路径怎么判都强制收，兜住路径过期/重规划漏发的安全
// 缺口（车在洞里而云台立着，撞顶板的代价不可逆）。
class GimbalLowerDecider
{
public:
  explicit GimbalLowerDecider(double hysteresis_m = 0.3)
  : hysteresis_m_(hysteresis_m) {}

  // 用当前位姿 + 是否穿洞刷新判定，返回刷新后的结果。
  bool update(const SemanticMap & map, double world_x, double world_y, bool will_cross = true);

  bool lower() const noexcept { return lower_; }

private:
  double hysteresis_m_{0.3};
  bool lower_{false};
};

}  // namespace navigation2

#endif  // NAVIGATION2__TUNNEL_POSTURE_HPP_

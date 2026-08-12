#ifndef NAVIGATION2__TUNNEL_POSTURE_HPP_
#define NAVIGATION2__TUNNEL_POSTURE_HPP_

// 「什么时候该让电控收云台」的判定。
//
// 导航侧只负责这一个布尔量：车即将进洞或正在洞里。收云台本身、收到多低、到位反馈
// 都归电控。所以这里不需要任何车体参数 —— 不需要知道云台多高、底盘离地多少。
//
// 判定只用位姿和语义地图，不用规划路径。路径会过期、会因为重规划抖动，而漏发一次
// 「收」的代价是云台撞在顶板上。用「附近有没有隧道」这个纯几何条件换来的是：没有
// 时序依赖，没有 stale 状态，任何一帧都能独立算出正确答案。
//
// 代价是贴着洞口开过去（并不进洞）也会收云台，损失的是这段时间的火力。两个失败
// 方向不对等，所以偏向收。

#include <optional>

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

// 带滞回的收云台判定。
//
// 滞回是必需的：判定跑在 10~20 Hz 上，边界附近若无滞回，标志位会在真假之间抖动，
// 电控那边就变成云台反复抬落。抬起的门槛比落下的高一档。
class GimbalLowerDecider
{
public:
  explicit GimbalLowerDecider(double hysteresis_m = 0.3)
  : hysteresis_m_(hysteresis_m) {}

  // 用当前位姿刷新判定，返回刷新后的结果。
  bool update(const SemanticMap & map, double world_x, double world_y);

  bool lower() const noexcept { return lower_; }

private:
  double hysteresis_m_{0.3};
  bool lower_{false};
};

}  // namespace navigation2

#endif  // NAVIGATION2__TUNNEL_POSTURE_HPP_

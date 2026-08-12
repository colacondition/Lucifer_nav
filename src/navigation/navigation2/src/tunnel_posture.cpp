#include "tunnel_posture.hpp"

#include <algorithm>
#include <cmath>

namespace navigation2
{

std::optional<TunnelProximity> nearestTunnelBody(
  const SemanticMap & map, double world_x, double world_y, double search_radius)
{
  if (!map.valid() || map.tunnels().empty()) {
    return std::nullopt;
  }

  const auto & geometry = map.geometry();
  const Eigen::Vector2d query(world_x, world_y);

  // 搜索框按物理距离换成格数。必须向上取整：search_radius 不是分辨率的整数倍时，
  // 截断会让恰好落在半径内的隧道格掉到框外，表现为「离洞口 0.58 米时标志位还没置真，
  // 再往前一格突然置真」—— 提前量凭空少一格。
  //
  // span 为 0 也是安全的：框退化成车所在的那一格，「已经在洞里」仍然找得到。
  const int span = static_cast<int>(std::ceil(search_radius / geometry.resolution));

  // 车可能在图外（局部窗口越过边界），此时用最靠近的格号做搜索中心而不是直接放弃。
  const int center_x = static_cast<int>(
    std::floor((world_x - geometry.origin.x()) / geometry.resolution));
  const int center_y = static_cast<int>(
    std::floor((world_y - geometry.origin.y()) / geometry.resolution));

  const int min_x = std::max(0, center_x - span);
  const int max_x = std::min(geometry.width - 1, center_x + span);
  const int min_y = std::max(0, center_y - span);
  const int max_y = std::min(geometry.height - 1, center_y + span);

  std::optional<TunnelProximity> best;
  for (int y = min_y; y <= max_y; ++y) {
    for (int x = min_x; x <= max_x; ++x) {
      if (!map.isTunnelBodyCell(x, y)) {
        continue;
      }
      const double distance = (geometry.cellCenter(x, y) - query).norm();
      if (best && distance >= best->distance) {
        continue;
      }
      // 本体格一定有 spec（isTunnelBodyCell 与 tunnelSpecAtCell 同源），但仍然检查：
      // spec 为空时下游会拿不到 run_up，宁可当成「这里没有隧道」也不要空指针。
      const TunnelSpec * spec = map.tunnelSpecAtCell(x, y);
      if (spec == nullptr) {
        continue;
      }
      best = TunnelProximity{distance, spec};
    }
  }
  return best;
}

bool GimbalLowerDecider::update(const SemanticMap & map, double world_x, double world_y)
{
  // 搜索框要覆盖任何一条隧道的 run_up，加上滞回那一档。逐隧道的 run_up 只有找到
  // 隧道之后才知道，所以先按图里最大的 run_up 搜，再逐条比自己的阈值。
  double max_run_up = 0.0;
  for (const auto & spec : map.tunnels()) {
    max_run_up = std::max(max_run_up, spec.run_up);
  }
  const auto nearest =
    nearestTunnelBody(map, world_x, world_y, max_run_up + hysteresis_m_);
  if (!nearest) {
    lower_ = false;
    return lower_;
  }

  // 落下用 run_up，抬起要多退 hysteresis_m_ 才算离开。10~20 Hz 下没有滞回会让
  // 标志位在边界上抖，电控那边就是云台反复抬落。
  const double threshold =
    lower_ ? nearest->spec->run_up + hysteresis_m_ : nearest->spec->run_up;
  lower_ = nearest->distance <= threshold;
  return lower_;
}

}  // namespace navigation2

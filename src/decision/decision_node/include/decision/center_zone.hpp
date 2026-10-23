#pragma once

#include <vector>

#include "decision/types.hpp"

namespace decision
{

// 中心占领区的几何。
//
// 为什么用**四个角点**而不是"中心 + 边长 + 朝向"：
//   1. `map` 系是开机位姿，不是场地系，所以区域相对 map 转了多少度是未知的。
//      用角点就完全不用管这件事。
//   2. 不要求它是正方形（规则里是正方形，但录点总有误差，四点法更宽容）。
//   3. 角点录错能被自动查出来：把 patrol.csv 的点喂给 zoneContainsAll()，
//      有一个落在外面就说明角点或 map 有问题。见 zoneContainsAll()。
//
// ⚠️ 夹取时必须留 safety_margin_m 余量：夹到边界后如果定位/控制误差把车推出区，
// **占领权当场就丢了**。LIO 的局部误差很小（<0.1m），但余量不能省。

// 四个角点（map 系）构成的区域，构造时就把"安全区"算好。
struct CenterZone
{
  // 原始角点，按逆时针排好（输入顺序随意，构造时会排）
  std::vector<Pose> corners;
  // 内缩 safety_margin 之后的安全区（逆时针）。夹取和判定都用它。
  std::vector<Pose> safe;
  bool usable{false};
};

// 由角点建区域。
// - 输入顺序随意（顺/逆时针、乱序都行），内部按凸包排列
// - 非 4 点、共线、非凸、面积退化 → usable = false（宁可不用，也不要一个错的区域）
// - margin_m <= 0 → 安全区 = 原区域
// - margin 大到把区域缩没了 → usable = false
CenterZone makeCenterZone(const std::vector<Pose> & corners, double margin_m);

// 点是否在**安全区**内。
bool zoneContains(const CenterZone & zone, const Pose & p);

// 把点夹进**安全区**：区内原样返回，区外返回安全区里离它最近的点。
//
// 这是"追击限制在区域内"的核心：敌人在区外时返回的不是"拒绝"，而是**边界上离敌人
// 最近的那个点** —— 于是哨兵会贴着边界压上去，既保持占领又贴脸。
//
// zone.usable == false 时返回入参原样（调用方应先检查 usable，不要拿它当"已夹取"）。
Pose clampToZone(const CenterZone & zone, const Pose & p);

// 一批点是否**全部**落在区域内（用 margin_m 判定，传 0 就是原始区域）。
// 用来拿 patrol.csv 反查角点录得对不对 —— 那些点本来就应该全在区内。
bool zoneContainsAll(
  const CenterZone & zone, const std::vector<Pose> & points, double margin_m);

}  // namespace decision

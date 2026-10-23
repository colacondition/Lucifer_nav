#pragma once

#include <cstddef>
#include <vector>

#include "decision/center_zone.hpp"
#include "decision/types.hpp"

namespace decision
{

// 中心走位：**在中心区域里来回走**，就这一件事。
//
// 刻意**没有**激进/保守之分，也**不看时间**。理由：
//   - "哪一侧算激进"取决于场上实际结构（过道是 S 型的、哪一侧有掩护、敌人从哪个口进来），
//     不是能从坐标推出来的几何量。没有可靠判据就不要假造一个。
//   - 走位本身不改变占领权（区域是凸的、点都在区域内），所以它只是"别站着不动"。
//
// 于是规则只剩一条：**在区域内的网格点上按顺序走，走完一组停一会儿，再走下一组。**

// 在安全区里生成网格点，顺序是**蛇形**（boustrophedon）：一行走完掉头走下一行。
//
// 布点用**格子中心**而不是包围盒的边界刻度：四边形上下边往往是斜的，把首末两行放在
// 包围盒边上会让它们几乎整行落在多边形外 —— 白白浪费掉两行，剩下的点还会挤在一起。
// 放格子中心就能均匀铺满。
//
// 顺序必须"列表里相邻的点在场上也相邻"：游标是顺序推进的，中间冒出一个远点车就会跳。
// 蛇形满足这一点（行内相邻约 spacing，行末到下一行行首也只有约 spacing）。
//
// zone 不可用、safe 少于 3 点、spacing <= 0 → 返回空。
std::vector<Pose> generateZoneGrid(const CenterZone & zone, double spacing_m);

// 一次走位的选取结果。
struct PatrolPath
{
  std::vector<std::size_t> indices;  // 点池下标
  std::size_t cursor{0};             // 推进后的游标
};

// 从网格里**按顺序**取 path_len 个点，走到尾绕回头。
// path_len 大于点池时最多取满整池（一条路径里不重复取同一个点）。
PatrolPath takePatrolPath(std::size_t pool_size, std::size_t cursor, int path_len);

}  // namespace decision

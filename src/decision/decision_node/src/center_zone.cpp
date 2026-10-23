#include "decision/center_zone.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace decision
{
namespace
{

// 面积太小就认为退化（共线/重合）。单位是 m^2。
constexpr double kMinAreaSq = 1.0e-4;
// 叉积判定的容差，避免边界上的浮点抖动被判成"外面"。
constexpr double kCrossEps = 1.0e-9;

struct Vec2
{
  double x{0.0};
  double y{0.0};
};

Vec2 toVec(const Pose & p) { return Vec2{p.x, p.y}; }

double cross(const Vec2 & a, const Vec2 & b) { return a.x * b.y - a.y * b.x; }

Vec2 sub(const Vec2 & a, const Vec2 & b) { return Vec2{a.x - b.x, a.y - b.y}; }

double dot(const Vec2 & a, const Vec2 & b) { return a.x * b.x + a.y * b.y; }

double norm(const Vec2 & a) { return std::hypot(a.x, a.y); }

double distance(const Vec2 & a, const Vec2 & b) { return norm(sub(a, b)); }

Pose makePose(const Vec2 & v)
{
  Pose p;
  p.x = v.x;
  p.y = v.y;
  return p;
}

double polygonArea(const std::vector<Pose> & pts)
{
  double twice = 0.0;
  for (std::size_t i = 0; i < pts.size(); ++i) {
    const auto & a = pts[i];
    const auto & b = pts[(i + 1) % pts.size()];
    twice += a.x * b.y - b.x * a.y;
  }
  return std::abs(twice) * 0.5;
}

// 按相对质心的极角排序 → 逆时针。对凸集来说这就等价于凸包顺序。
std::vector<Pose> orderCounterClockwise(const std::vector<Pose> & corners)
{
  Vec2 centroid{0.0, 0.0};
  for (const auto & c : corners) {
    centroid.x += c.x;
    centroid.y += c.y;
  }
  centroid.x /= static_cast<double>(corners.size());
  centroid.y /= static_cast<double>(corners.size());

  auto ordered = corners;
  std::sort(ordered.begin(), ordered.end(), [&](const Pose & a, const Pose & b) {
    return std::atan2(a.y - centroid.y, a.x - centroid.x) <
           std::atan2(b.y - centroid.y, b.x - centroid.x);
  });
  return ordered;
}

// 逆时针多边形是否处处左转（凸）。允许共线（叉积 ~ 0）。
bool isConvexCcw(const std::vector<Pose> & ccw)
{
  const std::size_t n = ccw.size();
  for (std::size_t i = 0; i < n; ++i) {
    const Vec2 a = toVec(ccw[i]);
    const Vec2 b = toVec(ccw[(i + 1) % n]);
    const Vec2 c = toVec(ccw[(i + 2) % n]);
    if (cross(sub(b, a), sub(c, b)) < -kCrossEps) {
      return false;
    }
  }
  return true;
}

// 逆时针多边形的每条边向内（左侧）平移 margin，返回新顶点。
// 边 i 的方向 d_i，内向法线 n_i = (-d_y, d_x) 归一化。
// 新顶点 i = 平移后的边 i-1 与边 i 的交点。
bool offsetInward(
  const std::vector<Pose> & ccw, double margin, std::vector<Pose> & out)
{
  const std::size_t n = ccw.size();
  out.clear();
  out.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    const Vec2 prev_a = toVec(ccw[(i + n - 1) % n]);
    const Vec2 prev_b = toVec(ccw[i]);
    const Vec2 cur_a = toVec(ccw[i]);
    const Vec2 cur_b = toVec(ccw[(i + 1) % n]);

    const Vec2 d_prev = sub(prev_b, prev_a);
    const Vec2 d_cur = sub(cur_b, cur_a);
    const double len_prev = norm(d_prev);
    const double len_cur = norm(d_cur);
    if (len_prev < kCrossEps || len_cur < kCrossEps) {
      return false;
    }
    // 内向法线（逆时针 ⇒ 内部在左侧）
    const Vec2 n_prev{-d_prev.y / len_prev, d_prev.x / len_prev};
    const Vec2 n_cur{-d_cur.y / len_cur, d_cur.x / len_cur};
    // 平移后的两条边各取一点
    const Vec2 p_prev{prev_a.x + margin * n_prev.x, prev_a.y + margin * n_prev.y};
    const Vec2 p_cur{cur_a.x + margin * n_cur.x, cur_a.y + margin * n_cur.y};

    const double denom = cross(d_prev, d_cur);
    if (std::abs(denom) < kCrossEps) {
      // 相邻边平行：正四边形里不该出现；出现了说明输入退化
      return false;
    }
    const Vec2 diff = sub(p_cur, p_prev);
    const double t = cross(diff, d_cur) / denom;
    out.push_back(makePose(Vec2{p_prev.x + t * d_prev.x, p_prev.y + t * d_prev.y}));
  }
  return true;
}

// 点到线段的最近点
Vec2 closestOnSegment(const Vec2 & p, const Vec2 & a, const Vec2 & b)
{
  const Vec2 ab = sub(b, a);
  const double len_sq = dot(ab, ab);
  if (len_sq < kCrossEps) {
    return a;
  }
  double t = dot(sub(p, a), ab) / len_sq;
  t = std::min(1.0, std::max(0.0, t));
  return Vec2{a.x + t * ab.x, a.y + t * ab.y};
}

}  // namespace

CenterZone makeCenterZone(const std::vector<Pose> & corners, double margin_m)
{
  CenterZone zone;
  if (corners.size() != 4) {
    return zone;
  }

  auto ordered = orderCounterClockwise(corners);
  // 去重后的点数：4 个点里有重合就退化
  double min_pair = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < ordered.size(); ++i) {
    for (std::size_t j = i + 1; j < ordered.size(); ++j) {
      min_pair = std::min(min_pair, distance(toVec(ordered[i]), toVec(ordered[j])));
    }
  }
  if (min_pair < 1.0e-3) {
    return zone;
  }
  if (polygonArea(ordered) < kMinAreaSq) {
    return zone;
  }
  if (!isConvexCcw(ordered)) {
    return zone;
  }

  zone.corners = ordered;
  if (margin_m <= 0.0) {
    zone.safe = ordered;
    zone.usable = true;
    return zone;
  }

  std::vector<Pose> shrunk;
  if (!offsetInward(ordered, margin_m, shrunk)) {
    return zone;
  }
  // 内缩后必须还是个有面积的凸多边形；缩没了就判定不可用
  if (polygonArea(shrunk) < kMinAreaSq) {
    return zone;
  }
  // ⚠️ 关键检查：内缩量超过内切半径时，"相邻边交点"拼出来的多边形**不是**那个
  // 内缩区域 —— 1.3 内缩一个半边长 1.2 的正方形会得到一个小方块 (0.1,0.1)…
  // 它面积正常、绕行方向也正常、顶点还在原区域内，但已经**不满足内缩半平面约束**
  // （那个小方块整个翻到了右边界的另一侧）。不查这一条，安全区会静默变成一个小方块：
  // 车还安全，但哪儿都去不了，而且没有任何报错。
  //
  // 精确判据：拼出来的每个顶点都必须满足全部内缩半平面。
  for (std::size_t i = 0; i < ordered.size(); ++i) {
    const Vec2 a = toVec(ordered[i]);
    const Vec2 b = toVec(ordered[(i + 1) % ordered.size()]);
    const Vec2 d = sub(b, a);
    const double len = norm(d);
    if (len < kCrossEps) {
      return zone;
    }
    const Vec2 n{-d.y / len, d.x / len};   // 逆时针 ⇒ 内向在左侧
    const Vec2 base{a.x + margin_m * n.x, a.y + margin_m * n.y};
    for (const auto & vertex : shrunk) {
      if (dot(n, sub(toVec(vertex), base)) < -kCrossEps) {
        return zone;
      }
    }
  }
  if (!isConvexCcw(shrunk)) {
    return zone;
  }
  // 安全区必须真的比原区域小；不满足说明 offset 算错了
  if (polygonArea(shrunk) >= polygonArea(ordered)) {
    return zone;
  }
  for (std::size_t i = 0; i < shrunk.size(); ++i) {
    // 内缩后的顶点应该仍在原区域内部（严格说应更靠里），再兜一层
    const Vec2 a = toVec(ordered[i]);
    const Vec2 b = toVec(ordered[(i + 1) % ordered.size()]);
    if (cross(sub(b, a), sub(toVec(shrunk[i]), a)) < -kCrossEps) {
      return zone;
    }
  }

  zone.safe = shrunk;
  zone.usable = true;
  return zone;
}

bool zoneContains(const CenterZone & zone, const Pose & p)
{
  if (!zone.usable) {
    return false;
  }
  const std::size_t n = zone.safe.size();
  const Vec2 point = toVec(p);
  for (std::size_t i = 0; i < n; ++i) {
    const Vec2 a = toVec(zone.safe[i]);
    const Vec2 b = toVec(zone.safe[(i + 1) % n]);
    // 逆时针 ⇒ 内部在每条边的左侧
    if (cross(sub(b, a), sub(point, a)) < -kCrossEps) {
      return false;
    }
  }
  return true;
}

Pose clampToZone(const CenterZone & zone, const Pose & p)
{
  if (!zone.usable || zoneContains(zone, p)) {
    return p;
  }

  const std::size_t n = zone.safe.size();
  const Vec2 point = toVec(p);
  Vec2 best = toVec(zone.safe[0]);
  double best_dist = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < n; ++i) {
    const Vec2 a = toVec(zone.safe[i]);
    const Vec2 b = toVec(zone.safe[(i + 1) % n]);
    const Vec2 candidate = closestOnSegment(point, a, b);
    const double d = distance(point, candidate);
    if (d < best_dist) {
      best_dist = d;
      best = candidate;
    }
  }
  return makePose(best);
}

bool zoneContainsAll(
  const CenterZone & zone, const std::vector<Pose> & points, double margin_m)
{
  if (!zone.usable) {
    return false;
  }
  if (margin_m <= 0.0) {
    return std::all_of(
      points.begin(), points.end(),
      [&](const Pose & p) { return zoneContains(zone, p); });
  }
  const auto shrunk = makeCenterZone(zone.corners, margin_m);
  if (!shrunk.usable) {
    return false;
  }
  return std::all_of(
    points.begin(), points.end(),
    [&](const Pose & p) { return zoneContains(shrunk, p); });
}

}  // namespace decision

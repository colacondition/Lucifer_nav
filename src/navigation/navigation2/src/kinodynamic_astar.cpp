#include "kinodynamic_astar.hpp"

#include "grid_utils.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <unordered_map>

namespace navigation2::mpc
{
namespace
{

constexpr int kControlDivisions = 4;   // 控制输入在 [-maxAcc, +maxAcc] 上的采样档数
constexpr int kDurationSteps = 3;      // 每个控制输入的持续时长档数
constexpr int kStartDurationSteps = 20;  // 首段（沿用给定加速度）的时长细分
constexpr double kGoalCellRadius = 12.0;  // 终点连接允许的格半径

// 四次/三次方程求根（解析），用于「最优时间」启发式。与 TDT 一致的 DAQP 风格。
std::vector<double> solveCubic(double a, double b, double c, double d)
{
  std::vector<double> roots;
  if (std::abs(a) < 1e-12) {
    return roots;
  }
  const double a2 = b / a;
  const double a1 = c / a;
  const double a0 = d / a;
  const double Q = (3.0 * a1 - a2 * a2) / 9.0;
  const double R = (9.0 * a1 * a2 - 27.0 * a0 - 2.0 * a2 * a2 * a2) / 54.0;
  const double D = Q * Q * Q + R * R;
  if (D > 0.0) {
    const double S = std::cbrt(R + std::sqrt(D));
    const double T = std::cbrt(R - std::sqrt(D));
    roots.push_back(-a2 / 3.0 + (S + T));
  } else if (D == 0.0) {
    const double S = std::cbrt(R);
    roots.push_back(-a2 / 3.0 + S + S);
    roots.push_back(-a2 / 3.0 - S);
  } else {
    const double theta = std::acos(R / std::sqrt(-Q * Q * Q));
    roots.push_back(2.0 * std::sqrt(-Q) * std::cos(theta / 3.0) - a2 / 3.0);
    roots.push_back(2.0 * std::sqrt(-Q) * std::cos((theta + 2.0 * M_PI) / 3.0) - a2 / 3.0);
    roots.push_back(2.0 * std::sqrt(-Q) * std::cos((theta + 4.0 * M_PI) / 3.0) - a2 / 3.0);
  }
  return roots;
}

std::vector<double> solveQuartic(double a, double b, double c, double d, double e)
{
  std::vector<double> roots;
  if (std::abs(a) < 1e-12) {
    return roots;
  }
  const double a3 = b / a;
  const double a2 = c / a;
  const double a1 = d / a;
  const double a0 = e / a;

  // depressed quartic 的 resolvent cubic
  const std::vector<double> ys = solveCubic(
    1.0, -a2, a1 * a3 - 4.0 * a0, 4.0 * a2 * a0 - a1 * a1 - a3 * a3 * a0);
  if (ys.empty()) {
    return roots;
  }
  const double y1 = ys.front();
  const double r = a3 * a3 / 4.0 - a2 + y1;
  if (r < 0.0) {
    return roots;
  }
  const double R = std::sqrt(r);
  double D = 0.0;
  double E = 0.0;
  if (R != 0.0) {
    D = std::sqrt(0.75 * a3 * a3 - R * R - 2.0 * a2 +
      0.25 * (4.0 * a3 * a2 - 8.0 * a1 - a3 * a3 * a3) / R);
    E = std::sqrt(0.75 * a3 * a3 - R * R - 2.0 * a2 -
      0.25 * (4.0 * a3 * a2 - 8.0 * a1 - a3 * a3 * a3) / R);
  } else {
    D = std::sqrt(0.75 * a3 * a3 - 2.0 * a2 + 2.0 * std::sqrt(y1 * y1 - 4.0 * a0));
    E = std::sqrt(0.75 * a3 * a3 - 2.0 * a2 - 2.0 * std::sqrt(y1 * y1 - 4.0 * a0));
  }
  if (!std::isnan(D)) {
    roots.push_back(-a3 / 4.0 + R / 2.0 + D / 2.0);
    roots.push_back(-a3 / 4.0 + R / 2.0 - D / 2.0);
  }
  if (!std::isnan(E)) {
    roots.push_back(-a3 / 4.0 - R / 2.0 + E / 2.0);
    roots.push_back(-a3 / 4.0 - R / 2.0 - E / 2.0);
  }
  return roots;
}

struct SearchContext
{
  const nav_msgs::msg::OccupancyGrid & grid;
  const KinoConfig & config;
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  int width{0};
  int height{0};

  SearchContext(const nav_msgs::msg::OccupancyGrid & g, const KinoConfig & c)
  : grid(g), config(c),
    resolution(static_cast<double>(g.info.resolution)),
    origin_x(static_cast<double>(g.info.origin.position.x)),
    origin_y(static_cast<double>(g.info.origin.position.y)),
    width(static_cast<int>(g.info.width)),
    height(static_cast<int>(g.info.height)) {}

  bool usable() const noexcept
  {
    return width > 0 && height > 0 && resolution > 0.0 &&
           grid.data.size() == static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  }

  int cellX(double x) const noexcept
  {
    return static_cast<int>(std::floor((x - origin_x) / resolution));
  }
  int cellY(double y) const noexcept
  {
    return static_cast<int>(std::floor((y - origin_y) / resolution));
  }

  // 致命判定：与全局规划器同一阈值语义（-1 按 unknown_is_lethal 处理）。
  bool cellLethal(int cx, int cy) const noexcept
  {
    if (cx < 0 || cx >= width || cy < 0 || cy >= height) {
      return true;  // 图外按不可行处理，搜索不会跑出地图
    }
    const std::int8_t v = grid.data[static_cast<std::size_t>(cy) * static_cast<std::size_t>(width) +
      static_cast<std::size_t>(cx)];
    if (v < 0) {
      return true;  // 未知格按不可行：动力学搜索不该往没观测过的地方钻
    }
    return static_cast<int>(v) >= 100;
  }

  bool pointBlocked(const Eigen::Vector2d & p) const noexcept
  {
    if (!p.allFinite()) {
      return true;
    }
    return cellLethal(cellX(p.x()), cellY(p.y()));
  }

  // 线段碰撞：Bresenham 逐格。与 TDT 的 lineInObsticle 同一作用，
  // 这里直接用 grid_utils 的 raytraceLine（恢复规划器已经在用同一实现）。
  bool lineBlocked(const Eigen::Vector2d & a, const Eigen::Vector2d & b) const noexcept
  {
    const int x0 = cellX(a.x());
    const int y0 = cellY(a.y());
    const int x1 = cellX(b.x());
    const int y1 = cellY(b.y());
    if (x0 < 0 || y0 < 0 || x0 >= width || y0 >= height) {
      return true;
    }
    if (x1 < 0 || y1 < 0 || x1 >= width || y1 >= height) {
      return true;
    }
    for (const auto & cell : raytraceLine(x0, y0, x1, y1)) {
      if (cellLethal(cell.x, cell.y)) {
        return true;
      }
    }
    return false;
  }
};

// 离散状态键：格坐标 + 离散速度。
using KinoKey = std::array<int, 4>;

struct KinoKeyHash
{
  std::size_t operator()(const KinoKey & key) const noexcept
  {
    std::size_t seed = 0;
    for (const int value : key) {
      seed ^= std::hash<int>{}(value) + 0x9e3779b9U + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

struct KinoNode
{
  Eigen::Vector4d state{Eigen::Vector4d::Zero()};
  Eigen::Vector2d input{Eigen::Vector2d::Zero()};
  double duration{0.0};
  double cost{0.0};
  int parent{-1};
};

// 恒加速度段的三次多项式系数（按列：x/y 两行，[c0,c1,c2,c3] 升幂）。
using CoefMat = Eigen::Matrix<double, 2, 4>;

// 逐轴解析检查速度/加速度上限：加速度线性、速度二次，端点 + 内部极值即可。
bool segmentWithinLimits(const CoefMat & coef, double duration, const KinoConfig & config)
{
  for (int dim = 0; dim < 2; ++dim) {
    const double a = coef(dim, 3);
    const double b = coef(dim, 2);
    const double c = coef(dim, 1);
    // 加速度 = 6a t + 2b；端点即可（线性）。
    if (std::max(std::abs(2.0 * b), std::abs(6.0 * a * duration + 2.0 * b)) >
      config.max_acc + 1e-9)
    {
      return false;
    }
    // 速度 = 3a t² + 2b t + c；端点 + 内部极值 t = -b/(3a)。
    if (std::max(std::abs(c), std::abs((3.0 * a * duration + 2.0 * b) * duration + c)) >
      config.max_vel + 1e-9)
    {
      return false;
    }
    if (a != 0.0) {
      const double t = -b / (3.0 * a);
      if (t > 0.0 && t < duration &&
        std::abs((3.0 * a * t + 2.0 * b) * t + c) > config.max_vel + 1e-9)
      {
        return false;
      }
    }
  }
  return true;
}

// 空间检查：按半格步距采样并逐段连线。只查端点会漏掉「曲线切进障碍再出来」。
bool segmentCollisionFree(
  const CoefMat & coef, double duration, const SearchContext & ctx)
{
  const double speed_bound = std::max(1e-3, ctx.config.max_vel * std::sqrt(2.0));
  const double samples_d = std::ceil(duration * speed_bound / (ctx.resolution * 0.5));
  if (!std::isfinite(samples_d) || samples_d >= static_cast<double>(std::numeric_limits<int>::max())) {
    return false;
  }
  const int count = std::max(1, static_cast<int>(samples_d));
  Eigen::Vector2d previous = coef.col(0);
  for (int i = 1; i <= count; ++i) {
    const double t = duration * static_cast<double>(i) / static_cast<double>(count);
    const Eigen::Vector2d p = ((coef.col(3) * t + coef.col(2)) * t + coef.col(1)) * t + coef.col(0);
    if (ctx.pointBlocked(p) || ctx.lineBlocked(previous, p)) {
      return false;
    }
    previous = p;
  }
  return true;
}

// 三次多项式连接两个状态（位置 + 速度都给定），返回系数；不可行返回 false。
bool connectCubic(
  const Eigen::Vector4d & start, const Eigen::Vector4d & end, double duration,
  CoefMat & coef)
{
  if (!std::isfinite(duration) || duration <= 0.0) {
    return false;
  }
  const Eigen::Vector2d dp = end.head<2>() - start.head<2>();
  const Eigen::Vector2d v0 = start.tail<2>();
  const Eigen::Vector2d dv = end.tail<2>() - v0;
  coef.col(0) = start.head<2>();
  coef.col(1) = v0;
  coef.col(2) = 3.0 * (dp - v0 * duration) / (duration * duration) - dv / duration;
  coef.col(3) =
    -2.0 * (dp - v0 * duration) / (duration * duration * duration) + dv / (duration * duration);
  return coef.allFinite();
}

// 「最优时间」启发式：对 ∫|a|² dt + time_weight*T 解四次方程取最小代价的正根。
double estimateHeuristic(
  const Eigen::Vector4d & start, const Eigen::Vector4d & end, const SearchContext & ctx,
  double & optimal_time)
{
  const Eigen::Vector2d dp = end.head<2>() - start.head<2>();
  const Eigen::Vector2d v0 = start.tail<2>();
  const Eigen::Vector2d v1 = end.tail<2>();

  const double c1 = -36.0 * dp.dot(dp);
  const double c2 = 24.0 * (v0 + v1).dot(dp);
  const double c3 = -4.0 * (v0.dot(v0) + v0.dot(v1) + v1.dot(v1));

  auto times = solveQuartic(ctx.config.time_weight, 0.0, c3, c2, c1);
  // 下界：即便一直以半速开，也至少要这么久。防止四次方程无正根时代价发散。
  const double lower = std::max(1e-3,
    dp.cwiseAbs().maxCoeff() / (ctx.config.max_vel * 0.5));
  times.push_back(lower);

  double best = std::numeric_limits<double>::infinity();
  optimal_time = lower;
  for (const double t : times) {
    if (!std::isfinite(t) || t < lower) {
      continue;
    }
    const double value =
      -c1 / (3.0 * t * t * t) - c2 / (2.0 * t * t) - c3 / t + ctx.config.time_weight * t;
    if (value < best) {
      best = value;
      optimal_time = t;
    }
  }
  return best;
}

KinoKey toKey(const Eigen::Vector4d & state, const SearchContext & ctx)
{
  return {ctx.cellX(state.x()), ctx.cellY(state.y()),
    static_cast<int>(std::round(state[2] / ctx.config.vel_resolution)),
    static_cast<int>(std::round(state[3] / ctx.config.vel_resolution))};
}

void sampleSegment(
  const CoefMat & coef, double duration, double time_offset, const SearchContext & ctx,
  KinoResult & result)
{
  const int count = std::max(1, static_cast<int>(
    std::ceil(duration / std::max(1e-6, ctx.config.sample_time))));
  for (int i = 1; i <= count; ++i) {
    const double t = duration * static_cast<double>(i) / static_cast<double>(count);
    KinoSample sample;
    sample.state.head<2>() =
      ((coef.col(3) * t + coef.col(2)) * t + coef.col(1)) * t + coef.col(0);
    sample.state.tail<2>() = (3.0 * coef.col(3) * t + 2.0 * coef.col(2)) * t + coef.col(1);
    sample.acceleration = 6.0 * coef.col(3) * t + 2.0 * coef.col(2);
    sample.time = time_offset + t;
    result.trajectory.push_back(sample);
  }
}

}  // namespace

bool kinoParamsValid(const KinoConfig & config) noexcept
{
  return config.max_vel > 0.0 && config.max_acc > 0.0 && config.max_tau > 0.0 &&
         config.vel_resolution > 0.0 && config.time_weight > 0.0 &&
         config.heuristic_weight > 0.0 && config.sample_time > 0.0 && config.max_nodes > 0 &&
         std::isfinite(config.max_vel) && std::isfinite(config.max_acc) &&
         std::isfinite(config.max_tau) && std::isfinite(config.vel_resolution) &&
         std::isfinite(config.time_weight) && std::isfinite(config.heuristic_weight) &&
         std::isfinite(config.sample_time) &&
         // 速度离散档数要能放进 int：这是 toKey() 的隐含约束。
         config.max_vel / config.vel_resolution <
         static_cast<double>(std::numeric_limits<int>::max());
}

KinoResult kinodynamicSearch(
  const nav_msgs::msg::OccupancyGrid & grid, const KinoConfig & config,
  const Eigen::Vector2d & start, const Eigen::Vector2d & start_vel,
  const Eigen::Vector2d & start_acc, const Eigen::Vector2d & goal)
{
  KinoResult result;
  SearchContext ctx(grid, config);

  if (!ctx.usable() || !kinoParamsValid(config) || !start.allFinite() || !goal.allFinite() ||
    !start_vel.allFinite() || !start_acc.allFinite())
  {
    return result;
  }
  // 起点速度/加速度超出上限、或起终点在致命格上：直接失败，不做部分搜索。
  if (start_vel.cwiseAbs().maxCoeff() > config.max_vel ||
    start_acc.cwiseAbs().maxCoeff() > config.max_acc ||
    ctx.pointBlocked(start) || ctx.pointBlocked(goal))
  {
    return result;
  }

  const Eigen::Vector4d start_state = (Eigen::Vector4d() << start, start_vel).finished();
  const Eigen::Vector4d goal_state =
    (Eigen::Vector4d() << goal, Eigen::Vector2d::Zero()).finished();

  if ((start_state - goal_state).squaredNorm() == 0.0) {
    result.success = true;
    result.trajectory.push_back({start_state, start_acc, 0.0});
    return result;
  }

  // 节点入队后不再修改（改进路径 = 创建新节点），保证堆里存的下标与父链稳定。
  // 这是 TDT 原实现的正确设计，保留 —— 懒删除堆配可变节点会有父链被改坏的坑。
  std::vector<KinoNode> nodes;
  nodes.reserve(config.max_nodes);
  using Entry = std::pair<double, int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
  std::unordered_map<KinoKey, int, KinoKeyHash> expanded;

  nodes.push_back({start_state, start_acc, 0.0, 0.0, -1});
  double time_to_goal = 0.0;
  open.push({config.heuristic_weight * estimateHeuristic(start_state, goal_state, ctx, time_to_goal), 0});
  expanded[toKey(start_state, ctx)] = 0;

  while (!open.empty()) {
    const int index = open.top().second;
    open.pop();
    const KinoNode current = nodes[static_cast<std::size_t>(index)];
    // 懒删除：同键已有更优节点在堆里就跳过。
    if (expanded[toKey(current.state, ctx)] != index) {
      continue;
    }
    ++result.iterations;

    // 终点连接：位置进入终点 12 格半径内就尝试三次多项式直连（速度归零）。
    if ((current.state.head<2>() - goal).cwiseAbs().maxCoeff() <= kGoalCellRadius * ctx.resolution) {
      estimateHeuristic(current.state, goal_state, ctx, time_to_goal);
      CoefMat shot;
      if (connectCubic(current.state, goal_state, time_to_goal, shot) &&
        segmentWithinLimits(shot, time_to_goal, config) &&
        segmentCollisionFree(shot, time_to_goal, ctx))
      {
        result.trajectory.push_back({start_state, start_acc, 0.0});
        std::vector<int> path;
        for (int at = index; nodes[static_cast<std::size_t>(at)].parent != -1;
          at = nodes[static_cast<std::size_t>(at)].parent)
        {
          path.push_back(at);
        }
        for (auto it = path.rbegin(); it != path.rend(); ++it) {
          const KinoNode & node = nodes[static_cast<std::size_t>(*it)];
          CoefMat segment;
          const KinoNode & parent = nodes[static_cast<std::size_t>(node.parent)];
          segment << parent.state.head<2>(), parent.state.tail<2>(),
            node.input * 0.5, Eigen::Vector2d::Zero();
          const double offset = result.trajectory.back().time;
          sampleSegment(segment, node.duration, offset, ctx, result);
        }
        const double offset = result.trajectory.back().time;
        sampleSegment(shot, time_to_goal, offset, ctx, result);
        result.success = true;
        result.nodes = nodes.size();
        return result;
      }
    }

    // 首段沿用给定加速度（车不是从静止起步时的真实初条件）；完全静止才枚举控制。
    const bool initial = index == 0 &&
      (start_vel.squaredNorm() != 0.0 || start_acc.squaredNorm() != 0.0);
    const int control_range = initial ? 0 : kControlDivisions;
    for (int ax = -control_range; ax <= control_range; ++ax) {
      for (int ay = -control_range; ay <= control_range; ++ay) {
        const Eigen::Vector2d input = initial ? start_acc :
          Eigen::Vector2d(static_cast<double>(ax), static_cast<double>(ay)) *
          (config.max_acc / static_cast<double>(kControlDivisions));
        const int duration_steps = initial ? kStartDurationSteps : kDurationSteps;
        for (int step = 1; step <= duration_steps; ++step) {
          const double duration =
            config.max_tau * static_cast<double>(step) / static_cast<double>(duration_steps);
          Eigen::Vector4d next;
          next.head<2>() = current.state.head<2>() + current.state.tail<2>() * duration +
            0.5 * input * duration * duration;
          next.tail<2>() = current.state.tail<2>() + input * duration;
          if (!next.allFinite() ||
            next.tail<2>().cwiseAbs().maxCoeff() > config.max_vel ||
            ctx.pointBlocked(next.head<2>()))
          {
            continue;
          }
          const KinoKey key = toKey(next, ctx);
          if (key == toKey(current.state, ctx)) {
            continue;
          }
          const double cost = current.cost +
            (input.squaredNorm() + config.time_weight) * duration;
          const auto found = expanded.find(key);
          if (found != expanded.end() &&
            nodes[static_cast<std::size_t>(found->second)].cost <= cost)
          {
            continue;
          }
          CoefMat segment;
          segment << current.state.head<2>(), current.state.tail<2>(),
            input * 0.5, Eigen::Vector2d::Zero();
          if (!segmentWithinLimits(segment, duration, config) ||
            !segmentCollisionFree(segment, duration, ctx))
          {
            continue;
          }
          // 内存有界：到达上限立即终止，宁可失败也不无限扩张。
          if (nodes.size() >= config.max_nodes) {
            result.nodes = nodes.size();
            return result;
          }
          const int next_index = static_cast<int>(nodes.size());
          nodes.push_back({next, input, duration, cost, index});
          expanded[key] = next_index;
          const double estimate = estimateHeuristic(next, goal_state, ctx, time_to_goal);
          open.push({cost + config.heuristic_weight * estimate, next_index});
        }
      }
    }
  }

  result.nodes = nodes.size();
  return result;
}

}  // namespace navigation2::mpc

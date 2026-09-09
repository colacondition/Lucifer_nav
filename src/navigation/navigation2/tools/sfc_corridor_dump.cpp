// 走廊可视化数据导出工具（开发用，不参与运行链路）。
//
// 目的：把 SFC 走廊、前端路径、后端优化轨迹算出来并导出成 JSON，交给
// tools/render_corridor.py 渲染成图 —— 配色对齐 TDT-nav-kit 的示例图：
// 绿色 = 前端化简路径，蓝色框 = 走廊，红色 = 后端优化轨迹。
//
// 为什么单独一个可执行文件而不是加进导航节点：这是离线分析工具，输入是一张
// 静态地图和一对起终点，没有 ROS 图、没有时序，塞进节点只会给热路径添负担。
// 它只链接 navigation2 的库（Eigen + 本包），不引入 OpenCV 等新依赖。
//
// 用法：
//   ros2 run navigation2 sfc_corridor_dump <map.msgpack> <sx> <sy> <gx> <gy> <out.json>
//
// 输出的 JSON 是手写的（不引第三方 JSON 库），字段见文件末尾的 dumpJson()。

#include "distance_transform.hpp"
#include "grid_utils.hpp"
#include "minco/minco_optimizer.hpp"
#include "minco_time_allocation.hpp"
#include "semantic_map.hpp"
#include "semantic_map_consumer.hpp"
#include "sfc_corridor.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <queue>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace
{

constexpr double kRobotRadius = 0.25;   // 与 rm_global_costmap 同值
constexpr double kClearanceDesired = 0.4;
constexpr double kClearanceWeight = 3.0;
constexpr double kMapCostWeight = 20.0;
constexpr double kMapCostExponent = 2.0;
constexpr double kSfcMaxRange = 2.5;
constexpr int kObstacleThreshold = 100;

// 语义地图 → OccupancyGrid。与 rm_map_server 同一套推导：
// OBSTACLE→100、UNKNOWN→-1、其余（含 TUNNEL）→0。
nav_msgs::msg::OccupancyGrid toOccupancyGrid(const navigation2::SemanticMapData & data)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.width = static_cast<std::uint32_t>(data.geometry.width);
  grid.info.height = static_cast<std::uint32_t>(data.geometry.height);
  grid.info.resolution = static_cast<float>(data.geometry.resolution);
  grid.info.origin.position.x = data.geometry.origin.x();
  grid.info.origin.position.y = data.geometry.origin.y();
  grid.data.assign(data.terrain.size(), 0);
  for (std::size_t i = 0; i < data.terrain.size(); ++i) {
    switch (data.terrain[i]) {
      case static_cast<std::uint8_t>(navigation2::TerrainType::OBSTACLE):
        grid.data[i] = 100;
        break;
      case static_cast<std::uint8_t>(navigation2::TerrainType::UNKNOWN):
        grid.data[i] = -1;
        break;
      default:
        grid.data[i] = 0;
        break;
    }
  }
  return grid;
}

bool lethalCell(const nav_msgs::msg::OccupancyGrid & grid, int x, int y)
{
  if (x < 0 || y < 0 || x >= static_cast<int>(grid.info.width) ||
    y >= static_cast<int>(grid.info.height))
  {
    return true;
  }
  const int v = grid.data[static_cast<std::size_t>(y) * grid.info.width + x];
  return v >= kObstacleThreshold;
}

// 简化版 A*：代价 = 1 + map_cost_weight*(v/100)^exponent + clearance 惩罚。
// 与 rm_global_planner 的代价语义一致，但去掉 turn/隧道轴向项 —— 这里要的是
// 一条有代表性的前端路径，不是复刻规划器的全部工程细节。
std::vector<Eigen::Vector2d> planPath(
  const nav_msgs::msg::OccupancyGrid & grid, const Eigen::Vector2d & start,
  const Eigen::Vector2d & goal)
{
  const int w = static_cast<int>(grid.info.width);
  const int h = static_cast<int>(grid.info.height);
  const double res = grid.info.resolution;
  const double ox = grid.info.origin.position.x;
  const double oy = grid.info.origin.position.y;

  auto toCell = [&](const Eigen::Vector2d & p, int & cx, int & cy) {
      cx = static_cast<int>(std::floor((p.x() - ox) / res));
      cy = static_cast<int>(std::floor((p.y() - oy) / res));
      return cx >= 0 && cx < w && cy >= 0 && cy < h;
    };

  // clearance 场：精确 EDT，种子是致命格。
  std::vector<std::uint8_t> seeds(static_cast<std::size_t>(w) * h, 0U);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      if (lethalCell(grid, x, y)) {
        seeds[static_cast<std::size_t>(y) * w + x] = 1U;
      }
    }
  }
  const auto dist = navigation2::exactSquaredDistanceTransform(seeds, w, h);
  std::vector<float> clearance(static_cast<std::size_t>(w) * h, 0.0F);
  for (std::size_t i = 0; i < clearance.size(); ++i) {
    clearance[i] = static_cast<float>(
      navigation2::clearancePenalty(dist[i] * res, kClearanceDesired, kClearanceWeight));
  }

  int sx = 0, sy = 0, gx = 0, gy = 0;
  if (!toCell(start, sx, sy) || !toCell(goal, gx, gy)) {
    return {};
  }
  const int start_index = sy * w + sx;
  const int goal_index = gy * w + gx;

  std::vector<double> g_score(static_cast<std::size_t>(w) * h, 1e18);
  std::vector<int> parent(static_cast<std::size_t>(w) * h, -1);
  std::vector<std::uint8_t> closed(static_cast<std::size_t>(w) * h, 0U);
  using Entry = std::pair<double, int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;

  auto heuristic = [&](int index) {
      const int ax = index % w;
      const int ay = index / w;
      return std::hypot(static_cast<double>(ax - gx), static_cast<double>(ay - gy));
    };
  g_score[static_cast<std::size_t>(start_index)] = 0.0;
  open.push({heuristic(start_index), start_index});

  constexpr int dirs[8][2] = {
    {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

  bool found = false;
  while (!open.empty()) {
    const int current = open.top().second;
    open.pop();
    if (closed[static_cast<std::size_t>(current)] != 0U) {
      continue;
    }
    if (current == goal_index) {
      found = true;
      break;
    }
    closed[static_cast<std::size_t>(current)] = 1U;
    const int cx = current % w;
    const int cy = current / w;
    for (const auto & d : dirs) {
      const int nx = cx + d[0];
      const int ny = cy + d[1];
      if (nx < 0 || ny < 0 || nx >= w || ny >= h || lethalCell(grid, nx, ny)) {
        continue;
      }
      // 斜步禁止穿角（与规划器一致）。
      if (d[0] != 0 && d[1] != 0 &&
        (lethalCell(grid, cx + d[0], cy) || lethalCell(grid, cx, cy + d[1])))
      {
        continue;
      }
      const int ni = ny * w + nx;
      const int value = std::max(0, static_cast<int>(grid.data[static_cast<std::size_t>(ni)]));
      const double normalized = std::clamp(static_cast<double>(value) / 100.0, 0.0, 1.0);
      const double cell_cost =
        1.0 + kMapCostWeight * std::pow(normalized, kMapCostExponent);
      const double step = (d[0] != 0 && d[1] != 0) ? std::sqrt(2.0) : 1.0;
      const double tentative = g_score[static_cast<std::size_t>(current)] +
        step * cell_cost + clearance[static_cast<std::size_t>(ni)];
      if (tentative >= g_score[static_cast<std::size_t>(ni)]) {
        continue;
      }
      g_score[static_cast<std::size_t>(ni)] = tentative;
      parent[static_cast<std::size_t>(ni)] = current;
      open.push({tentative + heuristic(ni), ni});
    }
  }
  if (!found) {
    return {};
  }

  std::vector<Eigen::Vector2d> path;
  for (int at = goal_index; at != -1; at = parent[static_cast<std::size_t>(at)]) {
    const int ax = at % w;
    const int ay = at / w;
    path.emplace_back(ox + (ax + 0.5) * res, oy + (ay + 0.5) * res);
  }
  std::reverse(path.begin(), path.end());
  return path;
}

// 弦化简：贪心向前延伸，保证连线不穿致命格，并限制单段长度。
//
// 为什么要限制段长：不限长时一条直路会被压成两个点，走廊就只剩两个框，看不出
// 「走廊沿路径铺开」的效果（TDT 示例图里是一排框）。按 0.6 m 上限切分后，框沿
// 路径成排铺开，同时路标点仍然远少于 0.18 m 重采样的原始折线。
std::vector<Eigen::Vector2d> simplify(
  const std::vector<Eigen::Vector2d> & path, const nav_msgs::msg::OccupancyGrid & grid,
  double max_segment)
{
  if (path.size() < 3) {
    return path;
  }
  const double res = grid.info.resolution;
  const double ox = grid.info.origin.position.x;
  const double oy = grid.info.origin.position.y;
  auto blocked = [&](const Eigen::Vector2d & a, const Eigen::Vector2d & b) {
      const int x0 = static_cast<int>(std::floor((a.x() - ox) / res));
      const int y0 = static_cast<int>(std::floor((a.y() - oy) / res));
      const int x1 = static_cast<int>(std::floor((b.x() - ox) / res));
      const int y1 = static_cast<int>(std::floor((b.y() - oy) / res));
      for (const auto & cell : navigation2::raytraceLine(x0, y0, x1, y1)) {
        if (lethalCell(grid, cell.x, cell.y)) {
          return true;
        }
      }
      return false;
    };

  std::vector<Eigen::Vector2d> out;
  out.push_back(path.front());
  std::size_t anchor = 0;
  while (anchor + 1 < path.size()) {
    std::size_t best = anchor + 1;
    for (std::size_t candidate = anchor + 2; candidate < path.size(); ++candidate) {
      if ((path[candidate] - path[anchor]).norm() > max_segment) {
        break;
      }
      if (blocked(path[anchor], path[candidate])) {
        break;
      }
      best = candidate;
    }
    out.push_back(path[best]);
    anchor = best;
  }
  return out;
}

std::vector<Eigen::Vector2d> sampleTrajectory(const std::vector<Piece<5, 2>> & pieces)
{
  std::vector<Eigen::Vector2d> samples;
  for (const auto & piece : pieces) {
    const double dur = piece.getDuration();
    const int n = std::max(1, static_cast<int>(std::ceil(dur / 0.02)));
    for (int i = 0; i <= n; ++i) {
      const double t = dur * static_cast<double>(i) / static_cast<double>(n);
      samples.push_back(piece.getPos(t));
    }
  }
  return samples;
}

void writeVec2(std::ofstream & out, const std::vector<Eigen::Vector2d> & pts)
{
  out << "[";
  for (std::size_t i = 0; i < pts.size(); ++i) {
    if (i != 0) {
      out << ",";
    }
    out << "[" << pts[i].x() << "," << pts[i].y() << "]";
  }
  out << "]";
}

// 跑一次 MINCO 并采样。corridor_weight 是唯一在两组之间变化的参数。
std::vector<Eigen::Vector2d> runMinco(
  const std::vector<Eigen::Vector2d> & waypoints, const std::vector<double> & times,
  double corridor_weight, const navigation2::SemanticMap & semantic,
  const navigation2::TunnelRegionGrid & region)
{
  navigation2::MincoOptimizer optimizer;
  navigation2::MincoOptimizer::Params params;
  params.smooth_weight = 0.5;
  params.data_weight = 10.0;
  params.tunnel_axis_weight = 5.0;
  params.tunnel_corridor_weight = corridor_weight;
  params.obstacle_weight = 0.0;
  params.max_iterations = 800;
  optimizer.setParams(params);
  optimizer.setTunnelAxisQuery(
    [&semantic](const Eigen::Vector2d & pos, Eigen::Vector2d & axis) {
      return navigation2::tunnelAxisAtPoint(semantic, pos, axis);
    });
  optimizer.setTunnelCorridorQuery(
    [&region](const Eigen::Vector2d & pos, navigation2::TunnelCorridorFrame & frame) {
      navigation2::TunnelRegionGrid::CorridorFrame geo;
      if (!region.corridorFrameAtPoint(pos.x(), pos.y(), kRobotRadius, 0.0, geo)) {
        return false;
      }
      frame.centroid = geo.centroid;
      frame.dir = geo.dir;
      frame.half_len = geo.half_len;
      frame.half_width_inner = geo.half_width_inner;
      frame.lateral_outer = geo.lateral_outer;
      return true;
    });
  const auto pieces = optimizer.optimize(waypoints, times);
  std::printf(
    "  minco corridor_weight=%5.1f -> %zu pieces\n", corridor_weight, pieces.size());
  return sampleTrajectory(pieces);
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 7) {
    std::fprintf(
      stderr,
      "usage: %s <map.msgpack> <sx> <sy> <gx> <gy> <out.json> [corridor_weight]\n",
      argv[0]);
    return 2;
  }
  const std::string map_path = argv[1];
  const Eigen::Vector2d start(std::atof(argv[2]), std::atof(argv[3]));
  const Eigen::Vector2d goal(std::atof(argv[4]), std::atof(argv[5]));
  const std::string out_path = argv[6];
  const double corridor_weight = (argc >= 8) ? std::atof(argv[7]) : 20.0;

  navigation2::SemanticMapData data;
  try {
    data = navigation2::loadSemanticMap(map_path);
  } catch (const std::exception & ex) {
    std::fprintf(stderr, "load map failed: %s\n", ex.what());
    return 1;
  }
  const auto grid = toOccupancyGrid(data);
  std::printf(
    "map %ux%u res=%.3f origin=(%.2f,%.2f) tunnels=%zu\n", grid.info.width,
    grid.info.height, grid.info.resolution, grid.info.origin.position.x,
    grid.info.origin.position.y, data.tunnels.size());

  const auto front = planPath(grid, start, goal);
  if (front.empty()) {
    std::fprintf(stderr, "A* found no path\n");
    return 1;
  }
  const auto waypoints = simplify(front, grid, 0.6);
  std::printf("front path %zu pts -> %zu waypoints\n", front.size(), waypoints.size());

  // SFC 走廊：每个路标点一个最大无致命格方形（内缩 robot_radius）。
  navigation2::SfcCorridor sfc;
  navigation2::SfcCorridorParams sfc_params;
  sfc_params.max_range = kSfcMaxRange;
  sfc_params.robot_radius = kRobotRadius;
  sfc_params.obstacle_threshold = kObstacleThreshold;
  sfc_params.unknown_is_lethal = false;
  sfc.updateGrid(grid, sfc_params);
  std::vector<navigation2::SfcBox> boxes;
  boxes.reserve(waypoints.size());
  for (const auto & p : waypoints) {
    boxes.push_back(sfc.usableBoxAt(p.x(), p.y()));
  }

  // 隧道走廊几何（给渲染脚本画轴线与洞内走廊）。
  navigation2::InflationParams inflation;
  inflation.resolution = data.geometry.resolution;
  inflation.full_cost_radius_m = 0.10;
  inflation.cutoff_radius_m = 0.30;
  inflation.decay_rate_per_m = 24.0;
  inflation.non_body_magnitude_cap = navigation2::kMaxInflatedMagnitude;
  const auto semantic = navigation2::SemanticMap::inflate(data, inflation);
  const auto region = navigation2::TunnelRegionGrid::build(semantic, 0.20);

  // 两套 MINCO：隧道走廊项关闭 vs 打开。同一条路径、同一套时间分配，
  // 唯一的差别就是那一个权重 —— 这样图上的差异就是走廊项本身造成的。
  const auto times = navigation2::allocateMincoSegmentTimes(waypoints, 2.0, 0.1, 0.12);
  std::printf("real A* path:\n");
  const auto traj_off = runMinco(waypoints, times, 0.0, semantic, region);
  const auto traj_on = runMinco(waypoints, times, corridor_weight, semantic, region);

  // —— 受控实验：把洞内的路标点人为横移 0.2 m ——
  // 真实的 A* 路径本来就在隧道中线附近（A* 自带 clearance 代价 + 隧道轴向代价），
  // 所以走廊项几乎无事可做，上面那对轨迹只差 1 cm 左右。这不能说明走廊项没用，
  // 只能说明「前端已经给对了」。要看清它的作用，就得给一条偏轴的参考：把洞内
  // 路标点沿横向推 0.2 m（仍在影响区内），再看走廊项把轨迹拉回来多少。
  // 这是一个明确标注的对照实验，不是真实规划结果。
  constexpr double kDemoLateral = 0.20;
  auto demo_waypoints = waypoints;
  int shifted = 0;
  for (auto & p : demo_waypoints) {
    navigation2::TunnelRegionGrid::CorridorFrame geo;
    if (!region.corridorFrameAtPoint(p.x(), p.y(), kRobotRadius, 0.0, geo)) {
      continue;
    }
    const Eigen::Vector2d normal(-geo.dir.y(), geo.dir.x());
    p += kDemoLateral * normal;
    ++shifted;
  }
  std::printf("off-axis demo: shifted %d waypoints by %.2f m\n", shifted, kDemoLateral);
  const auto demo_times =
    navigation2::allocateMincoSegmentTimes(demo_waypoints, 2.0, 0.1, 0.12);
  const auto demo_off = runMinco(demo_waypoints, demo_times, 0.0, semantic, region);
  const auto demo_on =
    runMinco(demo_waypoints, demo_times, corridor_weight, semantic, region);

  std::ofstream out(out_path);
  if (!out) {
    std::fprintf(stderr, "cannot write %s\n", out_path.c_str());
    return 1;
  }
  out.precision(6);
  out << std::fixed;
  out << "{\n";
  out << "  \"map\": {\"width\": " << grid.info.width << ", \"height\": "
      << grid.info.height << ", \"resolution\": " << grid.info.resolution
      << ", \"origin\": [" << grid.info.origin.position.x << ", "
      << grid.info.origin.position.y << "]},\n";
  out << "  \"start\": [" << start.x() << "," << start.y() << "],\n";
  out << "  \"goal\": [" << goal.x() << "," << goal.y() << "],\n";
  out << "  \"robot_radius\": " << kRobotRadius << ",\n";
  out << "  \"front_path\": ";
  writeVec2(out, front);
  out << ",\n  \"waypoints\": ";
  writeVec2(out, waypoints);
  out << ",\n  \"boxes\": [";
  for (std::size_t i = 0; i < boxes.size(); ++i) {
    if (i != 0) {
      out << ",";
    }
    out << "[" << boxes[i].x0 << "," << boxes[i].y0 << "," << boxes[i].x1 << ","
        << boxes[i].y1 << "]";
  }
  out << "],\n  \"traj_corridor_off\": ";
  writeVec2(out, traj_off);
  out << ",\n  \"traj_corridor_on\": ";
  writeVec2(out, traj_on);
  out << ",\n  \"demo_lateral\": " << kDemoLateral;
  out << ",\n  \"demo_waypoints\": ";
  writeVec2(out, demo_waypoints);
  out << ",\n  \"demo_traj_off\": ";
  writeVec2(out, demo_off);
  out << ",\n  \"demo_traj_on\": ";
  writeVec2(out, demo_on);
  out << ",\n  \"tunnels\": [";
  bool first_tunnel = true;
  for (const auto & spec : data.tunnels) {
    if (!first_tunnel) {
      out << ",";
    }
    first_tunnel = false;
    out << "{\"clear_width\": " << spec.clear_width << ", \"run_up\": " << spec.run_up
        << "}";
  }
  out << "]\n}\n";
  std::printf("wrote %s\n", out_path.c_str());
  return 0;
}

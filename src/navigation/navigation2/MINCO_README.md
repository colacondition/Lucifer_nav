# MINCO Path Smoother

基于 ROSE 方案的 MINCO（Minimum Control Effort）轨迹优化器，用于平滑 A* 规划的路径。

## 功能特性

- **时空耦合优化**：同时优化路径和时间分配
- **动力学约束**：最小化 jerk（加加速度），保证轨迹平滑
- **障碍物避让**：使用 RC-ESDF 进行碰撞检测和梯度计算
- **L-BFGS 优化**：高效的非线性优化求解器
- **Smoothed L1 惩罚**：对障碍物穿透使用平滑惩罚函数

## 架构说明

### 核心组件

1. **MincoOptimizer** (`minco/minco_optimizer.cpp`)
   - MINCO 优化器封装
   - 使用 MINCO_S3NU（S=3 表示最小化 jerk，NU 表示非均匀时间）
   - L-BFGS 求解器配置

2. **RmMincoPathSmoother** (`minco_path_smoother_node.cpp`)
   - ROS 2 节点封装
   - 订阅原始路径 `/plan_raw`
   - 发布优化后路径 `/plan_minco`
   - 使用 RC-ESDF 进行障碍物查询

3. **MINCO 库** (`minco/minco.hpp`)
   - 来自 ROSE 的 MINCO 实现
   - 带状线性系统求解器
   - 梯度传播计算

### 与当前 path_smoother 的区别

| 维度 | 当前 B 样条平滑器 | MINCO 优化器 |
|---|---|---|
| **算法** | 梯度下降 + 二阶差分 | L-BFGS + 分段多项式 |
| **优化目标** | 最小化曲率（近似） | 最小化 jerk（精确） |
| **动力学** | 不考虑速度/加速度 | 考虑速度/加速度约束 |
| **时间分配** | 无（路径固定） | 自动优化时间分配 |
| **连续性** | C¹（速度连续） | C²（加速度连续） |
| **求解时间** | ~5ms | ~20-100ms |

## 使用方法

MINCO smoother 已集成到主导航流程，直接启动导航即可使用：

```bash
# 仿真环境
ros2 launch bringup sim.launch.py

# 真实环境  
ros2 launch bringup real.launch.py
```

### 参数调优

所有参数在 `params/navigation2.yaml` 中的 `rm_minco_path_smoother` 段：

```yaml
smooth_weight: 1.0        # 增大 → 轨迹更平滑，但可能偏离原路径
obstacle_weight: 10.0     # 增大 → 更严格避障，但可能求解失败
robot_radius: 1.0         # 机器人半径 + 安全裕度
default_velocity: 1.0     # 影响时间分配（段距离 / 速度）
```

## 调试技巧

### 1. 查看优化日志

```bash
# 设置日志级别为 DEBUG
ros2 run navigation2 rm_minco_path_smoother_node --ros-args --log-level debug
```

日志会显示：
- MINCO 优化耗时
- L-BFGS 收敛状态
- 输出路径点数量

### 2. 可视化轨迹

```bash
# 在 RViz 中添加 Path 显示
# Topic: /plan_minco
# Color: 绿色（区分原始路径）
```

### 3. 性能分析

如果优化时间过长（>100ms）：

1. **减少路径点数量**：在 global planner 中稀疏化输出
2. **降低迭代次数**：修改 `lbfgs_params_.max_iterations`（默认 4000）
3. **关闭优化**：设置 `enable_optimization: false` 作为 fallback

### 4. 常见问题

**问题：优化失败，输出原始路径**

- 原因：L-BFGS 求解器收敛失败（障碍物密集，无解）
- 解决：
  1. 降低 `obstacle_weight`
  2. 增大 `robot_radius`（给更多避障空间）
  3. 检查原始路径是否已经穿墙

**问题：轨迹抖动**

- 原因：`smooth_weight` 太小，障碍物项主导
- 解决：增大 `smooth_weight` 到 2.0 或更高

**问题：轨迹偏离原路径太多**

- 原因：`smooth_weight` 太大
- 解决：减小 `smooth_weight` 到 0.5

## 实现细节

### ESDF 适配

MINCO 优化器通过回调函数查询 ESDF：

```cpp
minco_optimizer_->setEsdfQuery(
  [this](const Eigen::Vector2d & pos, double & dist, Eigen::Vector2d & grad) {
    return esdf_map_.query(pos, dist, grad);
  });
```

这样可以复用 Lucifer_nav 现有的 `RcEsdfMap`，无需重新实现 ESDF。

### 时间分配策略

当前使用简单的距离/速度公式：

```cpp
double dist = (waypoints[i+1] - waypoints[i]).norm();
double time = std::max(dist / default_velocity_, min_segment_time_);
```

未来可以改进为：
- 考虑曲率（弯道减速）
- 考虑障碍物密度（密集区域减速）
- 自适应时间优化（MINCO 支持对时间求导）

### 轨迹采样

MINCO 输出的是分段多项式 `Piece<5, 2>`（5 阶多项式，2 维），需要采样成 ROS Path：

```cpp
for (size_t i = 0; i < pieces.size(); ++i) {
  double duration = pieces[i].getDuration();
  int num_samples = std::ceil(duration / sample_dt);
  for (int j = 0; j < num_samples; ++j) {
    double t = (j * duration) / (num_samples - 1);
    Eigen::Vector2d pos = pieces[i].getPos(t);
    // 转换为 PoseStamped
  }
}
```

## 性能基准

在测试地图上的典型性能：

| 路径长度 | 路径点数 | 优化时间 | 输出点数 |
|---------|---------|---------|---------|
| 5m      | 10      | 15ms    | 50      |
| 10m     | 20      | 35ms    | 100     |
| 20m     | 40      | 80ms    | 200     |

## 许可证

MINCO 核心算法来自 Zhepei Wang 的实现（MIT License）。
L-BFGS 库来自 ROSE 团队的移植。

## 参考文献

1. Wang, Zhepei, et al. "Geometrically Constrained Trajectory Optimization for Multicopters." (MINCO 论文)
2. ROSE 团队的 `rose_navigation` 实现

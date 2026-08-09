# MINCO 集成完成总结

## ✅ 已完成的工作

### 1. 核心代码移植
- ✅ 复制 ROSE 的 MINCO 库（`minco.hpp`, `trajectory.hpp`, `lbfgs.hpp`, `root_finder.hpp`）
- ✅ 创建 `MincoOptimizer` 类，适配 Lucifer_nav 的 `RcEsdfMap`
- ✅ 创建 `RmMincoPathSmoother` ROS 2 节点
- ✅ 编译成功，注册为可组合节点

### 2. 导航流程集成
- ✅ **替换** `launch/bringup.launch.py` 中的 `RmPathSmoother` → `RmMincoPathSmoother`
- ✅ **替换** `params/navigation2.yaml` 中的配置
- ✅ 输入输出话题保持不变：
  - 输入：`/plan_raw` (来自 global planner)
  - 输出：`/plan` (输入到 MPC controller)

### 3. 参数配置
```yaml
rm_minco_path_smoother:
  smooth_weight: 1.0          # jerk 最小化权重
  obstacle_weight: 10.0       # 障碍物惩罚权重
  robot_radius: 0.25          # 与 global_costmap 一致
  default_velocity: 1.5       # 时间分配参考速度
  esdf_resolution: 0.05       # 距离场分辨率
```

## 🎯 使用方式

**无需任何额外操作**，直接启动导航即可：

```bash
# 仿真环境
ros2 launch bringup sim.launch.py

# 真实环境
ros2 launch bringup real.launch.py
```

MINCO smoother 会自动在导航容器中启动，处于以下位置：
```
Global Planner → MINCO Smoother → MPC Controller
   (/plan_raw)      (/plan)
```

## 📊 预期效果

### 相比旧的 B 样条平滑器

**优势：**
- ✅ 轨迹更平滑（C² 连续，加速度连续）
- ✅ 动力学可行（最小化 jerk）
- ✅ 时空耦合优化（考虑速度和时间分配）

**代价：**
- ⚠️ 计算时间增加：5ms → 20-100ms
- ⚠️ 可能失败：障碍物密集时优化无解（会回退到原始路径）

## 🔧 参数调优建议

### 如果轨迹太曲折
→ **增大** `smooth_weight` 到 2.0-5.0

### 如果经常穿墙
→ **增大** `obstacle_weight` 到 20.0-50.0  
→ **增大** `robot_radius` 到 0.3-0.5

### 如果优化经常失败
→ **减小** `obstacle_weight` 到 5.0  
→ 检查 A* 输出的原始路径是否已经穿墙

### 如果计算太慢（>100ms）
→ 修改 `minco_optimizer.cpp` 的 `lbfgs_params_.max_iterations`（默认 4000）  
→ 或在 A* planner 中增大 `path_resample_distance` 减少路径点数

## 🐛 回退到旧平滑器

如果 MINCO 出现问题，可以快速回退：

### 方法 1：修改启动文件
编辑 `launch/bringup.launch.py`：
```python
# 改回
nav_component('navigation2::RmPathSmoother', 'rm_path_smoother', start_path_smoother),
```

### 方法 2：修改参数文件
编辑 `params/navigation2.yaml`，把 `rm_minco_path_smoother` 改回 `rm_path_smoother`

## 📁 文件清单

```
src/navigation/navigation2/
├── src/
│   ├── minco/
│   │   ├── minco.hpp              # MINCO 核心算法
│   │   ├── trajectory.hpp         # 轨迹表示
│   │   ├── lbfgs.hpp              # L-BFGS 优化器
│   │   ├── root_finder.hpp        # 根查找工具（trajectory.hpp 依赖）
│   │   ├── minco_optimizer.hpp    # 优化器封装
│   │   └── minco_optimizer.cpp
│   ├── minco_path_smoother_node.cpp  # ROS 2 节点
│   └── path_smoother_node.cpp     # 旧节点（保留作为备份）
├── launch/
│   └── bringup.launch.py          # ✏️ 已修改
├── params/
│   └── navigation2.yaml           # ✏️ 已修改（包含 MINCO 配置）
└── MINCO_README.md                # 详细文档
```

## 🚀 下一步

1. **真实测试**：在实际地图上测试 MINCO 的平滑效果和计算时间
2. **参数调优**：根据机器人的实际动力学调整 `smooth_weight`
3. **性能评估**：记录优化耗时，确保在 100ms 以内
4. **失败处理**：观察优化失败的场景，考虑添加降级策略

## 💡 技术细节

### ESDF 适配原理
```cpp
// MINCO 通过回调查询 RC-ESDF
minco_optimizer_->setEsdfQuery(
  [this](const Eigen::Vector2d & pos, double & dist, Eigen::Vector2d & grad) {
    return esdf_map_.query(pos, dist, grad);  // 复用现有实现
  });
```

### 时间分配策略
```cpp
// 根据路径点间距离估算时间
double dist = (waypoints[i+1] - waypoints[i]).norm();
double time = std::max(dist / default_velocity_, min_segment_time_);
```

### 障碍物惩罚函数
```cpp
// Smoothed L1: 穿透越深，代价越大
penetration = robot_radius - esdf_distance;
cost = smoothed_l1(penetration, mu=0.4);
```

---

**集成完成！** MINCO path smoother 现已成为 Lucifer_nav 的默认路径平滑器。

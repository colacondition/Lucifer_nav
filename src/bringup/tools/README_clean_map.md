# clean_map_dynamic.py —— 离线清理 PCD 先验图里的动态残影

`fast_location` 用先验 PCD 做 GICP 配准。如果这张图是建图时人/车/其他动态物体
留下的残影（同一位置积累的一坨点时有时无），GICP 会把它当成真实结构，导致定位
在残影附近被拉偏甚至误配准。本工具用**建图时多个关键帧的可见性/自由空间证据**
离线删掉这类残影，思路同 TDT 的 `clean_map_bundle`（本项目为纯 Python + numpy
重写，不依赖 open3d/scipy）。

## 前提：先把关键帧落盘

small_glim 只有在 `mapping.save_raw_mapping_frames: true` 时才会在每个关键帧写：

- `frame_<idx>.pcd`：该关键帧的 `filtered_cloud_imu`（**IMU 系**，已按
  `cloud_range_min/max` 滤过距离）；
- `poses.txt`：一行一个 `se3(tx,ty,tz,qx,qy,qz,qw)`，即 `T_world_imu`。

第 `i` 行位姿对应 `frame_<i>.pcd`（两者在同一个关键帧里先后写出）。

launch 已接好开关（默认关）：

```bash
# 仿真建图并保存关键帧（关键帧写到 map_save_dir，默认 install/.../PCD）
ros2 launch bringup sim.launch.py mode:=mapping save_keyframes:=True
# 实车同理
ros2 launch bringup real.launch.py mode:=mapping save_keyframes:=True
```

建图结束后 `map_save_dir` 里会多出 `frame_*.pcd` 和 `poses.txt`，以及最终地图
`<world>.pcd`。注意 `poses.txt` 每次启动 small_glim 都会重建（截断重写），
`frame_<idx>.pcd` 也被同名覆盖；复用旧目录时如残留 index 比本次位姿行数大的
旧帧，工具会告警并跳过。

## 用法

```bash
python3 src/bringup/tools/clean_map_dynamic.py \
    --map   install/bringup/share/bringup/PCD/RMUL.pcd \
    --keyframes install/bringup/share/bringup/PCD \
    --output /tmp/RMUL_clean.pcd \
    --removed /tmp/RMUL_removed.pcd
```

建议先把清理结果目检后再替换正式地图：

```bash
# CloudCompare 同时打开原图 / clean / removed，removed 应为「人形/车形」一坨
cloudcompare install/bringup/share/bringup/PCD/RMUL.pcd /tmp/RMUL_clean.pcd /tmp/RMUL_removed.pcd
# 确认后备份再替换（注意 --symlink-install 下 install 里的 PCD 可能是指向源码的符号链接）
cp install/bringup/share/bringup/PCD/RMUL.pcd /tmp/RMUL_backup.pcd
cp /tmp/RMUL_clean.pcd install/bringup/share/bringup/PCD/RMUL.pcd
```

输出是 binary PCD（`FIELDS x y z`），`pcl::io::loadPCDFile<pcl::PointXYZI>` 可直接读。

## 原理

1. **逐关键帧建球形深度缓冲**：把该帧 IMU 系点云投到「方位角 × 俯仰角」网格
   （默认 0.5°，720 × 360 格），每格只记最近距离（即该方向上看到的第一个表面）。
   空格用邻域最小深度补洞（半径 2 格，只填空格，不动已有值），避免稀疏点云
   漏掉自由空间证据；用「邻域最小」是为了保守——补进来的深度只会更小。
2. **对地图点逐帧取证**：把地图点按 `T_world_imu` 转到该帧 IMU 系，取它所在
   方向的深度 `d`：
   - `|dist - d| <= occ_tol`：该帧真的看到了它 → `observed += 1`；
   - `dist < d - free_margin`：它浮在该帧观测面的前方（原本是空气/后来被人占了
     的位置）→ `free += 1`；
   - 其余不计：更远说明被别的物体挡住（遮挡不算自由空间），该方向没观测也不计。
3. **判定**：`observed < min_observed` 且 `free >= min_free` 才删除。动态残影
   只被极少关键帧直接看到（人走过去就没），却被大量关键帧判为「浮在自由空间」；
   真实静态面要么在许多帧里被直接观测到，要么被挡（不计 free）。
4. **保守性**：没有任何证据（两计数都为 0）的点一律保留，所以雷达量程外、没去过
   的区域不会被误删。地图点先用 `--voxel` 聚成体素代表点再取证（控制百万点 ×
   上百关键帧的内存/耗时），代表点被判删时整个体素一起删；输出仍保留原始点，
   不做重采样。地图按 `--chunk` 分批处理，内存与地图点数无关。

## 参数

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `--map` | 必填 | 输入世界系地图 PCD（ascii / binary 均可，binary_compressed 需先转 binary） |
| `--keyframes` | 必填 | 关键帧目录（含 `frame_<idx>.pcd` + `poses.txt`） |
| `--output` | 必填 | 清理后地图 PCD 输出路径（binary xyz） |
| `--removed` | 无 | 可选，被删点 PCD 输出路径，便于目检 |
| `--voxel` | `0.05` | 证据计算的体素降采样粒度（米）；`<=0` 表示逐点、不降采样 |
| `--occ-tol` | `0.15` | 判「观测到」的距离容差（米） |
| `--free-margin` | `0.30` | 判「自由空间」时须比观测面明显更近的裕量（米） |
| `--min-observed` | `2` | 观测次数 **小于** 该值才可能删（默认即「只看到 0/1 次」） |
| `--min-free` | `3` | 自由空间证据次数 **达到** 该值才删 |
| `--max-range` | `20` | 关键帧点云参与证据的最大距离（米），超出视为无量程证据 |
| `--chunk` | `200000` | 每个证据批次处理的地图代表点数，控制峰值内存 |

调参直觉：

- 残影删不干净：先看统计里的 observed 直方图。若建图关键帧很密
  （`keyframe_trans_thresh: 0.1` 量级），人走过一个位置会被好几个关键帧直接
  观测到（observed 可能 3~5），此时**提高 `--min-observed`**（如 5）比降低
  `--min-free` 更对症；代价是观测次数少的稀疏静态点更容易被误删。
- 也可以适当降低 `--min-free`（如 2）或放宽 `--free-margin`（如 0.2），
  但会更容易误删稀疏静态点；
- 静态结构被误删：提高 `--min-free`、收紧 `--free-margin`；
- 跑得太慢：加大 `--voxel`（如 0.1）或减小 `--max-range`。

工具结束时打印输入/保留/删除点数、删除率、observed/free 直方图与分阶段耗时，
建议把这些统计和 CloudCompare 目检一起作为是否替换地图的依据。

> 注意：如果没有任何点被删，`--removed` 写出的是 0 点 PCD，`pcl::io::loadPCDFile`
> 会拒绝读 0 点文件（打印 `No points to read` 并返回 -1），这是 PCL 的行为，不是
> 文件格式错误；此时 clean 输出与原图点数一致。

## 局限

- **遮挡与无观测区域**：工具只认「关键帧里真的有深度」的方向。被雷达量程、
  视野、家具长期挡住的区域没有证据，会被原样保留（残影也可能留下）；这类
  残影需要换路线补建图或手工删。
- **关键帧太少 / 覆盖不够**：关键帧只覆盖了残影出现的那几个视角时，`free`
  证据不足（默认要 3 次），残影删不掉。建图时把残影所在区域从不同方向多走几遍
  效果最好；本工具默认参数偏保守，宁可少删。
- **关键帧太密**：`min_observed` 判的是「这个位置被直接看到几次」。small_glim
  默认关键帧间距只有 0.1m，人走过一个位置往往被连续好几个关键帧看到，残影的
  observed 可能达到 3~5，默认 `min_observed=2` 就删不掉；这种建图要相应提高
  `--min-observed`（例如 5），并目检确认没有误删稀疏静态点。
- **稀疏静态点可能误删**：如果某个静态点只被观测 1 次、又恰好在多帧里落在
  粗糙的 0.5° 角网格边界上被判成 free，可能被删。默认 `min_free=3` +
  `min_observed=2` 已尽量压住，仍建议目检 `--removed`。
- **地面/贴地残影边界**：贴地的动态物体底部与地面相接，其后方地面距离很接近，
  这部分点通常判不成 free（保守），残影底部可能残留一圈。
- **位姿质量**：poses.txt 是建图时的 `T_world_imu`，若这段建图本身漂移/闭环后
  的轨迹与写盘关键帧不一致，证据会错位。建议用重建后正常闭合的建图结果。
- 清理后**一定要先 CloudCompare 目检、备份原图，再替换 `src/bringup/PCD/<world>.pcd`**；
  本工具不做自动备份。
- 这是离线工具，不参与 `ros2 run`（未注册成 ROS 可执行）；测试见
  `src/bringup/test/test_clean_map_dynamic.py`（合成房间 + 动态方块，
  删除率/保留率断言）。

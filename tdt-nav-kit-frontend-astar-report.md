# tdt-nav-kit 前端路径搜索技术报告

阅读范围（全文精读）：`src/YAstar/yastar.hpp`(366 行)、`src/YAstar/yastar.cpp`(959)、`src/YAstar/kinodynamicAstar.hpp`(73)、`src/YAstar/kinodynamicAstar.cpp`(290)、`doc/Astar.md`(92)、`main.cpp`(498)；补充 `README.md`、`doc/Usage.md`、`CMakeLists.txt`。本机 `/usr/include/opencv4/opencv2/opencv.hpp:43` 已确认 `OPENCV_ALL_HPP` 就是该头文件的 include guard。

---

## 1. 公开 API 与数据结构（接入成本）

**依赖**
- Eigen3 必需（`yastar.hpp:14`）；`TMatrix<T>` = 行主序动态矩阵（`yastar.hpp:56`）。
- OpenCV 可选但"事实上必需"：`yastar.hpp:15-17` 用 `__has_include(<opencv2/opencv.hpp>)` 探测，实现里用 `#ifdef OPENCV_ALL_HPP` 分支（`yastar.hpp:68,136`；`yastar.cpp:508,553`）。只要 OpenCV 头在 include path 上，`getSDF()` 就自动走 `cv::distanceTransform`（`yastar.cpp:559`）；否则走自研 Felzenszwalb EDT（`yastar.cpp:563-745`）。
- 前端**不依赖 OSQP**，OSQP/OsqpEigen 只服务后端 MinimumSnap。

**类与构造**
- `struct Mask`（`yastar.hpp:21-49`）：位运算多层掩码。
- `class YAstar`（`yastar.hpp:53`）：`YAstar()` 与 `YAstar(int width,int height,float mapping,float originx,float originy)`（`yastar.hpp:77-78`）。
- `class KinodynamicAstar : public YAstar`（`kinodynamicAstar.hpp:11`），`using YAstar::YAstar;` + `using YAstar::search;`（12-14）→ 同一对象同时拥有栅格搜索和动力学搜索两个重载。

**关键结构体**
- `YAstar::Node`（`yastar.hpp:269-283`）：`float cost, estim; int parent; float speedx, speedy; bool closed;`，`getCostTotal()=cost+estim`。约 24 B/节点（560×300=168k 格 → ~4.0 MB）。`speedx/speedy` 在普通 A* 中从未写入（死字段）。
- `CompareNode`（`yastar.hpp:286-295`）：持 `nodeMap` 的 const 引用，比较 `getCostTotal()`；构造参数用 `std::_Placeholder<1>` 占位 —— libstdc++ 内部类型，不可移植。
- `Mask::MaskMap`（`yastar.hpp:22-25`）：`data` 为 `vector<u64>`，`shape={height,width,num_layers}`，每 64 层共用一个 64 位字。
- `KinodynamicAstar::Config`（`kinodynamicAstar.hpp:17-26`）：`maxVel=4, maxAcc=4, maxTau=2, velResolution=0.5, timeWeight=1, heuristicWeight=8, sampleTime=0.1, maxNodes=10000`。
- `Sample{State state; Vector2d acceleration; double time;}`（28-32）、`Result{bool success; vector<Sample> trajectory; size_t nodes, iterations;}`（34-39）、`State = Eigen::Vector4d`（15）。

**核心签名**
- `void setMap(int w,int h,u_char* mapData)`（102）——**0 表示障碍**；
- `void setMap(int w,int h,float mapping,float ox,float oy,std::vector<int8_t>& data)`（111）——**0 表示可行、非 0 表示占用**（实现 `yastar.cpp:425-431`）。两者语义相反。
- `template<Func> setCostField(float radius=1.f, Func&& decay=[](float x){return 1./x;})`（181-182，实现 331-348）。
- `initCostMap(bool sparse=true)`（175）；`search(Eigen::Vector2f, Eigen::Vector2f) -> std::vector<Eigen::Vector2f>`（233）。
- 后处理：`simplifyPath / simplifyPathDP / simplifyPathHypot / densifyPath / lineInObsticle / pointInObsticle / findSavePoint / getLength`（193-227）。
- 掩码：`setMaskNumLayers / setMaskMap / setMaskMapTimed / resetMask`（117-134、187-191）。
- 动力学：`Result search(const Vector2d& start, const Vector2d& startVel, const Vector2d& startAcc, const Vector2d& end)`（`kinodynamicAstar.hpp:48-49`）。

**接入成本判断**：坐标体系只是"米 ↔ 栅格"的平移+缩放（`transformPos`，`yastar.cpp:883-891`，**无旋转**）。只要地图是行主序、0/非 0 二值、已知 resolution+origin，接入就是 `构造 → setMap → setCostField → initCostMap → search`。若要摆脱 OpenCV，注意 fallback 用 `std::span`（C++20）。

---

## 2. 普通 A* 完整实现细节（`yastar.cpp:802-880`）

- **open/closed**：`std::priority_queue<size_t, std::vector<size_t>, CompareNode> openList`（813），存节点线性索引；比较器**实时**读 `nodeMap` 的 `cost+estim`，所以"改键"后不重压堆也能被看见。没有独立 closed 表，用 `Node::closed` 就地标记（852）。无 reserve、无复用。
- **8 邻域**：`constexpr int neighour[8][2] = {{-1,0},{1,0},{0,-1},{0,1},{-1,-1},{-1,1},{1,-1},{1,1}}`（854），取用 `nx=x+neighour[i][1], ny=y+neighour[i][0]` → 前 4 个是四邻域，后 4 个是对角。**没有多分辨率、没有 JPS**（`doc/Astar.md:84` 提到的 JPS 在仓库里不存在）。
- **代价函数**：`newCost = nodeMap(y,x).cost + costMap(ny,nx) * costWeight * (i<4 ? 1.f : 1.414f)`（864）。没有拐弯/换向代价，也没有方向状态。
- **启发函数**：`nd.estim = hypotf(nx-endx, ny-endy)`（867-869），单位是"栅格"，**不乘 costWeight 也不乘 costMap**。由于 `costMap ≥ 1`（基础代价恒为 `1+f(d)`），每步代价 ≥ 几何步长 → h 可采纳且一致；但势场把 g 抬到 1~4.3 倍后 h 相对极小，障碍物附近退化为 Dijkstra 行为。普通 A* 没有 heuristicWeight。
- **不保证最优**：`if(nd.closed) continue;`（861-862）不重开；配合"就地改键 + 不重排堆"，弹出顺序不再严格 f 最小，节点可能以次优 g 被关闭后永不重开。
- **终止与回溯**：弹出时判 `x==endx && y==endy`（834），沿 `parent` 回溯（837-849）。
- **无解/越界返回 `{start}`**（820、824、879），而头文件注释（`yastar.hpp:232`）写"返回空数组表示无解"——注释与实现矛盾；`main.cpp:128` 的 `if(path.empty())` 永不成立。

**代价地图与势场**
- `initCostMap(sparse=true)`（760-800）：sparse 分支先 `getSDF()`，然后 `costMap(i,j)=decayFunction(dist)+1.f`，`dist<1e-5 → +inf`（768-773）。公式即 `cost = 1 + f(d)`；`main.cpp:109` 传 `f(x)=0.5/(0.1+x)` → d=0.05 m 处 cost≈4.33，d=1 m 处≈1.45，d≥2 m 处≈1.24。
- **sparse 模式下 `setCostField` 的半径参数被完全忽略**：SDF 是全图精确欧氏距离，f 在全图生效；`_funcInflateRadius` 只在 `sparse=false` 的 biasMask 分支用（331-348 预计算偏移、787-795 逐格扫描，偏移按代价降序排序，第一个命中障碍物的偏移即最近障碍 → 等价于在离散圆盘上取 max f(d)）。
- **障碍判定统一**：`isInObstacle(x,y) = mask.masked(x,y) || occMap(y,x) > occThs`（936-943），`occThs` 默认 0.5（309）。
- **掩码层作用**：`masked()` 把该格所有层按位或（60-67）；优先级 mask > occMap。`setMaskNumLayers(4)`=256 层 → 168k×4×8 B ≈ 5.4 MB。`setMaskMapTimed`（451-506）实现 `name@id` 的 TTL 槽位复用。
- **膨胀处理：没有几何膨胀**。安全完全由软势场承担，硬障碍就是原始占用栅格；对角线扩展也不检查两个正交邻格（无 corner-cutting 抑制），窄缝处可能擦角。

**路径后处理**
- `simplifyPath(path, threshold=0.1)` → `simplifyPathDP`（195-244）：经典 Douglas-Peucker，但分裂判据是 `maxDistance>threshold || isInObsticle(path[index0], path[index1])`（220）——**把"首尾直线穿障碍"并入分裂条件**，简化后天然无碰撞。`lineInObsticle` 是 Bresenham 逐格查障碍（151-183）。
- `simplifyPathHypot`（246-276）：贪心弦优化，从 current 不断外推 next，`lineInObsticle` 失败就落点。`main.cpp:132-135` 先 DP 再弦优化。
- `Tools::simplifyPath/DP`（318-352）是不查地图的纯几何版；`densifyPath` 用 Bresenham 还原稠密栅格点（278-305）。

**是否支持增量/多次查询**：不支持增量（无 D* Lite、无复用）；每次 `search` 全量 reset `nodeMap`（947-949）并重建 open 表。多次查询可重复调用，但每次都要付全量重置代价（见 §4）。

---

## 3. Kinodynamic A* 完整实现细节

- **状态空间**：`State=[x,y,vx,vy]`；`Key = array<int,4>`（hpp:52），`toKey` 用 `floor` 离散位置、`round` 离散速度（cpp:15-20）→ "位置栅格 × 速度格点"。
- **控制输入采样**（cpp:193-219）：若 `index==0 && (|startVel|≠0 || |startAcc|≠0)`，只用给定 `startAcc`、沿时间取 20 段（`duration = maxTau*step/20`）；否则枚举 9×9=81 个加速度 `(ax,ay)∈[-4..4]² × maxAcc/4`，每个再配 3 个时长 `maxTau*step/3` → **每节点最多 243 次扩展**。加速度是**各轴独立限幅**，不是二范数限幅。
- **运动原语**：解析积分 `stateTransit`（32-37）`p=p0+v0t+½at², v=v0+at`；检查时把它写成三次多项式 `coef << p0, v0, input*0.5, 0`（207）→ 与终点连接段共用同一套多项式表示。
- **时间步长**：单段最长 `maxTau=2 s`；输出采样 `sampleTime=0.1 s`（`main.cpp:401` 改为 0.01）。
- **约束检查** `checkTrajectory`（39-70）：各轴查加速度端点 `|2b|`、`|6aT+2b| ≤ maxAcc`，速度端点 `|c|`、`|3aT²+2bT+c| ≤ maxVel`，再求速度极值点 `t=-b/(3a)`（47-52）；采样密度 `samples = ceil(duration*maxVel*√2/(mapping*0.5))`（55），每步都做 `pointInObstacle` + `lineInObsticle(previous, point)`（64）——半格采样 + 逐段 Bresenham。
- **代价**：`cost = parent.cost + (|a|² + timeWeight) * duration`（204），LQMT 风格混合量纲。
- **启发函数**：**不是无障碍 A*，也不是欧氏距离**，而是闭式 LQMT 估计。`estimateHeuristic`（103-124）对 `∫a²dt + timeWeight·T` 求极小，得四次方程 `timeWeight·T⁴ + c3T² + c2T + c1 = 0`，`c1=-36·dp·dp, c2=24(v0+v1)·dp, c3=-4(v0²+v0·v1+v1²)`（107-110），用闭式 `quartic/cubic`（226-290）求根，下界 `max(1e-3, maxAbs(dp)/(maxVel*0.5))`（111），代价 `-c1/(3T³)-c2/(2T²)-c3/T+timeWeight·T`（117）；再乘 `heuristicWeight=8.0` 压入 f（162、217）→ 强加权，快但次优。
- **末端条件**（172）：`(pos-end).cwiseAbs().maxCoeff() <= 12*mapping`（切比雪夫 12 格 ≈ 0.6 m），随后用 estimateHeuristic 的最优时间构造三次多项式直连终点；终点速度强制为 0（147），`computeShotTraj`（89-101）解满足两端位置+速度的 3 次系数（98-99）并过同一套 `checkTrajectory`。**没有位置/速度容差**：连上即精确命中（`main.cpp:423` 用 `error<1e-6` 校验）；连不上不拉黑，继续扩展。
- **轨迹存储与拼接**：节点只存 `{state, input, duration, cost, parent}`（hpp:56-61），**不存多项式**；成功时回溯 parent 链（176-178），再逐段用 `coef << nodes[parent].state.head<2>(), nodes[parent].state.tail<2>(), node.input*0.5, 0`（182-184）重建恒加速原语并按 `sampleTime` 重采样，最后接终点连接段（187）。`sampleTrajectory`（72-87）以 `trajectory.back().time` 为偏移，`i=1..count` 只补新点，边界不重复。
- **与普通 A* 的关系**：只复用地图与碰撞层（`occMap`、`mask`、`isInObstacle`、`lineInObsticle`、`mapping`、`originPos`、`mapLocker`），**不复用 nodeMap/costMap/open 表**；用局部 `vector<PathNode> nodes`（`reserve(maxNodes)`，155-156）+ `unordered_map<Key,int> expanded` + `priority_queue<pair<double,int>, vector, greater>`。**kino 搜索完全不读代价地图 → 没有势场/clearance 概念，只避硬障碍**。
- 节点数达 `maxNodes` 直接返回失败（209-212）；弹出前用 `expanded[toKey(...)] != index` 做陈旧条目剪枝（169）。

---

## 4. 性能相关

- **每次 search 都重建代价地图**：`resetedCostMap` 只在 `initCostMap` 里置 true（761），而 search 开头把它置 false（811）→ 下一次调用必然重跑 `getSDF()+initCostMap()`（806-808）。README 里 1604 µs 的"生成代价地图"是**每帧都付**的，且 `main.cpp:107-111` 已手动调过一次，等于重复计算。同理 `reseted` 每次置 false → 每帧全量重写 `nodeMap`（`reset()` 947-949，560×300×24 B ≈ 4 MB）。
- **分配模式**：每次 search 新建 `priority_queue`（底层 vector 无 reserve）；`initCostMap` 里 `getSDF()` 分配 1 个 N 浮点矩阵 + `getMap()` 的 u_char 副本；`mask.pushMask` 每层全图 168k 次写（109-118）。
- **复杂度**：普通 A* O(N log N)（N=扩展节点数），邻居 8 次/节点，代价地图 O(N)。SDF 走 OpenCV 是 O(N)；走自研 Felzenszwalb 也是 O(N)，但**每行每列各新建 4 个临时 vector**（688-691、716-719），常数极差。
- **kino A* 成本**：每节点 ≤243 个子节点，每个子节点都要 `checkTrajectory`，采样数 `ceil(duration*maxVel*√2/(mapping*0.5))`；`duration=2 s, maxVel=4, mapping=0.05` 时约 452 次"点 + 线段"碰撞检查 → 单节点最坏 ~10⁵ 次碰撞查询。这解释了 `doc/Astar.md:90` 的"全局搜索约 2 秒"。`main.cpp:400` 还把 `maxNodes` 提到 150000、`sampleTime` 降到 0.01（后者只影响输出采样，不影响搜索成本）。
- **README 时间数据对应的环节**（README:99-113 ↔ `main.cpp` 的 bench 名）：Astar 7627 µs = `astar search`（114-116，仅 search 本身）；生成代价地图 1604 µs = `set cost map`（107-111，setCostField+initCostMap）；化简路径 34 µs = `simplify path`（132-135）；Minimum Snap 1255 µs 属后端。测试机 AMD 7735H / GCC 11.2，`CMakeLists.txt` 只有 `-O2 -Wall`（无 `-march=native`、无 OpenMP、无 LTO）。另 `yastar.hpp:52` 注释：300×560 时 costmap≈3 ms、search≈17 ms。

---

## 5. 代码质量与可复用性

- **OpenCV 耦合**：仅 3 处（`Tools::fillPoly`、`setMaskPoly`、`getSDF` 的 `cv::distanceTransform`），全部用 `#ifdef OPENCV_ALL_HPP` 隔离，**核心搜索逻辑不碰 OpenCV**，这层做得干净。
- **与 main.cpp**：无纠缠，main 只用公开 API；但示例里塞了 Y 轴翻转（81-91）、`*20` 像素缩放、`cv::waitKey(0)` 阻塞，不能直接当生产模板。
- **硬编码常量**：`12*mapping`（终点邻域）、`±4 / maxAcc/4`（加速度档）、`20/3`（时间档）、代价里 `a²` 的 `0.5`、启发下界 `maxVel*0.5`、采样密度 `√2 / (mapping*0.5)`、SDF 障碍阈值 `1e-5`、对角 `1.414f`、`occThs=0.5`、`heuristicWeight=8.0`、掩码字宽 64。多数没进 Config（Config 只管 kino）。
- **线程安全**：`mapLocker` 只覆盖 `search/setMap/getCostMap/getMap/reset`；`setMaskMap`、`setMaskMapTimed`、`resetMask`、`setMaskNumLayers`、`setCostField` 与各 setter **完全不锁**。search 全程持锁（`yastar.cpp:809`、`kinodynamicAstar.cpp:128`）→ 同对象两次搜索串行，地图更新会被一次 7 ms 的搜索阻塞。`reseted/resetedCostMap` 是 atomic 但读写都在锁内，属冗余。
- **命名/注释**：中文注释详尽（含公式与取舍，如 275 行"我有一个主意，把 speedx/speedy 也加入代价"、864 行"依赖分支预测"），但拼写错误进了 API：`lineInObsticle`/`pointInObsticle`/`findSavePoint`（应为 Obstacle/Safe）；部分注释与实现不符（"返回空数组表示无解"）。
- **死代码**：`Node::speedx/speedy` 未用；`biasMask` 里的 `int bias` 计算后未用（340）；`checkTrajectory` 的 `outputSamples` 仅用于有限性检查（56-58）；`getCostMapImage` 里注释掉的 clamp。

---

## 6. 相比"OccupancyGrid 上跑 A* + 距离场 clearance"的独到之处（逐条）

1. **势场即代价（cost = 1 + f(d)）而非附加 clearance 惩罚项**（`yastar.cpp:772`；`doc/Astar.md:36-40`）：A* 主循环无需额外项，f 通过模板 lambda 注入（`yastar.hpp:181`），零虚函数开销。比"length + w/clearance"更省事，且能自由调"贴墙 vs 绕远"。
2. **位图多层掩码（64 层/u64，OR 判定）**（`yastar.hpp:21-49`、`yastar.cpp:60-67`）：一层临时障碍只占 168k bit ≈ 21 KB，可叠很多层，判定是一次 64 位 OR 循环；比多张 u_char 地图逐张判断更省内存、更 cache 友好。
3. **带 TTL 的掩码槽位池 `setMaskMapTimed`**（`yastar.cpp:451-506`）：`name@id` 编码 + 过期槽位复用 + 超限覆盖最旧 → 前端内置"短期记忆障碍"，调用者不必管生命周期。
4. **SDF 与势场解耦，稀疏/非稀疏双路径**（`yastar.cpp:552-747` vs `760-800`）：障碍稀疏时一次全图精确 EDT；局部小地图/稠密时用预排序圆盘偏移扫描。两套成本模型可按场景选。
5. **DP 化简把碰撞判据并入分裂条件**（`yastar.cpp:220`）：常规 DP 只按几何偏差分裂，简化后可能穿墙；这里 `maxDistance>threshold || lineInObsticle(...)` 使简化结果天然无碰撞，省掉"简化→检查→回插"的循环。
6. **弦优化补充 DP**（`yastar.cpp:246-276`）：DP 保留"离直线最远"的点，容易留下抖动；弦优化按方向贪心前推，进一步压点，两者串联（`main.cpp:132-135`）是实用组合。
7. **kino A* 与栅格 A* 共享地图层与碰撞原语**（继承 + `isInObstacle`/`lineInObsticle`）：一套地图/掩码同时服务"全局通路（栅格 A*）"和"近端动力学轨迹（kino）"，正对应 `doc/Astar.md:82-86` 的混合搜索建议，集成时不用维护两套地图。
8. **终点用闭式 3 次多项式"打靶"**（`kinodynamicAstar.cpp:172-190、89-101`）：把"精确到点 + 精确零速"交给解析解，避免在 4 维状态格里搜到"接近但不精确"的终点，也免去终端容差调参。
9. **启发用 LQMT 闭式解（四次方程）而非欧氏距离/无障碍 A***（`kinodynamicAstar.cpp:103-124`）：h 内含 v0/v1 与 timeWeight，天然编码"当前速度越快、剩余时间越短"，比欧氏距离强得多，且不必额外跑一遍 A*。
10. **恒加速解析积分 + 三次多项式统一表示**（`32-37`、`89-101`、`207`）：扩展、终点连接、采样重放三处共用同一套系数与检查代码，代码量极小。
11. **碰撞检查"自适应半格采样 + 逐段 Bresenham"**（`55`、`64`）：采样密度由 `duration*maxVel*√2/(mapping*0.5)` 自适应，慢速段不浪费采样、快速段不漏障碍。
12. **检查速度/加速度极值而非只看端点**（`43-52`）：对三次多项式速度求 `t=-b/(3a)` 极值点，避免"端点满足、中间超限"的假通过——这是 kino 可行性检查最易漏的一步。

---

## 7. 接入前必须注意的坑（按优先级）

1. `setMap(..., u_char*)` 与 `setMap(..., vector<int8_t>&)` 的 0 语义**相反**（`yastar.cpp:411-413` vs `425-431`）。
2. `search` 失败/越界返回 `{start}` 而非空数组（820/824/879），与头文件注释矛盾。
3. 每次 `search` 都会重建 `costMap` 与 `nodeMap`（806-811 + 761）；地图没变时建议自行管理该标志或改逻辑。
4. **`setMaskMap` 不会把 `resetedCostMap` 置 false**（442-449）→ 推完掩码直接 `search`，掩码区域在 `costMap` 里仍可能是有限代价，A* 可能穿过去。必须手动 `initCostMap()`。
5. 无几何膨胀、无 corner-cutting 抑制，只有软势场 → 贴墙/擦角风险；接入时要么加膨胀掩码，要么补正交格检查。
6. kino 搜索不读代价地图 → 其轨迹可能贴墙；需要 clearance 得自行后过滤或改 `checkTrajectory`。
7. `CompareNode` 使用 libstdc++ 内部类型 `std::_Placeholder<1>`（`yastar.hpp:288`），MSVC/libc++ 不可移植。
8. 无 OpenCV 时 `getSDF` 的 fallback 用 `std::span`（`yastar.cpp:681`）→ 需 C++20（本机已验证 C++17 下 `std::span` 不可用）；`CMakeLists.txt` 未指定标准。
9. 不保证最优：closed 不重开（861）+ 就地改键不重排堆（813、871）。
10. 线程安全只覆盖少数入口，mask/costField 的 setter 无锁。

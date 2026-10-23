#pragma once

#include <optional>
#include <string>

#include "decision/types.hpp"

namespace decision
{

// 自瞄发来的原始观测。
//
// 话题 `auto_aim_target_pos`（`std_msgs/String`，**RELIABLE** QoS），载荷是 4 个
// double 用 `std::to_string` 拼成的字符串 `"x,y,valid,id"`。
// 来源：sp_vision_25 的 `tasks/omniperception/decider.cpp:190`。
//
//   x, y   目标**装甲板中心**在**云台系**下的位置（米）。
//          ⚠️ 随云台转动而变 —— 所以必须用云台的位姿去旋转它，不能直接当 map 坐标。
//   valid  1 = 有目标。**没有目标时上游照常发、只是整帧全 0**，所以只能看这一位，
//          不能靠"没收到消息"判断目标丢失。
//   id     发布的机器人编号 = `ArmorName + 1`，见 publishedIdName()。
//
// ⚠️ 第三个数本来是 z，但上游用常量 1 把它顶掉了（`decider.cpp:200`），拿不到高度。
//    对区内追击的影响是厘米级（误差 = |z|·sin(云台俯仰)），可以接受。
//
// ⚠️ 上游是**单帧 PnP、未滤波**（取的是原始检测列表，EKF 结果没发），
//    所以不能一帧就用 —— 见 TargetInputConfig::min_valid_sec。
struct TargetObservation
{
  double x{0.0};
  double y{0.0};
  int published_id{0};
};

// 解析载荷。
//
// 返回 nullopt 表示"这一帧不可用"，它同时覆盖两种情况：
//   - 格式错（字段数不对、不是数字）
//   - 上游明确说没有目标（valid=0 / 全 0）
// 两者对下游是同一个含义（这一拍没有可用目标），所以不区分。
// 想区分"自瞄节点是不是还活着"，由订阅端自己数消息条数。
std::optional<TargetObservation> parseTargetPayload(const std::string & payload);

// 发布编号 -> 可读名字，**只用于日志**。
//
// ⚠️ 这套编号和裁判系统的 `robot_id` **不是一回事**：
//    这里是 `ArmorName + 1` —— 1=英雄 2=工程 3/4/5=步兵 6=哨兵 7=前哨站 8=基地
//    而裁判协议里哨兵的 robot_id 是 7。**不要交叉引用这两套号。**
std::string publishedIdName(int published_id);

// ==================== 输入判据 ====================

struct TargetInputConfig
{
  // 连续有效这么久才认这个目标。上游是单帧 PnP，一帧就用会被误检带跑。
  double min_valid_sec{0.5};
  // 这么久没有有效帧就放弃
  double lost_timeout_sec{1.5};
  // 一阶滤波系数（0..1）：越大跟得越快、越抖
  double filter_alpha{0.35};
  // 目标点比上次下发移动超过这么多就重发
  double reissue_move_m{0.4};
  // 至少隔这么久才重发（防止把规划器打爆）
  double reissue_sec{0.5};
  // 目标离车太近就**整个不用**它（避免原地抖）。
  //
  // ⚠️ 有了 standoffGoal 的保持距离圆之后，默认设成 0（关掉）：敌人贴脸时我们想要的
  // 行为是"退到 1.5m 圈上"，而不是"放弃这个目标"。留着这个参数只是为了以后想过滤
  // 极近的误检时有地方可调。
  double min_range_m{0.0};
};

struct TargetInputState
{
  bool usable{false};       // 现在能不能用于追击
  double x{0.0};            // 滤波后的云台系坐标
  double y{0.0};
  int published_id{0};
  double range_m{0.0};      // 云台系下的水平距离
};

// 目标输入的全部判据：解析之后怎么滤波、什么时候算"稳定可用"、什么时候放弃。
//
// 纯逻辑、不碰 ROS，所以能脱离节点单测 —— 这些判据飘了之后的现场表现是
// 「哨兵朝一个不存在的方向冲」或者「目标明显在却不动」，都是没有报错的故障。
class TargetInput
{
public:
  explicit TargetInput(TargetInputConfig config = {});

  // 喂一帧。obs == nullopt 表示这一拍没有可用目标（没收到 / 格式错 / 上游无目标）。
  // 时间用**到达时刻** —— 上游消息没有 header，没有别的时间可用。
  void update(const std::optional<TargetObservation> & obs, double now_sec);

  const TargetInputState & state() const { return state_; }

  // 这一拍检测到目标换了一个（published_id 变了）。调用方据此重置自己的滤波/状态。
  // 注意它是"本拍事件"，下一拍自动清零。
  bool idChanged() const { return id_changed_; }

  // 该不该重发目标点。传**夹取之后**的 map 系目标点。
  bool shouldReissue(double x, double y, double now_sec) const;

  // 记下"已经按这个点下发过"。
  void markReissued(double x, double y, double now_sec);

private:
  // 按当前时刻重算 usable。丢帧窗口内也要走它 —— 不能因为"这一拍没数据"就把
  // 一个还在 lost_timeout 容忍期内的目标判成不可用（那样超时参数就白设了）。
  void refreshUsability(double now_sec);

  TargetInputConfig config_;
  TargetInputState state_;
  bool id_changed_{false};
  bool filtered_{false};
  double valid_since_sec_{-1.0e9};
  double last_valid_sec_{-1.0e9};
  double last_reissue_sec_{-1.0e9};
  double last_reissue_x_{0.0};
  double last_reissue_y_{0.0};
};

// ==================== 换算到 map ====================

// 云台系下的目标 -> map 系位置。
//
// 三件事加起来让这个式子成立：
//   1. `map -> base_link` 这个 TF **装的是雷达位姿**，不是车体位姿
//      （`small_glim_node.cpp:345`："the base_link odometry actually carries the lidar pose"）
//   2. 雷达与云台共 yaw 轴；雷达与 IMU 都平放朝前、无 rpy
//      ⇒ 雷达系 = 云台系 = x前 y左 z上 ⇒ 安装旋转 = 单位阵
//   3. 雷达与云台共轴 ⇒ 雷达原点到旋转中心的平移只有竖直分量 ⇒ 对 XY 零影响
//      （自瞄的 (x,y) 原点是**云台旋转中心**，见其 readme 手眼标定章节）
//
// 于是只需要 p_map = t + R · (x, y, 0)，全程不碰任何 IMU 积分 ⇒ 零漂移。
//
// ⚠️ 传进来的必须是**雷达/云台**的位姿。**不要**传 `base_link_fake` ——
//    它的 yaw 被人为改成"路径切向"了，拿它旋转会得到完全错误的方向。
Pose targetToMap(const Pose & lidar_pose_in_map, double x_gimbal, double y_gimbal);

// 在"敌人周围 standoff_m 的圆"上取目标点：从敌人朝**机器人方向**退 standoff_m 的那一点。
//
// 为什么不能直接以敌人为目标点：**离太近就打不到了**（自瞄需要一定距离才能稳定命中）。
// 这个式子同时管两种情况：
//   机器人离得远 → 目标点在"机器人→敌人"连线上、距敌人 standoff_m ⇒ 推进，但停在圈上
//   机器人已经太近 → 同一个式子 ⇒ 自动退回圈上
//
// 边界：机器人正好与敌人重合（|d|≈0）时方向无定义 → 返回敌人位置本身（调用方还会夹取）。
Pose standoffGoal(const Pose & enemy, const Pose & robot, double standoff_m);

}  // namespace decision

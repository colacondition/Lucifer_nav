// 云台状态可视化节点（实车 / 仿真通用）。
//
// rm_tunnel_posture 只发一个「切换命令」（/gimbal_posture），真正知道云台现在
// 收没收下来的是电控的持续回传（/gimbal_posture_state）。这节点把「实态」和
// 「命令」都画在车上方，一眼能看出云台现在在哪：
//
//   * 方块 —— 实态：绿 = 收下（低，进洞安全），红 = 立着（高，进洞会撞顶板）；
//     方块 z 高度随实态上下移动，是收是放看位置就知道，不用认颜色。
//   * 两行文字 —— 上：当前实态（GIMBAL DOWN/UP）；下：当前命令（CMD DOWN/UP）。
//     命令先翻、实态慢半拍才跟上的过程，正是电控的执行延迟（仿真里由
//     simulated_gimbal 的 action_delay 模拟），RViz 里能看到这一段滞后。
//
// 为什么放 navigation2 而不是 simulated_gimbal：显示跟「谁来假装电控」无关，
// 实车仿真都要看，所以跟着导航容器一起起。RViz 的 MarkerArray 显示块订阅
// marker_topic。
//
// 文字用 ASCII 而不是中文：RViz 的 OGRE 文本渲染对 CJK 支持很差，中文会渲染成
// 方框。状态语义由颜色承担（绿=收下、红=立着）。

#include <string>

#include <decision_interfaces/msg/gimbal_posture.hpp>
#include <decision_interfaces/msg/gimbal_posture_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace navigation2
{

class RmGimbalVisualizer : public rclcpp::Node
{
public:
  explicit RmGimbalVisualizer(const rclcpp::NodeOptions & options)
  : Node("rm_gimbal_visualizer", options)
  {
    robot_base_frame_ = declare_parameter<std::string>("robot_base_frame", "base_link_fake");
    posture_state_topic_ =
      declare_parameter<std::string>("posture_state_topic", "/gimbal_posture_state");
    posture_topic_ = declare_parameter<std::string>("posture_topic", "/gimbal_posture");
    marker_topic_ = declare_parameter<std::string>("marker_topic", "/gimbal_status");

    // 状态型话题，与 serial_driver / 电控一致用 transient_local + reliable：
    // 晚起的可视化（或 MPC）也立刻拿到当前值，而不是干等到下一帧。
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    state_sub_ = create_subscription<decision_interfaces::msg::GimbalPostureState>(
      posture_state_topic_, qos,
      [this](decision_interfaces::msg::GimbalPostureState::SharedPtr msg) {
        lowered_ = msg->lowered;
        publishMarkers();
      });
    cmd_sub_ = create_subscription<decision_interfaces::msg::GimbalPosture>(
      posture_topic_, qos,
      [this](decision_interfaces::msg::GimbalPosture::SharedPtr msg) {
        lower_cmd_ = msg->lower;
        publishMarkers();
      });

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      marker_topic_, qos);

    // 开机先画一版（立着 = 电控/MPC 的初始值），晚起的 RViz 立刻有得看。
    publishMarkers();

    RCLCPP_INFO(
      get_logger(),
      "rm_gimbal_visualizer ready: state=%s cmd=%s out=%s frame=%s",
      posture_state_topic_.c_str(), posture_topic_.c_str(), marker_topic_.c_str(),
      robot_base_frame_.c_str());
  }

private:
  void publishMarkers()
  {
    visualization_msgs::msg::MarkerArray arr;
    // 时间戳用 0 而不是 now()：这个节点在导航容器里 use_sim_time=true，而 RViz
    // 没用仿真时钟（sim.launch.py 的 rviz2 节点没传 use_sim_time）。带仿真时间戳的
    // marker 在 RViz 里按系统时钟查 TF 会失败，显示块报红 Status Error。stamp=0
    // 让 RViz 用最新 TF，marker 实时跟着 base_link_fake 走 —— 跟车指示器的标准写法。
    const auto stamp = rclcpp::Time(0);

    // 方块：实态。z 高度随实态上下移动，绿=收下、红=立着。
    visualization_msgs::msg::Marker cube;
    cube.header.frame_id = robot_base_frame_;
    cube.header.stamp = stamp;
    cube.ns = "gimbal_status";
    cube.id = 0;
    cube.type = visualization_msgs::msg::Marker::CUBE;
    cube.action = visualization_msgs::msg::Marker::ADD;
    cube.pose.position.z = lowered_ ? 0.45 : 0.95;
    cube.scale.x = 0.4;
    cube.scale.y = 0.3;
    cube.scale.z = 0.25;
    cube.color.r = lowered_ ? 0.0 : 1.0;
    cube.color.g = lowered_ ? 1.0 : 0.0;
    cube.color.b = 0.0;
    cube.color.a = 1.0;
    arr.markers.push_back(cube);

    // 实态文字（用 ASCII，见文件头注释）。
    visualization_msgs::msg::Marker state_text;
    state_text.header = cube.header;
    state_text.ns = "gimbal_status";
    state_text.id = 1;
    state_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    state_text.action = visualization_msgs::msg::Marker::ADD;
    state_text.pose.position.z = 1.7;
    state_text.scale.z = 0.35;
    state_text.text = lowered_ ? "GIMBAL: DOWN" : "GIMBAL: UP";
    state_text.color = cube.color;
    arr.markers.push_back(state_text);

    // 命令文字：云台该往哪边去（rm_tunnel_posture 的目标）。
    visualization_msgs::msg::Marker cmd_text;
    cmd_text.header = cube.header;
    cmd_text.ns = "gimbal_status";
    cmd_text.id = 2;
    cmd_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    cmd_text.action = visualization_msgs::msg::Marker::ADD;
    cmd_text.pose.position.z = 1.35;
    cmd_text.scale.z = 0.25;
    cmd_text.text = lower_cmd_ ? "CMD: DOWN" : "CMD: UP";
    cmd_text.color.r = 0.9;
    cmd_text.color.g = 0.9;
    cmd_text.color.b = 0.9;
    cmd_text.color.a = 1.0;
    arr.markers.push_back(cmd_text);

    marker_pub_->publish(arr);
  }

  std::string robot_base_frame_;
  std::string posture_state_topic_;
  std::string posture_topic_;
  std::string marker_topic_;
  bool lowered_{false};
  bool lower_cmd_{false};

  rclcpp::Subscription<decision_interfaces::msg::GimbalPostureState>::SharedPtr state_sub_;
  rclcpp::Subscription<decision_interfaces::msg::GimbalPosture>::SharedPtr cmd_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

}  // namespace navigation2

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(navigation2::RmGimbalVisualizer)

// 固定线程数的组件容器。
//
// Humble 自带的 component_container_mt 不接受任何线程数参数（位置参数与
// --thread-count 均被忽略），executor 线程数恒为 hardware_concurrency()。
// 在核数很多的机器（本机 24 核）上会白白起 24 个 executor 线程。本可执行文件
// 的第一个位置参数指定线程数（默认 2），其余参数照常是 ros args。
//
// 用法：nav_container_mt [线程数] [--ros-args ...]
// 与 component_container_mt 的差异只有线程数可配，加载/卸载组件的服务
// 与 ComponentManager 行为完全一致。
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/component_manager.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  const std::vector<std::string> non_ros_args =
    rclcpp::remove_ros_arguments(argc, argv);
  std::size_t num_threads = 2;
  if (non_ros_args.size() > 1) {
    try {
      num_threads = std::stoul(non_ros_args[1]);
    } catch (const std::exception &) {
      throw std::invalid_argument(
        "nav_container_mt: first positional argument must be a thread count");
    }
  }
  if (num_threads == 0) {
    num_threads = 1;
  }

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
    rclcpp::ExecutorOptions(), num_threads);
  auto node = std::make_shared<rclcpp_components::ComponentManager>(executor);
  executor->add_node(node);

  // 兜底：executor 回调里任何未捕获异常都会沿 spin() 抛出。不接住的话进程
  // 直接 SIGABRT（线上出现过：action 目标竞态抛 runtime_error → 容器死亡 →
  // 地图/代价图/点云全部消失）。接住后干净退出，launch 的 respawn 会 2s 内
  // 重建整个容器（组件从文件/参数重新加载，无不可恢复状态）。
  try {
    executor->spin();
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      rclcpp::get_logger("nav_container"),
      "Executor exception, exiting for respawn: %s", ex.what());
  } catch (...) {
    RCLCPP_ERROR(
      rclcpp::get_logger("nav_container"),
      "Executor unknown exception, exiting for respawn");
  }

  rclcpp::shutdown();
  return 0;
}

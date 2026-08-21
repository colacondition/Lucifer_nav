// 感知链固定线程数组件容器。
//
// Humble 自带的 component_container_mt 不接受线程数参数，executor 线程数恒为
// hardware_concurrency()。本机 24 核时会起 24 个 executor 线程，而感知链只有
// lidar_filter + ground_segmentation 两个节点、单帧流水线很短，绝大多数线程
// 都在空转。本可执行文件的第一个位置参数指定线程数（默认 2；bringup 传入 1），
// 的 perception_threads 参数传入），其余参数照常是 ros args。
//
// 用法：perception_container_mt [线程数] [--ros-args ...]
// 与 component_container_mt 的差异只有 executor 线程数可配，组件加载/卸载
// 服务与 ComponentManager 行为完全一致。
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

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
        "perception_container_mt: first positional argument must be a thread count");
    }
  }
  if (num_threads == 0) {
    num_threads = 1;
  }

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
    rclcpp::ExecutorOptions(), num_threads);
  auto node = std::make_shared<rclcpp_components::ComponentManager>(executor);
  executor->add_node(node);

  // 兜底：executor 回调里任何未捕获异常都会沿 spin() 抛出。干净退出，launch 的
  // respawn 会在 2s 内重建容器（组件从参数重建，无不可恢复状态）。
  try {
    executor->spin();
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      rclcpp::get_logger("perception_container"),
      "Executor exception, exiting for respawn: %s", ex.what());
  } catch (...) {
    RCLCPP_ERROR(
      rclcpp::get_logger("perception_container"),
      "Executor unknown exception, exiting for respawn");
  }

  rclcpp::shutdown();
  return 0;
}

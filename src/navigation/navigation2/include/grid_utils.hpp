#ifndef RM_NAVIGATION2__GRID_UTILS_HPP_
#define RM_NAVIGATION2__GRID_UTILS_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/time.hpp>

/*

 ██╗  ██╗  ██████╗  ███╗   ██╗  ██████╗  ██╗       ██████╗  ███╗   ██╗  ██████╗ 
 ██║  ██║ ██╔═══██╗ ████╗  ██║ ██╔════╝  ██║      ██╔═══██╗ ████╗  ██║ ██╔════╝ 
 ███████║ ██║   ██║ ██╔██╗ ██║ ██║  ███╗ ██║      ██║   ██║ ██╔██╗ ██║ ██║  ███╗
 ██╔══██║ ██║   ██║ ██║╚██╗██║ ██║   ██║ ██║      ██║   ██║ ██║╚██╗██║ ██║   ██║
 ██║  ██║ ╚██████╔╝ ██║ ╚████║ ╚██████╔╝ ███████╗ ╚██████╔╝ ██║ ╚████║ ╚██████╔╝
 ╚═╝  ╚═╝  ╚═════╝  ╚═╝  ╚═══╝  ╚═════╝  ╚══════╝  ╚═════╝  ╚═╝  ╚═══╝  ╚═════╝ 

*/

namespace navigation2
{

struct GridCell
{
  int x{0};
  int y{0};
};

double normalizeAngle(double angle);

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q);

geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw);

bool inBounds(const nav_msgs::msg::OccupancyGrid & grid, int x, int y);

std::size_t gridIndex(const nav_msgs::msg::OccupancyGrid & grid, int x, int y);

bool worldToMap(
  const nav_msgs::msg::OccupancyGrid & grid, double world_x, double world_y, int & map_x,
  int & map_y);

void mapToWorld(
  const nav_msgs::msg::OccupancyGrid & grid, int map_x, int map_y, double & world_x,
  double & world_y);

bool isOccupied(int8_t value, int occupied_threshold, bool unknown_is_occupied);

void inflateOccupancyGrid(
  nav_msgs::msg::OccupancyGrid & grid, double inflation_radius, int occupied_threshold = 50);

void applyInflationCostGradient(
  nav_msgs::msg::OccupancyGrid & grid, double inflation_radius, int occupied_threshold = 50,
  double cost_scaling_factor = 8.0);

std::vector<GridCell> raytraceLine(int x0, int y0, int x1, int y1);

geometry_msgs::msg::Point transformPoint(
  const geometry_msgs::msg::TransformStamped & transform, double x, double y, double z);

geometry_msgs::msg::Point transformPointPlanar(
  const geometry_msgs::msg::TransformStamped & transform, double x, double y, double z);

geometry_msgs::msg::PoseStamped makePose(
  const std::string & frame_id, const rclcpp::Time & stamp, double x, double y, double yaw);

}  // namespace navigation2

#endif  // RM_NAVIGATION2__GRID_UTILS_HPP_

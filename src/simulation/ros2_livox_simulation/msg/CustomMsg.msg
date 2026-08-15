# Livox publish pointcloud msg format.
# 原定义来自 livox_ros_driver2（该官方驱动包已移除，实车由 mid360_driver 直发
# PointCloud2）；此处仅保留给仿真插件仿真官方驱动的话题兼容层。

std_msgs/Header header    # ROS standard message header
uint64 timebase           # The time of first point
uint32 point_num          # Total number of pointclouds
uint8  lidar_id           # Lidar device id number
uint8[3]  rsvd            # Reserved use
CustomPoint[] points      # Pointcloud data

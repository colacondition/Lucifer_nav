# 一条隧道的物理尺寸与通行参数。见 navigation2/include/semantic_map.hpp。

# 净高（m）。底盘变形后的车体高度必须小于它。
float64 clear_height
# 净宽（m）。决定隧道格用多大的膨胀半径。
float64 clear_width
# 入口前必须完成变形的距离（m）。
float64 run_up
# 洞内速度窗（m/s）。velocity_max 为 0 表示不限。
float64 velocity_min
float64 velocity_max

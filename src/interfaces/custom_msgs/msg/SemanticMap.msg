# 语义导航地图：地形标签 + 方向场 + 离线膨胀好的代价场。
#
# 几何和三个通道放在同一条消息里而不是分成几个话题：接收端不可能拿到互相错位的
# 语义层和占据层。rm_map_server 用 transient_local 发一次，晚启动的订阅者也能收到。

std_msgs/Header header

# 栅格几何。与 nav_msgs/MapMetaData 的约定一致：origin 是栅格 (0,0) 的角点。
# 这里不复用 MapMetaData 是因为它带一个本格式不支持的 yaw。
uint32 width
uint32 height
float64 resolution
float64 origin_x
float64 origin_y

# 地形标签，行主序，长度 = width * height。取值见 semantic_map.hpp 的 TerrainType：
# 0=FLAT 1=OBSTACLE 2=TUNNEL 7=UNKNOWN。
uint8[] terrain

# 方向场角度，0~255 映射到 0~2π。隧道轴线是无向的，± 等价。
uint8[] direction_angle

# 方向场模长，0~255 映射到 0~1。
#   > 242 (0.95)  地形本体，车体正位于该地形上，方向约束是硬的
#   <= 229 (0.9)  膨胀圈，只表示「附近有方向地形」，用于软塑形
# 中间是留给量化误差的空档。
uint8[] direction_magnitude

# 离线膨胀好的障碍代价，0~255。物理距离衰减，连续可导，给 MINCO / MPC 当梯度源。
# 注意这跟 OccupancyGrid 的 0~100/-1 不是一套刻度。
uint8[] cost

# 地图里的隧道，按 tunnel_ids 里的值 - 1 索引。
TunnelSpec[] tunnels

# 每格所属的隧道 id + 1，0 表示不属于任何隧道。为空时所有隧道格共用 tunnels[0]。
uint8[] tunnel_ids

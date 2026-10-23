# RoboMaster 2026 通信协议 0x0201 机器人性能体系数据（按 RMUL 3v3 筛过）
#
# 只装哨兵自身的状态。比赛阶段在 GameStatus。
uint8 ROBOT_ID_SENTRY = 7

uint8 robot_id          # 3v3 编制固定为哨兵 7（hero 1 / infantry 3 / sentry 7）
uint16 current_hp       # 自身血量（0x0201 偏移 2）

bool is_attacked

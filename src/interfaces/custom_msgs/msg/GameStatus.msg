# RoboMaster 2026 通信协议 0x0001 比赛状态数据（按 RMUL 3v3 筛过）
#
# game_type 是 3v3 时的值应为 4，可用作一次「接的是不是 3v3 那套规则」的校验。
# 线上一个字节同时装 game_type（低 4 bit）和 game_progress（高 4 bit），
# 原样透传、在这侧拆位。
uint8 GAME_TYPE_RMUC = 1        # 超级对抗赛
uint8 GAME_TYPE_RMUL_3V3 = 4    # 高校联盟赛 3V3 对抗

uint8 game_type
uint8 game_progress             # 0 未开始 / 1 准备 / 2 十五秒自检 / 3 五秒倒计时 / 4 比赛中 / 5 结算中
uint16 stage_remain_time        # 当前阶段剩余时间（秒）

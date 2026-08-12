# 云台姿态回传。电控持续报告云台**实际**在哪个姿态，与 GimbalPosture（导航侧的请求）配对。
#
# 为什么单独一条消息而不是给 GimbalPosture 加个字段：请求和实测是两个方向的量，混在一
# 条消息里会分不清「谁写的这个字段」—— 导航读回自己发出去的值当成到位确认，是这类接口
# 最容易出的错，而且表现是「一切正常，只是云台没收就进洞了」。
#
# 注意极性：线上电控发 0 表示低、1 表示高，这里的 lowered 已经转过来了（线上 0 → true）。
# 转换只在 serial_driver 里做一次（isGimbalLowered），理由见 packet.hpp 的注释。
#
# 谁在用：rm_mpc_controller。请求收云台而这里还是 false 时它停车等，等到 true 才继续。
bool lowered

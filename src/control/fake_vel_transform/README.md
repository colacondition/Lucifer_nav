# fake_vel_transform

西南石油大学（南充）泓龙战队哨兵导航工程中的底盘速度坐标转换节点。

该包用于处理哨兵“雷达 / 云台共 yaw 轴”时的导航兼容问题：即使云台或底盘处于旋转扫描状态，也尽量让局部规划器输出的速度能够稳定转换到底盘执行坐标系。

## 核心思路

节点会创建一个虚拟坐标系 `base_link_fake`：

- `base_link_fake` 的位置跟随 `base_link`。
- `base_link_fake` 的 yaw 方向参考局部路径 `/local_plan` 的前方姿态。
- 导航控制链路先输出并平滑中间速度 `/cmd_vel`，节点再将其转换为底盘可执行的 `/cmd_vel_chassis`。

这样可以让底盘小陀螺或云台旋转时，导航链路仍更接近“沿路径前进”的控制效果。

## 节点接口

### 订阅

- `/cmd_vel` (`geometry_msgs/msg/Twist`): 速度平滑后的中间导航速度指令。
- `/local_plan` (`nav_msgs/msg/Path`): 局部控制器输出的局部路径，用于估计期望朝向。
- TF: 查询 `map` 到 `base_link` 的变换，用于计算真实底盘方向。

### 发布

- `/cmd_vel_chassis` (`geometry_msgs/msg/Twist`): 转换到底盘执行坐标系后的最终速度指令。
- TF: 发布 `base_link` 到 `base_link_fake` 的虚拟坐标变换。

## 参数

- `spin_speed`: 底盘旋转速度，默认 `-6.0`。配合电控固定小陀螺时，可按实车方向调整正负号。
- `angular_deadband`: 角速度死区，默认 `0.05`。低于该值时不额外叠加小陀螺角速度。
- `min_translate_speed_for_spin`: 触发小陀螺叠加的最小平移速度，默认 `0.15`。

## 启动

通常由 `bringup` 统一拉起；也可以单独启动：

```sh
source install/setup.bash
ros2 launch fake_vel_transform fake_vel_transform.launch.py use_sim_time:=True
```

实车运行时按需要设置 `use_sim_time:=False`。

## 联调提示

- 如果底盘运动方向与预期相反，优先检查电控坐标约定和 `spin_speed` 正负号。
- 如果 `/cmd_vel_chassis` 没有输出，检查 `/local_plan`、`/cmd_vel` 和 `map -> base_link` TF 是否存在。
- 如果路径跟踪抖动明显，先降低控制器速度上限，再调整 `angular_deadband` 与 `min_translate_speed_for_spin`。

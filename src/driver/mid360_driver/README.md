# Mid-360 Driver

This is an implementation of the Mid-360 driver, intended to serve as a replacement for [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2).

<img src="./img/ACE.jpg" width="200px">

## Supported models

**Mid-360 与 Mid-360s**，可直接混接（多雷达话题按源 ip 区分）。

两款雷达共用同一份 Livox 点云/IMU 数据协议：端口相同（点云 56300→host 56301，
IMU 56400→host 56401）、包头与点格式相同（data_type 0/1/2/3，默认只推 0x01）、
出厂默认推送目标相同（上位机 192.168.1.50，雷达 192.168.1.1XX = SN 尾两位）。
官方 livox_ros_driver2 对 Mid-360s 的支持也只是把新设备类型（dev_type 35）走
与 Mid-360 完全相同的解析路径，本驱动按源地址收流、不依赖设备类型，因此无需
区分即可接收两种雷达。

Mid-360s 相对 Mid-360 新增的能力都在控制面指令上（本驱动为纯被动收流，不涉及）：

- ESC 转速模式（正常/慢速，慢速会降低点云包率）；
- IMU 量程配置：输出率 50~500Hz、加速度计 4~32G、陀螺 15.625~2000dps —— 若用
  Livox Viewer 2 / SDK 改过默认量程，注意同步检查 `max_imu_acc` / `max_imu_gyro`
  过滤阈值；
- PPS 同步模式等。

驱动内部的时间戳重锚窗口按墙钟时长（2.5s）而非包数计算，对上述任何速率配置
都保持一致语义。

## Install dependencies

1. Please make sure you have install ROS2.
2. Install Asio. If you are using ubuntu, you can install by following command: `sudo apt install libasio-dev`

## Param

here are some parameters you can set in config file:

```yaml
mid360_driver:
    ros__parameters:
        lidar_topic: /livox/lidar
        lidar_frame: livox_frame
        imu_topic: /livox/imu
        imu_frame: imu_frame
        lidar_publish_time_interval: 0.1
        is_topic_name_with_lidar_ip: false # 是否在话题名后面加雷达ip，可以用于区分多个雷达
```

## Contact

QQ group: 1070252119

Email: 1709185482@qq.com

## License

Copyright (C) 2025 Yingjie Huang

Licensed under the MIT License. See License.txt in the project root for license information.

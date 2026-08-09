# 集成测试：障碍高度带必须是「相对机器人底盘」的差值 point.z - robo_z。
#
# 这条测试锁的是一个查了很久的 bug：20cm 高台在代价地图上完全看不见，没有任何日志
# 或报错，而墙照常可见。两种绝对判法都会漏掉台面：
#   1) 在 transformPoint **之前**拿点云原始 z 比。点云 frame 是 livox_frame，原点
#      比地面高一个安装高度（仿真外参 base_link->livox_frame z=0.175），台面在那个
#      系里只有 +0.025，被 0.05 的阈值滤掉。
#   2) 变换到 map 之后跟固定的 z=0 比。Super-LIO 的 odom 原点锚在开机瞬间的雷达
#      位姿上，所以 map 的 z=0 是雷达平面而不是地面 —— 实测 RMUL.pcd 地面主峰在
#      -0.17（= -安装高度），台面绝对 z 只有 +0.025。
# 改成相对底盘做差（rose_navigation 的 bottom_z_to_robo_z / top_z_to_robo_z）之后，
# 任何固定的雷达平面偏移和定位 z 漂移都被抵消。
#
# 测试同时把这两条错路堵死：机器人在 map 里沉到 -0.30，传感器再抬高 0.175，台面点
# 在 livox_frame 里是 +0.025、在 map 里是 -0.10，只有做差（+0.20）才会被标记。
import os
import struct
import sys
import time
import unittest

import launch
import launch_ros
import launch_testing
from nav_msgs.msg import OccupancyGrid
import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# 仿真外参 base_link->livox_frame 的安装高度，见
# bringup/config/simulation/measurement_params_sim.yaml。
SENSOR_MOUNT_Z = 0.175
# 场地里那个 20cm 高台。
PLATFORM_HEIGHT = 0.20
# navigation2.yaml 里 rm_local_costmap 的实际取值。
OBSTACLE_Z_MIN_TO_ROBO = 0.05
RESOLUTION = 0.05

PLATFORM_XY = (1.0, 0.5)
GROUND_XY = (-1.0, -0.5)

# 机器人底盘在 map 下的 z。故意取负：RViz 里「机器人陷到地底下」就是这个状态
# （定位在 z 上飘）。此时台面在 map 系里的绝对 z 只有 PLATFORM_HEIGHT + ROBOT_MAP_Z
# = -0.10，低于任何正的阈值 —— 拿固定的 map z=0 平面做参考面就会整片漏掉。
ROBOT_MAP_Z = -0.30


@pytest.mark.launch_test
def generate_test_description():
    node = launch_ros.actions.Node(
        package='navigation2',
        executable='rm_local_costmap_node',
        name='rm_local_costmap',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'global_frame': 'map',
            'robot_base_frame': 'base_link_fake',
            'subscribe_scan': False,
            'subscribe_pointcloud': True,
            'pointcloud_topic': '/segmentation/obstacle',
            'update_frequency': 20.0,
            'publish_frequency': 20.0,
            'width': 5.0,
            'height': 5.0,
            'resolution': RESOLUTION,
            'robot_radius': 0.25,
            'inflation_radius': 0.0,
            'obstacle_min_range': 0.1,
            'obstacle_max_range': 6.0,
            'obstacle_z_min_to_robo': OBSTACLE_Z_MIN_TO_ROBO,
            'obstacle_z_max_to_robo': 2.0,
            'observation_timeout': 2.0,
            'update_on_new_observation_only': False,
            'reuse_previous_grid': False,
            'previous_obstacle_decay': 0,
        }],
    )
    return launch.LaunchDescription([
        node, launch_testing.actions.ReadyToTest(),
    ]), {'costmap': node}


def _make_cloud(frame_id, stamp, points):
    """构造最小的 xyz float32 PointCloud2，代价地图节点只读这三个字段."""
    cloud = PointCloud2()
    cloud.header.frame_id = frame_id
    cloud.header.stamp = stamp
    cloud.height = 1
    cloud.width = len(points)
    cloud.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    cloud.is_bigendian = False
    cloud.point_step = 12
    cloud.row_step = cloud.point_step * cloud.width
    cloud.is_dense = True
    cloud.data = b''.join(struct.pack('<fff', *p) for p in points)
    return cloud


class Harness(Node):

    def __init__(self):
        super().__init__('costmap_height_frame_harness')
        self.cloud_pub = self.create_publisher(PointCloud2, '/segmentation/obstacle', 1)
        self.grids = []
        self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap_raw',
            lambda m: self.grids.append(m),
            QoSProfile(
                depth=1,
                history=HistoryPolicy.KEEP_LAST,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ))

        # 机器人在 map 原点；传感器抬高 SENSOR_MOUNT_Z。这就是让原始 z 和 map 系 z
        # 差出一个安装高度的地方。
        self.tf_static = StaticTransformBroadcaster(self)
        self.tf_static.sendTransform([
            self._static('map', 'base_link_fake', ROBOT_MAP_Z),
            self._static('base_link_fake', 'livox_frame', SENSOR_MOUNT_Z),
        ])

    def _static(self, parent, child, z):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = parent
        tf.child_frame_id = child
        tf.transform.translation.z = z
        tf.transform.rotation.w = 1.0
        return tf

    def publish_cloud(self):
        # 点云在 livox_frame 里。地面和台面的高度都相对机器人底盘给出，所以不管
        # 机器人在 map 里沉多深，这两个点在 livox_frame 里的 z 都不变 —— 真实雷达
        # 也是这样：车沉下去，它测到的地面就跟着沉。
        points = [
            (PLATFORM_XY[0], PLATFORM_XY[1], PLATFORM_HEIGHT - SENSOR_MOUNT_Z),
            (GROUND_XY[0], GROUND_XY[1], 0.0 - SENSOR_MOUNT_Z),
        ]
        self.cloud_pub.publish(
            _make_cloud('livox_frame', self.get_clock().now().to_msg(), points))

    def spin_until_grid(self, timeout=15.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.publish_cloud()
            rclpy.spin_once(self, timeout_sec=0.05)
            # 拿变换后发布的那一帧，不要开机瞬间的空图。
            if len(self.grids) >= 3:
                return self.grids[-1]
        return self.grids[-1] if self.grids else None


def cell_value(grid, world_x, world_y):
    mx = int((world_x - grid.info.origin.position.x) / grid.info.resolution)
    my = int((world_y - grid.info.origin.position.y) / grid.info.resolution)
    assert 0 <= mx < grid.info.width and 0 <= my < grid.info.height, \
        f'({world_x}, {world_y}) 落在栅格外: ({mx}, {my})'
    return grid.data[my * grid.info.width + mx]


class TestCostmapHeightFrame(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.harness = Harness()

    def tearDown(self):
        self.harness.destroy_node()

    def test_platform_is_marked_even_when_robot_z_drifts(self):
        grid = self.harness.spin_until_grid()
        self.assertIsNotNone(grid, '没有收到 /local_costmap/costmap_raw')

        # 台面相对底盘高 0.20 > 0.05，必须被标记。两种错误的判定方式都会漏掉它：
        # 在 livox_frame 原始 z 上比是 +0.025；在 map 系跟固定 z=0 比是 -0.10。
        self.assertEqual(
            cell_value(grid, *PLATFORM_XY), 100,
            '20cm 高台没有进局部代价地图。检查 obstacle_z_min_to_robo 的判定：'
            '是不是跑到 transformPoint 之前了，或者拿绝对 z 跟固定的 map z=0 比、'
            '而不是减掉机器人底盘的 robo_z？')

        # 地面相对底盘是 0.0 < 0.05，必须被滤掉，否则地面残留点会把车围死。
        self.assertEqual(
            cell_value(grid, *GROUND_XY), 0,
            '地面点被当成障碍了：obstacle_z_min_to_robo 没有起作用')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])

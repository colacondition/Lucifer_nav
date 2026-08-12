# 集成测试：隧道顶板不得进代价地图，洞内的真障碍必须照常进。
#
# 这条测试锁的是隧道语义的第一个致命失败：顶板和侧壁在点云里跟墙毫无区别，按常规
# 高度带（上限 2 m）判定会把整个洞口涂成致命格，规划器彻底看不到通路 —— 而且没有
# 任何报错，表现只是「车不去走那条唯一的路」。参数调不出来，必须有先验语义。
#
# 反方向同样重要：不能变成「隧道内不看点云」。洞里真有个箱子的时候还得躲，否则语义
# 层就成了一个开在墙上的洞，什么都能穿过去。所以测试同时验证阈值之下的点照常标记。
#
# 高度全部相对机器人底盘（point.z - robo_z），理由见 test_costmap_height_frame.py：
# map 的 z=0 是雷达平面而不是地面。这里也让机器人在 map 里沉下去，确认隧道判定不是
# 靠绝对 z 蒙对的。
import os
import struct
import sys
import time
import unittest

from decision_interfaces.msg import SemanticMap, TunnelSpec
from geometry_msgs.msg import TransformStamped
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

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

SENSOR_MOUNT_Z = 0.175
RESOLUTION = 0.05
# 机器人在 map 下沉 0.30：拿绝对 z 判隧道就会算错，跟 height_frame 测试同一个理由。
ROBOT_MAP_Z = -0.30

# 语义地图覆盖 -2.5~2.5 米，跟局部代价地图的窗口对齐（机器人在 map 原点）。
SEM_W, SEM_H = 100, 100
SEM_ORIGIN = (-2.5, -2.5)

# 顶板放行的阈值是节点参数，不是 TunnelSpec 里的净高 —— 净高从地面量、这里的高度
# 相对 base_link，差一个未知的底盘离地偏置。净高故意给一个跟阈值明显不同的值，这样
# 「代码又拿净高当阈值」这种回退会被下面的断言抓到。
TUNNEL_GATE_HEIGHT = 0.20
TUNNEL_CLEAR_HEIGHT = 0.60
TUNNEL_CLEAR_WIDTH = 0.30

TERRAIN_FLAT = 0
TERRAIN_OBSTACLE = 1
TERRAIN_TUNNEL = 2

# 隧道沿 +x 轴穿过 y ∈ [-0.15, 0.15)，x ∈ [0.5, 1.0)。
TUNNEL_X_RANGE = (0.5, 1.0)
TUNNEL_Y_RANGE = (-0.15, 0.15)

# 洞内取样点：顶板（净高之上）和箱子（净高之下）放在同一 xy，逼着判定只能靠高度区分。
IN_TUNNEL_XY = (0.75, 0.0)
# 洞外的墙：同样的高度，必须照常被标记 —— 证明放行只发生在隧道格里。
OUTSIDE_XY = (0.75, 1.0)

# 顶板高度落在阈值 0.20 和净高 0.60 之间：拿阈值判就该被滤掉，拿净高判就会漏进来。
ROOF_HEIGHT = 0.45
# 洞内真障碍，在阈值之下。
BOX_HEIGHT = 0.12


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
            'semantic_map_topic': '/map_server/semantic_map',
            'update_frequency': 20.0,
            'publish_frequency': 20.0,
            'width': 5.0,
            'height': 5.0,
            'resolution': RESOLUTION,
            'robot_radius': 0.25,
            'inflation_radius': 0.0,
            'obstacle_min_range': 0.1,
            'obstacle_max_range': 6.0,
            'obstacle_z_min_to_robo': 0.05,
            # 关键：上限 2.0 就是原来会把顶板当障碍的那个值。测试不放宽它 ——
            # 顶板必须靠语义放行，而不是靠把高度带调小（那样会漏掉真的高障碍）。
            'obstacle_z_max_to_robo': 2.0,
            'tunnel_obstacle_z_max_to_robo': TUNNEL_GATE_HEIGHT,
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


def _make_semantic_map():
    """一条沿 +x 的隧道，周围是空地，隧道两侧是墙."""
    msg = SemanticMap()
    msg.header.frame_id = 'map'
    msg.width = SEM_W
    msg.height = SEM_H
    msg.resolution = RESOLUTION
    msg.origin_x = SEM_ORIGIN[0]
    msg.origin_y = SEM_ORIGIN[1]

    cells = SEM_W * SEM_H
    terrain = bytearray([TERRAIN_FLAT] * cells)
    angle = bytearray([0] * cells)
    magnitude = bytearray([0] * cells)
    cost = bytearray([0] * cells)
    ids = bytearray([0] * cells)

    def cell_range(lo, hi, origin):
        return range(int((lo - origin) / RESOLUTION), int((hi - origin) / RESOLUTION))

    for gy in cell_range(TUNNEL_Y_RANGE[0], TUNNEL_Y_RANGE[1], SEM_ORIGIN[1]):
        for gx in cell_range(TUNNEL_X_RANGE[0], TUNNEL_X_RANGE[1], SEM_ORIGIN[0]):
            index = gy * SEM_W + gx
            terrain[index] = TERRAIN_TUNNEL
            angle[index] = 0          # 轴线沿 +x
            magnitude[index] = 255    # 本体（> 0.95 阈值）
            ids[index] = 1

    # 洞两侧的墙。只是让地图像真的，判定不依赖它们。
    for gy in list(cell_range(-1.0, TUNNEL_Y_RANGE[0], SEM_ORIGIN[1])) + \
            list(cell_range(TUNNEL_Y_RANGE[1], 1.0, SEM_ORIGIN[1])):
        for gx in cell_range(TUNNEL_X_RANGE[0], TUNNEL_X_RANGE[1], SEM_ORIGIN[0]):
            terrain[gy * SEM_W + gx] = TERRAIN_OBSTACLE

    msg.terrain = bytes(terrain)
    msg.direction_angle = bytes(angle)
    msg.direction_magnitude = bytes(magnitude)
    msg.cost = bytes(cost)
    msg.tunnel_ids = bytes(ids)

    spec = TunnelSpec()
    spec.clear_height = TUNNEL_CLEAR_HEIGHT
    spec.clear_width = TUNNEL_CLEAR_WIDTH
    spec.run_up = 0.5
    spec.velocity_min = 0.2
    spec.velocity_max = 0.6
    msg.tunnels = [spec]
    return msg


class Harness(Node):

    def __init__(self):
        super().__init__('costmap_tunnel_roof_harness')
        self.cloud_pub = self.create_publisher(PointCloud2, '/segmentation/obstacle', 1)
        # 语义地图用 transient_local 发，跟 rm_map_server 一致，晚起的订阅者也收得到。
        self.semantic_pub = self.create_publisher(
            SemanticMap, '/map_server/semantic_map',
            QoSProfile(
                depth=1,
                history=HistoryPolicy.KEEP_LAST,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ))
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

        self.tf_static = StaticTransformBroadcaster(self)
        self.tf_static.sendTransform([
            self._static('map', 'base_link_fake', ROBOT_MAP_Z),
            self._static('base_link_fake', 'livox_frame', SENSOR_MOUNT_Z),
        ])
        self.semantic_pub.publish(_make_semantic_map())

    def _static(self, parent, child, z):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = parent
        tf.child_frame_id = child
        tf.transform.translation.z = z
        tf.transform.rotation.w = 1.0
        return tf

    def publish_cloud(self, points):
        # 高度按「相对机器人底盘」给出，再减去传感器安装高度换到 livox_frame。
        # 机器人在 map 里沉多深都不影响这些点在 livox_frame 里的坐标。
        self.cloud_pub.publish(_make_cloud(
            'livox_frame', self.get_clock().now().to_msg(),
            [(x, y, h - SENSOR_MOUNT_Z) for x, y, h in points]))

    def spin_until_grid(self, points, timeout=20.0):
        deadline = time.time() + timeout
        self.grids.clear()
        while time.time() < deadline:
            self.publish_cloud(points)
            rclpy.spin_once(self, timeout_sec=0.05)
            # 前几帧可能在语义地图到达之前就发出来了，取稳定之后的。
            if len(self.grids) >= 5:
                return self.grids[-1]
        return self.grids[-1] if self.grids else None


def cell_value(grid, world_x, world_y):
    mx = int((world_x - grid.info.origin.position.x) / grid.info.resolution)
    my = int((world_y - grid.info.origin.position.y) / grid.info.resolution)
    assert 0 <= mx < grid.info.width and 0 <= my < grid.info.height, \
        f'({world_x}, {world_y}) 落在栅格外: ({mx}, {my})'
    return grid.data[my * grid.info.width + mx]


class TestCostmapTunnelRoof(unittest.TestCase):

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

    def test_roof_is_ignored_but_wall_and_box_are_not(self):
        grid = self.harness.spin_until_grid([
            (IN_TUNNEL_XY[0], IN_TUNNEL_XY[1], ROOF_HEIGHT),
            (OUTSIDE_XY[0], OUTSIDE_XY[1], ROOF_HEIGHT),
        ])
        self.assertIsNotNone(grid, '没有收到 /local_costmap/costmap_raw')

        # 顶板在隧道阈值之上且落在隧道本体内，必须被滤掉，否则洞口被封死。
        # 它同时在净高（0.60）之下：如果代码回退成拿净高当阈值，这条就会失败。
        self.assertEqual(
            cell_value(grid, *IN_TUNNEL_XY), 0,
            '隧道顶板被标成障碍了，洞口会被封死。检查 tunnelHeightLimit：语义地图收到了吗？'
            '隧道格的 direction_magnitude 有没有超过本体阈值 0.95？'
            '阈值有没有被误接成 TunnelSpec.clear_height？')

        # 同样高度、同样距离，只是不在隧道里 —— 必须照常标记。否则说明放行泄漏到了
        # 隧道以外，等于把整个高度上限调没了。
        self.assertEqual(
            cell_value(grid, *OUTSIDE_XY), 100,
            '隧道外的同高度障碍也被滤掉了：放行范围泄漏出了隧道本体')

    def test_obstacle_inside_tunnel_below_clear_height_is_marked(self):
        grid = self.harness.spin_until_grid([
            (IN_TUNNEL_XY[0], IN_TUNNEL_XY[1], BOX_HEIGHT),
        ])
        self.assertIsNotNone(grid, '没有收到 /local_costmap/costmap_raw')

        # 阈值之下的点是洞里真的障碍物，不是隧道结构。放过它等于「隧道内不看点云」，
        # 车会直接撞上去。
        self.assertEqual(
            cell_value(grid, *IN_TUNNEL_XY), 100,
            '洞内阈值以下的障碍被滤掉了：隧道语义不能退化成「洞里不看点云」')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])

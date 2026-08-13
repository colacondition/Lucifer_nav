# 集成测试：隧道影响区内的点云不得进代价地图，区外的必须照常进。
#
# 这条测试锁的是隧道语义的第一个致命失败：顶板、门楣、侧壁上沿在点云里跟墙毫无
# 区别，按常规高度带（上限 2 m）判定会把整个洞口涂成致命格，规划器彻底看不到通路
# —— 而且没有任何报错，表现只是「车不去走那条唯一的路」。参数调不出来，必须有
# 先验语义。
#
# 放行的判据是「落在隧道影响区（本体格 + tunnel_margin_m 边距）内」，跟高度无关。
# 任何高度阈值都在「滤掉结构」和「漏掉真障碍」之间赌；净高够不够、姿态收没收是电控
# 的职责，导航只负责把车沿轴线送进洞，能不能进由静态地图的壁面致命格决定。所以洞里
# 低矮的点也一并不标 —— 这是确认过的设计，别改回按高度分层。
#
# 反方向同样重要：放行不能泄漏出影响区，否则等于把高度上限整个调没了。下面用同 x、
# 同高度、只差 y 的点钉住边距的边界。
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

# 影响区边距，跟节点参数 tunnel_margin_m 一致：本体格向外扩这么多米。
TUNNEL_MARGIN = 0.20
# 净高在这里不该是任何判据 —— 它从地面量，点云高度相对 base_link，中间差一个未知的
# 底盘离地偏置。故意取一个夹在下面两个取样高度中间的值：谁把它（或任何别的高度阈值）
# 接回放行判据，「洞里低点也不标」那条必然挂。
TUNNEL_CLEAR_HEIGHT = 0.30
TUNNEL_CLEAR_WIDTH = 0.30

TERRAIN_FLAT = 0
TERRAIN_OBSTACLE = 1
TERRAIN_TUNNEL = 2

# 隧道沿 +x 轴穿过 y ∈ [-0.15, 0.15)，x ∈ [0.5, 1.0)。
TUNNEL_X_RANGE = (0.5, 1.0)
TUNNEL_Y_RANGE = (-0.15, 0.15)

# 本体格内的取样点。顶板和低矮的点放在同一 xy，只差高度：两个都必须被放行，这是
# 「判据与高度无关」唯一能被钉死的地方。
IN_TUNNEL_XY = (0.75, 0.0)
# 影响区内、本体格外 —— 门楣和顶板前沿的点云正落在这一圈。本体行中心最远到
# y = 0.125，这个点所在格中心 y = 0.275，距离 0.15 < 边距 0.20。
IN_MARGIN_XY = (0.75, 0.28)
# 刚出影响区：所在格中心 y = 0.375，距离 0.25 > 边距 0.20，必须照常标记。差这两格
# 就是「边距是有边的」和「边距把半张图都放行了」的分界。
OUTSIDE_MARGIN_XY = (0.75, 0.36)
# 洞外的墙：同样的高度，必须照常被标记 —— 证明放行只发生在影响区里。
OUTSIDE_XY = (0.75, 1.0)

# 结构点：净高之上，常规高度带（上限 2.0）照收，只能靠语义放行。
ROOF_HEIGHT = 0.45
# 洞里低矮的点：净高之下，同样不标。见开头，别改回按高度分层。
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
            'tunnel_margin_m': TUNNEL_MARGIN,
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

    def test_region_points_are_ignored_but_points_outside_it_are_not(self):
        grid = self.harness.spin_until_grid([
            (IN_TUNNEL_XY[0], IN_TUNNEL_XY[1], ROOF_HEIGHT),
            (IN_MARGIN_XY[0], IN_MARGIN_XY[1], ROOF_HEIGHT),
            (OUTSIDE_MARGIN_XY[0], OUTSIDE_MARGIN_XY[1], ROOF_HEIGHT),
            (OUTSIDE_XY[0], OUTSIDE_XY[1], ROOF_HEIGHT),
        ])
        self.assertIsNotNone(grid, '没有收到 /local_costmap/costmap_raw')

        # 本体格内的结构点必须被滤掉，否则洞口被封死。
        self.assertEqual(
            cell_value(grid, *IN_TUNNEL_XY), 0,
            '隧道本体格里的点被标成障碍了，洞口会被封死。语义地图收到了吗？'
            '隧道格的 direction_magnitude 有没有超过本体阈值 0.95？')

        # 门楣/顶板前沿的点云落在本体格外一到两格。只认本体格时它们被原样标成致命格，
        # 横在洞口上把洞封死 —— tunnel_margin_m 就是为了把放行扩出这一圈。
        self.assertEqual(
            cell_value(grid, *IN_MARGIN_XY), 0,
            '影响区边距内的点被标成障碍了：门楣点云会横在洞口上。'
            'tunnel_margin_m 传进节点了吗？TunnelRegionGrid 是按这个边距建的吗？')

        # 边距外两格，同 x、同高度 —— 必须照常标记。否则说明边距没有边界，
        # 洞口附近的真墙也会被无视。
        self.assertEqual(
            cell_value(grid, *OUTSIDE_MARGIN_XY), 100,
            '影响区边距外的障碍也被滤掉了：放行范围泄漏出了影响区')

        # 离洞更远的墙同理，钉住泄漏不是「只多漏一格」那种量级。
        self.assertEqual(
            cell_value(grid, *OUTSIDE_XY), 100,
            '隧道外的同高度障碍也被滤掉了：放行范围泄漏出了影响区')

    def test_low_point_inside_the_tunnel_is_ignored_too(self):
        grid = self.harness.spin_until_grid([
            (IN_TUNNEL_XY[0], IN_TUNNEL_XY[1], BOX_HEIGHT),
            (OUTSIDE_XY[0], OUTSIDE_XY[1], BOX_HEIGHT),
        ])
        self.assertIsNotNone(grid, '没有收到 /local_costmap/costmap_raw')

        # 同一个 xy、只是高度换到净高之下：判据与高度无关，照样不标。这条挂了就说明
        # 有人把某个高度阈值接回了放行判据（净高、tunnel_obstacle_z_max_to_robo 之类）。
        self.assertEqual(
            cell_value(grid, *IN_TUNNEL_XY), 0,
            '洞内低矮的点被标成障碍了：放行判据不该跟高度有关，见文件开头')

        # 同样的低点在洞外必须照常标 —— 否则上面那条可能只是因为整个高度带失效。
        self.assertEqual(
            cell_value(grid, *OUTSIDE_XY), 100,
            '洞外的低矮障碍也被滤掉了：高度带本身出了问题，不是隧道语义在起作用')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])

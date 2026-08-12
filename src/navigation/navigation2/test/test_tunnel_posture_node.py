# 集成测试：进洞前必须发出收云台请求，出洞后必须收回。
#
# gtest 已经锁住了判定本身（test_tunnel_posture.cpp）。这里锁的是节点层那几个只有跑
# 起来才会错的接线：语义地图订阅的 QoS 对不对（transient_local 不匹配就是永远收不到
# 图、tunnels 永远为空、标志位永远 false，而且没有任何报错）、tf 查的是哪两个坐标系、
# 话题名有没有拼错。这些错误的表现完全一样 —— 云台在洞口撞上去，日志里干干净净。
#
# 用真实位姿驱动而不是灌假距离：run_up 是世界系里的物理距离，语义地图的 origin 参与
# 换算。用格号或者机器人系坐标算出来的距离在这条链路上都是错的，而错的方向是提前量
# 变小。
import os
import sys
import time
import unittest

from decision_interfaces.msg import GimbalPosture, SemanticMap, TunnelSpec
from geometry_msgs.msg import TransformStamped
import launch
import launch_ros
import launch_testing
import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

RESOLUTION = 0.05
# origin 故意非零：世界坐标和格号在这张图上差了 2.5 米，拿格号算距离会立刻错。
SEM_W, SEM_H = 100, 100
SEM_ORIGIN = (-2.5, -2.5)

TERRAIN_FLAT = 0
TERRAIN_TUNNEL = 2

# 隧道沿 +x 轴穿过 y ∈ [-0.15, 0.15)，x ∈ [0.5, 1.0)。本体格中心因此落在
# x ∈ [0.525, 0.975]、y ∈ [-0.125, 0.125]。
TUNNEL_X_RANGE = (0.5, 1.0)
TUNNEL_Y_RANGE = (-0.15, 0.15)
TUNNEL_ENTRANCE_X = 0.525
TUNNEL_EXIT_X = 0.975

RUN_UP = 0.5
HYSTERESIS = 0.3

# 离洞口 0.325 米，在 run_up 之内。
APPROACH_XY = (TUNNEL_ENTRANCE_X - 0.325, 0.0)
# 洞里。
INSIDE_XY = (0.75, 0.0)
# 远到搜索框（run_up + 滞回 = 0.8）都够不着，用来把节点状态复位成 false。
FAR_XY = (-2.0, 0.0)


@pytest.mark.launch_test
def generate_test_description():
    node = launch_ros.actions.Node(
        package='navigation2',
        executable='rm_tunnel_posture_node',
        name='rm_tunnel_posture',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'global_frame': 'map',
            'robot_base_frame': 'base_link_fake',
            'semantic_map_topic': '/map_server/semantic_map',
            'posture_topic': '/gimbal_posture',
            'update_frequency': 20.0,
            'hysteresis': HYSTERESIS,
        }],
    )
    return launch.LaunchDescription([
        node, launch_testing.actions.ReadyToTest(),
    ]), {'posture': node}


def _make_semantic_map():
    """一条沿 +x 的隧道，其余是空地."""
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

    msg.terrain = bytes(terrain)
    msg.direction_angle = bytes(angle)
    msg.direction_magnitude = bytes(magnitude)
    msg.cost = bytes(cost)
    msg.tunnel_ids = bytes(ids)

    spec = TunnelSpec()
    spec.clear_height = 0.6
    spec.clear_width = 0.3
    spec.run_up = RUN_UP
    spec.velocity_min = 0.2
    spec.velocity_max = 0.6
    msg.tunnels = [spec]
    return msg


class Harness(Node):

    def __init__(self):
        super().__init__('tunnel_posture_harness')
        state_qos = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.semantic_pub = self.create_publisher(
            SemanticMap, '/map_server/semantic_map', state_qos)
        self.requests = []
        self.create_subscription(
            GimbalPosture, '/gimbal_posture',
            lambda m: self.requests.append(m.lower), state_qos)

        # 动态 tf：位姿要在测试过程中改，静态广播做不到。
        self.tf_broadcaster = TransformBroadcaster(self)
        self.semantic_pub.publish(_make_semantic_map())

    def publish_pose(self, x, y):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'base_link_fake'
        tf.transform.translation.x = float(x)
        tf.transform.translation.y = float(y)
        tf.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tf)

    def settle_at(self, x, y, timeout=20.0):
        """把车放到 (x, y)，返回稳定之后的请求值."""
        deadline = time.time() + timeout
        self.requests.clear()
        while time.time() < deadline:
            self.publish_pose(x, y)
            rclpy.spin_once(self, timeout_sec=0.02)
            # 头几帧可能是位姿更新之前算的，取之后的。
            if len(self.requests) >= 8:
                return self.requests[-1]
        return self.requests[-1] if self.requests else None


class TestTunnelPostureNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.harness = Harness()
        # 节点状态跨测试方法保留（进程只起一次），每条测试先复位成 false。
        self.assertIs(
            self.harness.settle_at(*FAR_XY), False,
            '车在 2.5 米开外还在请求收云台。语义地图收到了吗（QoS 是否匹配）？'
            'run_up 阈值有没有被当成整图生效？')

    def tearDown(self):
        self.harness.destroy_node()

    def test_lowers_before_the_entrance(self):
        # 离洞口 0.325 米 < run_up 0.5：必须已经在请求收云台，电控要靠这段距离完成动作。
        self.assertIs(
            self.harness.settle_at(*APPROACH_XY), True,
            '进洞前没有发出收云台请求 —— 云台会撞在顶板上。检查 tf 查的坐标系'
            '（map -> base_link_fake）、话题名，以及距离是不是拿世界坐标算的')

    def test_stays_lowered_inside_and_raises_after_leaving(self):
        # 洞里全程保持。
        self.assertIs(
            self.harness.settle_at(*INSIDE_XY), True, '车在洞里却没有请求收云台')

        # 出洞后退开 run_up + 滞回还多一截，必须回 false，否则出了洞云台再也抬不起来。
        self.assertIs(
            self.harness.settle_at(TUNNEL_EXIT_X + 1.2, 0.0), False,
            '离开隧道后请求没有收回，云台会一直放着')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])

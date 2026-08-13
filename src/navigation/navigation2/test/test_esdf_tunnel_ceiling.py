#!/usr/bin/env python3
"""
锁住 RC-ESDF 的隧道顶板豁免：影响区内的顶板/门楣点云不得触发整车碰撞。

背景：代价地图在 processPointCloud 里按 TunnelRegionGrid 把隧道影响区内的点滤掉，
所以洞里/洞口不会被顶板封死。但 MPC 的 RC-ESDF 直接吃 /segmentation/obstacle 原始
点云，且只查 xy 平面 —— 顶板点（z≈clear_height，xy 在车体正上方）被投影成 xy 障碍，
把洞口和洞内都判成碰撞，表现为「云台已绿却停在洞口 / 蹭进去后动不了」。

ESDF 必须跟代价地图用同一张影响区跳过这些点；区外的同高度点必须照常拦车（见下方
outside 用例）。不补这条豁免，ESDF 会重新把唯一通路堵上，而且全程无报错。
"""
import time
import unittest

from decision_interfaces.msg import SemanticMap, TunnelSpec
from geometry_msgs.msg import PoseStamped, Twist
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import launch_testing
import launch_testing.actions
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import rclpy
from rclpy.node import Node as RclpyNode
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header


def transient_local_qos(depth=1):
    return QoSProfile(
        depth=depth,
        history=HistoryPolicy.KEEP_LAST,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )


def generate_test_description():
    container = ComposableNodeContainer(
        name='test_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='navigation2',
                plugin='navigation2::RmMpcController',
                name='rm_mpc_controller',
                parameters=[{
                    'use_sim_time': False,
                    'use_tf_pose': False,
                    'target_frame': 'map',
                    'robot_base_frame': 'base_link',
                    'path_topic': '/plan',
                    'odom_topic': '/Odometry',
                    'cmd_vel_topic': '/cmd_vel_nav_raw',
                    'control_fps': 30.0,
                    'expected_speed': 1.5,
                    'goal_tolerance': 0.25,
                    'esdf.enable': True,
                    'esdf.robot_radius': 0.25,
                    'esdf.safety_margin': 0.04,
                    'esdf.check_steps': 5,
                    'esdf.obstacle_topic': '/segmentation/obstacle',
                    'tunnel_speed_window.enable': True,
                    'tunnel_margin_m': 0.20,
                    'semantic_map_topic': '/map_server/semantic_map',
                    'local_safety.costmap_topic': '/local_costmap/costmap',
                    'local_safety.costmap_timeout': 1.0,
                    'feedback.executed_cmd_topic': '/cmd_vel',
                    'feedback.executed_cmd_timeout': 0.5,
                    'progress.min_displacement': 0.15,
                    'progress.no_progress_timeout': 2.0,
                    'progress.stuck_timeout': 2.0,
                    'recovery.enable': True,
                    'recovery.suppress_near_goal': 0.35,
                    'recovery.reverse_speed': 0.3,
                    'recovery.reverse_distance': 0.4,
                    'recovery.max_speed': 0.4,
                    'recovery.reach_tolerance': 0.12,
                    'recovery.dwell_time': 0.2,
                    'recovery.max_duration': 5.0,
                    'recovery.max_attempts': 2,
                    'recovery.veto_recovery_time': 3.0,
                }],
            ),
        ],
        output='screen',
    )
    return (
        LaunchDescription([
            container,
            launch_testing.actions.ReadyToTest(),
        ]),
        {'container': container},
    )


# 语义地图：沿 +x 的隧道，覆盖世界 y ∈ [-0.15, 0.15)，x ∈ [0.0, 3.0)。机器人在原点，
# 目标 +x 方向，路径正穿隧道。
SEM_W, SEM_H = 100, 100
SEM_RES = 0.05
SEM_ORIGIN = (-2.5, -2.5)
TUNNEL_X = (0.0, 3.0)
TUNNEL_Y = (-0.15, 0.15)


def make_semantic_map():
    msg = SemanticMap()
    msg.header.frame_id = 'map'
    msg.width = SEM_W
    msg.height = SEM_H
    msg.resolution = SEM_RES
    msg.origin_x = SEM_ORIGIN[0]
    msg.origin_y = SEM_ORIGIN[1]

    cells = SEM_W * SEM_H
    terrain = bytearray([0] * cells)
    angle = bytearray([0] * cells)
    magnitude = bytearray([0] * cells)
    cost = bytearray([0] * cells)
    ids = bytearray([0] * cells)

    def crange(lo, hi, origin):
        return range(int((lo - origin) / SEM_RES), int((hi - origin) / SEM_RES))

    for gy in crange(TUNNEL_Y[0], TUNNEL_Y[1], SEM_ORIGIN[1]):
        for gx in crange(TUNNEL_X[0], TUNNEL_X[1], SEM_ORIGIN[0]):
            index = gy * SEM_W + gx
            terrain[index] = 2      # TERRAIN_TUNNEL
            angle[index] = 0        # 轴线 +x
            magnitude[index] = 255  # 本体（> 0.95）
            ids[index] = 1

    msg.terrain = bytes(terrain)
    msg.direction_angle = bytes(angle)
    msg.direction_magnitude = bytes(magnitude)
    msg.cost = bytes(cost)
    msg.tunnel_ids = bytes(ids)

    spec = TunnelSpec()
    spec.clear_height = 0.30
    spec.clear_width = 0.30
    spec.run_up = 0.5
    spec.velocity_min = 0.2
    spec.velocity_max = 0.6
    msg.tunnels = [spec]
    return msg


class TestEsdfTunnelCeiling(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = RclpyNode('test_esdf_tunnel_ceiling')
        self.path_pub = self.node.create_publisher(Path, '/plan', 1)
        self.odom_pub = self.node.create_publisher(Odometry, '/Odometry', 10)
        self.costmap_pub = self.node.create_publisher(
            OccupancyGrid, '/local_costmap/costmap', transient_local_qos())
        self.obstacle_pub = self.node.create_publisher(
            PointCloud2, '/segmentation/obstacle', rclpy.qos.qos_profile_sensor_data)
        self.executed_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.semantic_pub = self.node.create_publisher(
            SemanticMap, '/map_server/semantic_map', transient_local_qos())

        self.recovery_entered = []
        self.node.create_subscription(
            Bool, '/goal_approach_controller/enabled',
            lambda m: self.recovery_entered.append(not m.data),
            transient_local_qos())

        # 语义地图用 transient_local 发，晚起的订阅者（MPC）也收得到。
        self.semantic_pub.publish(make_semantic_map())

    def tearDown(self):
        self.node.destroy_node()

    def _stamp(self):
        return self.node.get_clock().now().to_msg()

    def _publish_inputs(
            self, robot_x, robot_y, goal_x, goal_y, obstacle_x, obstacle_y, obstacle_z):
        stamp = self._stamp()

        # 语义地图每拍都重发：MPC 节点的订阅在容器里启动，setUp 只发一次会跟它的
        # 订阅建立抢跑，transient_local 的 latched 消息可能已经错过。
        self.semantic_pub.publish(make_semantic_map())

        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = stamp
        for x, y in ((0.0, 0.0), (goal_x, goal_y)):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_pub.publish(path)

        odom = Odometry()
        odom.header.frame_id = 'map'
        odom.header.stamp = stamp
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = float(robot_x)
        odom.pose.pose.position.y = float(robot_y)
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)

        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.header.stamp = stamp
        grid.info.resolution = 0.05
        grid.info.width = 160
        grid.info.height = 120
        grid.info.origin.position.x = -1.0
        grid.info.origin.position.y = -3.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * (160 * 120)
        self.costmap_pub.publish(grid)

        # 障碍点在 base_link 系，z 顶到顶板高度。MPC 只读 x/y，但顶板点正是这个 z。
        header = Header(stamp=stamp, frame_id='base_link')
        points = [(obstacle_x - robot_x, obstacle_y - robot_y, obstacle_z)]
        cloud = point_cloud2.create_cloud_xyz32(header, points)
        self.obstacle_pub.publish(cloud)

        executed = Twist()
        executed.linear.x = 0.5
        self.executed_pub.publish(executed)

    def test_tunnel_ceiling_point_does_not_block(self):
        """
        机器人在 (0,0)，目标 (5,0)，隧道沿 +x，顶板点在 (1,0,z=0.3) 落在隧道影响区内。
        ESDF 应该跳过它（不判碰撞），MPC 不进恢复。
        """
        self.recovery_entered.clear()
        deadline = time.time() + 3.0
        while time.time() < deadline:
            self._publish_inputs(
                robot_x=0.0, robot_y=0.0,
                goal_x=5.0, goal_y=0.0,
                obstacle_x=1.0, obstacle_y=0.0, obstacle_z=0.30)
            rclpy.spin_once(self.node, timeout_sec=0.03)

        self.assertFalse(
            any(self.recovery_entered),
            "隧道影响区内的顶板点触发了 ESDF 碰撞：MPC 进了恢复。"
            "ESDF 没跟代价地图用同一张影响区跳过顶板点云")

    def test_ceiling_point_outside_tunnel_still_blocks(self):
        """
        同高度、同 x，但 y 在隧道影响区之外：必须照常拦车。否则说明豁免范围泄漏出
        影响区，等于把整车碰撞检查整体关掉了。
        """
        self.recovery_entered.clear()
        deadline = time.time() + 3.0
        while time.time() < deadline:
            self._publish_inputs(
                robot_x=0.0, robot_y=0.0,
                goal_x=5.0, goal_y=0.0,
                obstacle_x=1.0, obstacle_y=1.0, obstacle_z=0.30)
            rclpy.spin_once(self.node, timeout_sec=0.03)

        self.assertTrue(
            any(self.recovery_entered),
            "隧道影响区外的点没有被 ESDF 拦住：豁免范围泄漏出了影响区")


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)

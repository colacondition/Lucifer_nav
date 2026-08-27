# 集成测试：waypoint_executor follow 模式的执行语义。
#
# 全系统唯一的「航点 -> 目标串」执行语义载体此前零测试；它坏掉的表现是
# 整车静默趴窝且无 ERROR。executor 类私有于 TU（gtest 够不到），必须走
# launch_test 做图级闭环。
#
# 四条用例对应四条源码语义：
#   dispatch:      action 目标接受后立刻向 goal_topic 发布第一个航点
#   switch:        对非终点航点 remaining <= switch_distance 即切下一个并重发
#   timeout_fuse:  终点段等不到 GOAL_REACHED 且超出容差 -> status_timeout 熔断
#                  判该航点失败 -> 整个 action 以 ABORTED 收尾（防 MPC 卡死拖死链路）
#   completed:     终点 remaining <= final_goal_tolerance 直接接受 -> SUCCEEDED
#
# 输入替身：map->base_link_fake 的持续 TF 广播是 executor 的真实位姿来源；
# 测试中在 spin 循环里不断刷新 TF 时间戳避免 tf2 过期过滤。
import os
import tempfile
import time
import unittest

import launch
import launch_ros
import launch_testing
import pytest
import rclpy
import tf2_ros
from decision_interfaces.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String

WP1 = (1.0, 0.0)
WP2 = (2.6, 0.0)
ROBOT_HOME_X = 0.7          # 距 WP1 恰好 0.3m <= switch_distance(0.5)


@pytest.mark.launch_test
def generate_test_description():
    fd, wp_file = tempfile.mkstemp(suffix=".csv")
    with os.fdopen(fd, "w") as f:
        f.write("id,pose_x,pose_y,pose_z,rot_x,rot_y,rot_z,rot_w,command,\n")
        for i, (x, y) in enumerate([WP1, WP2]):
            f.write(f"{i},{x},{y},0,0,0,0,1,,\n")

    node = launch_ros.actions.Node(
        package='waypoint_editor',
        executable='waypoint_executor',
        name='wpt_follow_executor',
        output='screen',
        parameters=[{
            'mode': 'follow',
            'waypoint_file': wp_file,
            'frame_id': 'map',
            'status_topic': '/wpt_test/mpc_status',
            'goal_topic': '/wpt_test/goal_pose',
            'override_waypoints_topic': '/wpt_test/override',
            'approach_enabled_topic': '/wpt_test/approach_enabled',
            'current_waypoints_topic': '/wpt_test/current_waypoints',
            'saved_waypoint_file_topic': '/wpt_test/saved_file',
            'executor_status_topic': '/wpt_test/follow_status',
            'robot_base_frame': 'base_link_fake',
            'global_frame': 'map',
            'switch_distance': 0.5,
            'final_goal_tolerance': 0.4,
            'status_timeout': 3.5,
            'goal_republish_interval': 0.3,
            'use_sim_time': False,
        }],
    )
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()])


class Fixture(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('wpt_test_client')
        cls.action = ActionClient(cls.node, FollowWaypoints,
                                  '/waypoint_editor/follow_waypoints')
        cls.published_goals = []          # executor 发出的目标串
        cls.node.create_subscription(
            PoseStamped, '/wpt_test/goal_pose', cls._on_goal, 10)
        cls.tf_br = tf2_ros.StaticTransformBroadcaster(cls.node)
        cls.status_in = []                # executor 发布的执行状态串
        latch = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
        cls.node.create_subscription(String, '/wpt_test/follow_status',
                                     lambda m: cls.status_in.append(m.data), latch)

    @classmethod
    def _on_goal(cls, m):
        cls.published_goals.append((round(m.pose.position.x, 2),
                                    round(m.pose.position.y, 2)))

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def broadcast_robot_at(self, x):
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link_fake'
        t.transform.translation.x = float(x)
        t.transform.rotation.w = 1.0
        self.tf_br.sendTransform(t)

    def spin_until(self, pred, timeout_s=10.0):
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            self.broadcast_robot_at(getattr(self, 'robot_x', ROBOT_HOME_X))
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if pred():
                return True
        return False

    def send_waypoints(self, waypoints):
        goal = FollowWaypoints.Goal()
        goal.waypoints.header.frame_id = 'map'
        for (x, y) in waypoints:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            goal.waypoints.poses.append(ps)
        assert self.action.wait_for_server(timeout_sec=10.0), "action server 未上线"
        fut = self.action.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=8.0)
        handle = fut.result()
        self.assertIsNotNone(handle, "目标请求失败")
        self.assertTrue(handle.accepted, "目标被拒绝")
        return handle

    def await_result(self, handle, timeout_s=15.0):
        rfut = handle.get_result_async()
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            self.broadcast_robot_at(getattr(self, 'robot_x', ROBOT_HOME_X))
            if rfut.done():
                return rfut.result().status   # GoalStatus: SUCCEEDED=4 ABORTED=6
        return None


class TestFollowSemantics(Fixture):

    def setUp(self):
        # 每条用例独立的「车位置」与发布记录视角（发布历史跨用例共享，
        # 断言时只看本用例触发的增量即可）。
        self.base_goals = len(self.published_goals)

    def test_01_dispatch_publishes_first_waypoint(self):
        """action 目标一收，第一航点立刻上 /goal_pose。"""
        h = self.send_waypoints([WP1])
        ok = self.spin_until(lambda: len(self.published_goals) > self.base_goals
                             and abs(self.published_goals[-1][0] - WP1[0]) < 0.01,
                             timeout_s=6.0)
        self.assertTrue(ok, f"首航点未发布: tail={self.published_goals[-3:]}")
        cfut = h.cancel_goal_async()
        rclpy.spin_until_future_complete(self.node, cfut, timeout_sec=3.0)
        self.spin_until(lambda: any(s == "IDLE" for s in self.status_in[-4:]),
                        timeout_s=4.0)  # 等 worker 真正停下，避免污染下一用例

    def test_02_switch_advances_on_proximity(self):
        """车距 WP1 仅 0.3m(<=switch_distance) => 不追 WP1 直接切 WP2 重发。"""
        self.robot_x = WP1[0] + 0.3     # remaining(WP1)=0.3 <= 0.5
        before = len(self.published_goals)
        self.send_waypoints([WP1, WP2])
        ok = self.spin_until(
            lambda: any(abs(g[0] - WP2[0]) < 0.01 for g in self.published_goals[before:]),
            timeout_s=8.0)
        self.assertTrue(ok, f"未自动切到 WP2: new_goals={self.published_goals[before:]}")

    def test_03_completed_when_final_within_tolerance(self):
        """终点容差路径：车贴近 WP2(<=0.4m) -> SUCCEEDED。"""
        self.robot_x = WP2[0] - 0.35
        h = self.send_waypoints([WP2])
        status = self.await_result(h, timeout_s=12.0)
        self.assertEqual(status, 4, "final_goal_tolerance 内的终点应成功收尾")

    def test_04_timeout_fuse_aborts_when_mpc_stuck(self):
        """终点段既无 GOAL_REACHED 又超容差 -> status_timeout 熔断 ABORTED。"""
        # 关键：先把远位置 TF 广播出去再下发目标，防止 executor 在首个
        # 周期读到上一用例遗留的近场位姿而「瞬移成功」。
        self.robot_x = -5.0
        end = time.monotonic() + 0.8
        while time.monotonic() < end:
            self.broadcast_robot_at(-5.0)
            rclpy.spin_once(self.node, timeout_sec=0.05)
        h = self.send_waypoints([WP2])
        status = self.await_result(h, timeout_s=14.0)
        self.assertEqual(status, 6, "MPC 卡死时应由 status_timeout 熔断并 ABORTED")

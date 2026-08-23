# 集成测试：定位丢失时停车等重定位。
#
# 为什么要在 ROS 层测：判据用了 latch 话题 + control() 主路径，gtest 碰不到订阅
# 回调和定时器调度。错了的表现分两种，都没有任何一层会报错：
#   - LOST 还在跟 map 系路径 → 开去错的地方
#   - 没收到过消息也停车 → mapping_nav / 现有测试全部开不动
#
# OK 不停：定位侧已经把上一拍有效 map→odom 握住了，短暂不一致不该把车钉死。
# 「从没收到过状态」在 test_integrity_gate_no_status.py：/localization_status
# 是 transient_local，跟 LOST 用例共进程会被 latch 污染。
import os
import sys
import unittest

import launch
import launch_ros
import launch_testing
import pytest
import rclpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mpc_harness import Harness, mpc_test_params  # noqa: E402


@pytest.mark.launch_test
def generate_test_description():
    node = launch_ros.actions.Node(
        package='navigation2',
        executable='rm_mpc_controller_node',
        name='rm_mpc_controller',
        output='screen',
        parameters=[mpc_test_params()],
    )
    return launch.LaunchDescription([
        node, launch_testing.actions.ReadyToTest(),
    ]), {'mpc': node}


def max_speed(cmds):
    """Return the largest linear speed in a batch of Twist commands."""
    return max((abs(c.linear.x) + abs(c.linear.y) for c in cmds), default=0.0)


class TestIntegrityGate(unittest.TestCase):

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

    def drive_and_collect(self, seconds=1.5):
        """Drive for a while and return the commands seen after settling."""
        self.harness.drive(seconds, (0.5, 0.0), (5.0, 0.0), 0.0)
        # 丢掉前几帧：latch 话题跟路径/里程计不同步，头几个控制周期可能还没看到状态。
        return self.harness.nav_cmds[5:]

    def test_holds_still_while_localization_is_lost(self):
        self.harness.set_localization_status('LOST waiting')
        cmds = self.drive_and_collect()

        self.assertTrue(cmds, '没收到任何指令 —— 门把 publishStop 也挡掉了？')
        self.assertAlmostEqual(
            max_speed(cmds), 0.0, places=6,
            msg='localization_status=LOST 时车还在动 —— 会按坏 map→odom 开出去')

    def test_ok_hold_still_drives(self):
        # OK nis_reject：定位侧握住上一拍有效 TF，短暂 NIS 拒绝不该把车钉死。
        self.harness.set_localization_status('OK nis_reject')
        cmds = self.drive_and_collect()

        self.assertGreater(
            max_speed(cmds), 0.05,
            'OK hold 就把车停住了 —— 走廊/短暂不一致会把导航钉死')

    def test_resumes_once_status_is_ok(self):
        self.harness.set_localization_status('LOST tracking_lost')
        lost_cmds = self.drive_and_collect()
        self.assertAlmostEqual(
            max_speed(lost_cmds), 0.0, places=6,
            msg='LOST 阶段车还在动')

        self.harness.nav_cmds.clear()
        self.harness.set_localization_status('OK accepted')
        cmds = self.drive_and_collect()
        self.assertGreater(
            max_speed(cmds), 0.05,
            '重定位成功后车还停着 —— 门没有在 OK 后放开')

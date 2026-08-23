# 集成测试：从没收到过 localization_status 时车必须能走。
#
# 为什么单独一个文件：/localization_status 是 transient_local，一旦发过
# LOST 就会 latch 在节点里。跟 test_integrity_gate.py 共进程的话，字母序
# 下 test_holds_still_while_localization_is_lost 会先跑，把「从未收到过」
# 这个分支污染掉 —— 跟 test_gimbal_gate_no_feedback.py 同一原因。
#
# mapping_nav 不起 fast_location；绝大多数集成测试也不发这条。
# 门加在 control() 主路径上，缺省停车会把建图导航和现有测试全停死。
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


class TestIntegrityGateWithoutStatus(unittest.TestCase):

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

    def test_no_status_does_not_block_normal_driving(self):
        self.harness.drive(1.5, (0.5, 0.0), (5.0, 0.0), 0.0)
        cmds = self.harness.nav_cmds[5:]

        self.assertTrue(cmds, '没收到任何指令 —— 门把 publishStop 也挡掉了？')
        speed = max((abs(c.linear.x) + abs(c.linear.y) for c in cmds), default=0.0)
        self.assertGreater(
            speed, 0.05,
            '没收到 localization_status 时车也不走 —— 门的缺省方向反了')

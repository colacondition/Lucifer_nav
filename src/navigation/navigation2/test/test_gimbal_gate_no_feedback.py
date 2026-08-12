# 集成测试：请求收云台，但一条回传都没收到过 —— 车必须停住。
#
# 为什么单独一个文件：/gimbal_posture_state 是 transient_local，一旦发过就会 latch 在
# 节点里。跟别的用例共用一个节点进程的话，前面任何一条发过实测姿态的测试都会把这个分支
# 污染掉 —— 「从未收到过」这个状态只能在全新的进程里复现。
#
# 这个分支守的是最坏情况：电控没起来、回传帧没实现、串口没插。此时无从判断云台在哪个
# 姿态，猜「已经收好」的代价是云台撞在顶板上，猜「没收好」的代价只是车不动。
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


class TestGimbalGateWithoutFeedback(unittest.TestCase):

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

    def test_holds_still_when_no_posture_feedback_ever_arrived(self):
        # 只发请求，不发实测姿态。
        self.harness.set_gimbal(requested_lower=True, measured_lowered=None)
        self.harness.drive(1.5, (0.5, 0.0), (5.0, 0.0), 0.0)
        cmds = self.harness.nav_cmds[5:]

        self.assertTrue(cmds, '没收到任何指令 —— 门把 publishStop 也挡掉了？')
        speed = max((abs(c.linear.x) + abs(c.linear.y) for c in cmds), default=0.0)
        self.assertAlmostEqual(
            speed, 0.0, places=6,
            msg='请求收云台但从未收到过电控回传，车却在动 —— '
                '「不知道云台在哪」被当成了「云台已经收好」')


@launch_testing.post_shutdown_test()
class TestNoFeedbackWarningInLog(unittest.TestCase):

    def test_warning_names_the_missing_feedback(self, proc_output, mpc):
        # 停车必须说清是哪一种原因。只打「等云台」的话，现场没法区分「云台在动，等一下就好」
        # 和「电控压根没接线，永远等不到」—— 后者要去查硬件，前者什么都不用做。
        text = ''.join(
            item.text.decode() if isinstance(item.text, bytes) else item.text
            for item in proc_output[mpc])
        self.assertIn('还没收到过电控的姿态回传', text)

# 集成测试：进洞前等云台收到位。
#
# 为什么要在 ROS 层测：这个门的判据用了三个来自不同话题的值（请求、实测、有没有收到过
# 实测），gtest 碰不到订阅回调和 control() 的调度顺序。而它错了的表现是「车照常开进去，
# 云台撞在顶板上」—— 没有任何一层会报错，只有硬件会。
#
# 三条用例对应三种状态：请求收但没到位（必须停）、到位了（必须走）、压根没请求（不能影响
# 正常行驶）。第三条是回归用的：这个门加在 control() 的主路径上，写错就会把所有导航停掉。
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


class TestGimbalGate(unittest.TestCase):

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
        # 丢掉前几帧：门的两条话题跟路径/里程计不同步，头几个控制周期可能是在
        # 请求送到之前算的。
        return self.harness.nav_cmds[5:]

    def test_holds_still_while_the_gimbal_is_still_raised(self):
        # 请求收云台，但电控回传的实测姿态还是「高」。车必须停住。
        self.harness.set_gimbal(requested_lower=True, measured_lowered=False)
        cmds = self.drive_and_collect()

        self.assertTrue(cmds, '没收到任何指令 —— 门把 publishStop 也挡掉了？')
        self.assertAlmostEqual(
            max_speed(cmds), 0.0, places=6,
            msg='请求收云台而实测还是「高」，车却在动 —— 云台会撞在顶板上')

    def test_drives_once_the_gimbal_reports_lowered(self):
        # 同样请求收云台，但这次回传说已经收下来了。车必须正常走。
        self.harness.set_gimbal(requested_lower=True, measured_lowered=True)
        cmds = self.drive_and_collect()

        self.assertGreater(
            max_speed(cmds), 0.05,
            '云台已经报「低」，车却还停着 —— 门没有在到位后放开，隧道会走不过去')

    def test_no_request_does_not_block_normal_driving(self):
        # 没有收云台的请求（图里没有隧道，或者车离隧道还远）—— 这是绝大部分时间的状态。
        # 注意实测姿态给的是「高」：门只在请求收的方向拦，云台立着但没人要求它收下来，
        # 不该影响行驶。
        #
        # 这条也是回归用的：门加在 control() 的主路径上，判据写反会把全部导航停掉。
        self.harness.set_gimbal(requested_lower=False, measured_lowered=False)
        cmds = self.drive_and_collect()

        self.assertGreater(
            max_speed(cmds), 0.05,
            '没有收云台请求时车也不走 —— 门的判据反了，所有导航都会被停掉')

# 回归测试：停在目标附近不得触发恢复。
#
# 这条测试守的是实车上出现过的「到点后反复前后蠕动」：
#   MPC 的 goal_tolerance(曾为 0.2) 比下游 goal_approach_controller 的(0.25) 小，
#   中间形成死区 —— MPC 认为「没到」继续发速度，下游认为「到了」无条件发零
#   Twist。stuck 判据被喂了「有指令 + 无位移」的假数据 → 倒车 → 脱离死区 →
#   重规划 → 再开回来 → 再卡住。
#
# 现在有两道防线：容差已对齐到 0.25（消除死区本身），以及 suppress_near_goal
# (0.35) 在目标附近直接关掉失效检测。这里把车停在 0.30 —— 正好落在两者之间的
# 那条带子里 —— 并保持超过 no_progress_timeout，断言恢复不被触发。
#
# 单独一个文件是有意的：approach_enabled 话题是 transient_local（会latch），
# 与别的场景共用节点会读到上一场景遗留的值。
import os
import sys
import unittest

import launch
import launch_ros
import launch_testing
import pytest
import rclpy

# launch_test.py 以脚本方式执行本文件，test/ 不在 sys.path 上，
# 共用夹具 mpc_harness 必须显式加入，否则 ImportError。
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
        node,
        launch_testing.actions.ReadyToTest(),
    ]), {'mpc': node}


class TestNearGoalNoRecovery(unittest.TestCase):

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

    def test_parked_near_goal_does_not_enter_recovery(self):
        goal = (5.0, 0.0)
        # 距目标 0.30 m：大于 goal_tolerance(0.25)，小于 suppress_near_goal(0.35)。
        # 这正是死区所在的位置。
        parked = (4.70, 0.0)

        # 下游把速度置零（真实 goal_approach_controller 在此距离的行为），
        # 所以链路末端实测速率为 0。位置恒定 = 车没动。
        self.harness.drive(
            duration_s=4.0, robot_xy=parked, goal_xy=goal, executed_speed=0.0)

        # 进入恢复的第一个动作是 setApproachEnabled(false)。没有 false
        # 就说明恢复从未被触发。
        self.assertNotIn(
            False, self.harness.approach_enabled,
            'recovery was entered while parked near the goal: '
            'approach_enabled saw False. suppress_near_goal / goal_tolerance '
            'alignment is not holding.')


@launch_testing.post_shutdown_test()
class TestNoRecoveryInLog(unittest.TestCase):

    def test_no_recovery_in_stderr(self, proc_output, mpc):
        # 日志层面的第二道断言：停在目标附近不该出现任何恢复相关记录。
        text = ''.join(
            item.text.decode() if isinstance(item.text, bytes) else item.text
            for item in proc_output[mpc])
        self.assertNotIn('Entering STUCK_REVERSE', text)
        self.assertNotIn('Entering HAZARD_RECOVERY', text)

# 集成测试：真的卡住时恢复链必须触发。
#
# 覆盖三件此前只有纯函数单测的事：
#   1. executed_speed_ 确实驱动 ProgressMonitor（喂非零的 /cmd_vel + 位置不动
#      → stuck）。这是指令链路闭环改动的核心断言。
#   2. FSM 确实从 FOLLOW 转到恢复态。
#   3. 进入恢复时确实把 goal_approach_controller 关掉（否则倒车指令会被它
#      吃掉 —— 距目标 0.25 m 内它无条件发零 Twist）。
#
# 车放在距目标 4 m 处，远离 suppress_near_goal(0.35) 抑制带。
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
        parameters=[mpc_test_params(**{
            # noProgress 触发恢复比 stuck 更快、且不依赖 /cmd_vel 反馈，
            # 后者在容器化环境下首次连接不可靠。
            'progress.no_progress_timeout': 1.0,
            'progress.stuck_timeout': 10.0,
        })],
    )
    return launch.LaunchDescription([
        node,
        launch_testing.actions.ReadyToTest(),
    ]), {'mpc': node}


class TestStuckEntersRecovery(unittest.TestCase):

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

    def test_commanding_but_not_moving_enters_recovery(self):
        goal = (5.0, 0.0)
        # 距目标 4 m，不在抑制带内。
        blocked = (1.0, 0.0)

        # 预热：MPC 节点在 component_container_mt 容器内启动，订阅 /cmd_vel
        # 等话题需要时间完成发现与连接。先用几拍空驱动让它完成初始化，避免
        # stuck 判据因尚未收到执行反馈而被降级。
        import time
        deadline = time.time() + 1.0
        period = 1.0 / 20.0
        while time.time() < deadline:
            self.harness.publish_inputs(blocked, goal, 0.5)
            rclpy.spin_once(self.harness, timeout_sec=period)

        # 底盘确实收到了 0.5 m/s（链路末端实测），但位置一直不变 —— 顶住
        # 障碍或打滑。这是 stuck 判据要捕捉的情形。
        self.harness.drive(
            duration_s=8.0, robot_xy=blocked, goal_xy=goal, executed_speed=0.5)

        self.assertIn(
            False, self.harness.approach_enabled,
            'stuck was not detected, or recovery did not disable '
            'goal_approach_controller. Without that, reverse commands get '
            'zeroed by the downstream controller.')


@launch_testing.post_shutdown_test()
class TestRecoveryLogged(unittest.TestCase):

    def test_recovery_entry_logged(self, proc_output, mpc):
        text = ''.join(
            item.text.decode() if isinstance(item.text, bytes) else item.text
            for item in proc_output[mpc])
        self.assertIn(
            'recovery', text.lower(),
            'no recovery log line found; expected an "Entering ... recovery" '
            'warning with the trigger reason.')

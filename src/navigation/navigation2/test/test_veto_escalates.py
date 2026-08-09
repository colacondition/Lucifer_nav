# Integration test: persistent safety veto escalates to recovery.
#
# A lethal wall on the predicted path causes isPathSafe to fail every tick.
# handleVeto() keeps calling requestReplan (fast path) until veto_recovery_time
# is exceeded, then enters recovery (slow path). This test verifies that
# escalation -- the new code path added in handleVeto() -- actually fires.
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
        parameters=[mpc_test_params(
            **{'recovery.veto_recovery_time': 0.5,
               'progress.no_progress_timeout': 10.0,
               'progress.stuck_timeout': 10.0}
        )],
    )
    return launch.LaunchDescription([
        node, launch_testing.actions.ReadyToTest(),
    ]), {'mpc': node}


class TestVetoEscalates(unittest.TestCase):

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

    def test_lethal_wall_on_path_escalates_to_recovery(self):
        # Lethal wall across the full y range at x in [1.5, 1.7].
        # With expected_speed 1.5 m/s and predict_steps 30 * dt 0.1 the
        # predicted horizon reaches ~1.5 m ahead, so the wall is inside.
        self.harness.set_cost_region(1.5, 1.7, -3.0, 3.0, 100)
        goal = (5.0, 0.0)
        # veto_recovery_time = 0.5 s; drive 2 s -- recovery must trigger
        self.harness.drive(
            duration_s=2.0, robot_xy=(1.0, 0.0), goal_xy=goal,
            executed_speed=0.0)
        self.assertIn(
            False, self.harness.approach_enabled,
            'Safety veto did not escalate to recovery within '
            'veto_recovery_time (0.5 s). handleVeto() slow path is broken.')


@launch_testing.post_shutdown_test()
class TestVetoLog(unittest.TestCase):

    def test_recovery_reason_logged(self, proc_output, mpc):
        text = ''.join(
            item.text.decode() if isinstance(item.text, bytes) else item.text
            for item in proc_output[mpc])
        self.assertIn(
            'recovery', text.lower(),
            'No recovery log line found; expected "Entering ... recovery" from '
            'the veto escalation path.')

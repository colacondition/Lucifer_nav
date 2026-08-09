# Integration test: full HAZARD_RECOVERY -> FOLLOW cycle.
#
# Robot starts inside a hazard band; a lethal wall on the predicted path
# forces the veto to escalate. Because the current position is hazardous,
# handleVeto() enters HAZARD_RECOVERY directly (not StuckReverse).
# findSafePoint selects the only westward exit; the P-controller drives
# there; the node dwells then exits recovery. The whole cycle is verified
# by watching approach_enabled go False then True.
#
# Map design (verified analytically):
#   Hazard band: x in [0.96, 2.0], cost 85 (>= hazard_cost 80, < lethal 99)
#   Lethal wall: x in [1.5, 1.6], all y, cost 100 -- triggers safety veto
#   Robot: (1.0, 0)  -- cell centre 0.975 falls inside band
#   Westward candidate (0.7, 0): ray cells all at x~[0.675,0.975], cost 0
#     -> score = 0 + 20 * 0.3 = 6
#   Eastward candidate (1.3, 0): ray cells at x~[1.025..1.275], cost 85
#     -> score = 85 + 20 * 0.3 = 91
#   West uniquely wins; robot escapes to x < 0.9.
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
               'recovery.hazard_cost': 80,
               'recovery.lethal_cost': 99,
               'progress.no_progress_timeout': 10.0,
               'progress.stuck_timeout': 10.0}
        )],
    )
    return launch.LaunchDescription([
        node, launch_testing.actions.ReadyToTest(),
    ]), {'mpc': node}


class TestHazardRecoveryCycle(unittest.TestCase):

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

    def test_hazard_recovery_enters_and_exits(self):
        # Hazard band makes current position dangerous; lethal wall on the
        # path ahead triggers the veto that escalates to HAZARD_RECOVERY.
        self.harness.set_cost_region(0.96, 2.0, -3.0, 3.0, 85)
        self.harness.set_cost_region(1.5, 1.6, -3.0, 3.0, 100)

        goal = (5.0, 0.0)
        final_pos = self.harness.closed_loop_drive(
            duration_s=9.0, start_xy=(1.0, 0.0), goal_xy=goal)

        self.assertIn(
            False, self.harness.approach_enabled,
            'HAZARD_RECOVERY was never entered (approach_enabled never went '
            'False). Check that the lethal wall triggers the veto and that '
            'isHazardous returns True at (1.0, 0).')
        self.assertIn(
            True, self.harness.approach_enabled,
            'HAZARD_RECOVERY was entered but never exited (approach_enabled '
            'never went True after False). Recovery did not complete the '
            'reach-and-dwell sequence.')
        self.assertLess(
            final_pos[0], 0.9,
            f'Robot ended at x={final_pos[0]:.3f}, expected < 0.9. '
            'findSafePoint should have selected the westward exit.')


@launch_testing.post_shutdown_test()
class TestHazardRecoveryLog(unittest.TestCase):

    def test_recovery_enter_and_exit_logged(self, proc_output, mpc):
        text = ''.join(
            item.text.decode() if isinstance(item.text, bytes) else item.text
            for item in proc_output[mpc])
        self.assertIn('Entering HAZARD_RECOVERY', text)
        self.assertIn('Recovery finished', text)

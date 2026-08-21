# Integration test: downstream override detection.
#
# The node publishes commands on /cmd_vel_nav_raw, but downstream nodes
# (rm_velocity_smoother) can zero them out.
# After override_detect_time the node should log a warning.
# This test is what caught the creep-at-goal bug root cause.
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
            **{'feedback.override_detect_time': 0.5,
               'recovery.veto_recovery_time': 5.0}
        )],
    )
    return launch.LaunchDescription([
        node, launch_testing.actions.ReadyToTest(),
    ]), {'mpc': node}


class TestOverrideDetection(unittest.TestCase):

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

    def test_override_warning_fires_when_executed_lags_published(self):
        goal = (5.0, 0.0)
        # Robot moves forward each tick so MPC publishes non-zero velocity on
        # /cmd_vel_nav_raw.  But /cmd_vel always reports 0 (simulating the
        # downstream controller zeroing the command). After override_detect_time
        # (0.5 s) the warning should fire.
        x = 0.5
        import time
        deadline = time.time() + 2.5
        period = 1.0 / 20.0
        while time.time() < deadline:
            self.harness.publish_inputs((x, 0.0), goal, 0.0)
            rclpy.spin_once(self.harness, timeout_sec=period)
            x += 0.02


@launch_testing.post_shutdown_test()
class TestOverrideInLog(unittest.TestCase):

    def test_override_warning_in_output(self, proc_output, mpc):
        text = ''.join(
            item.text.decode() if isinstance(item.text, bytes) else item.text
            for item in proc_output[mpc])
        self.assertIn(
            'Downstream is zeroing our command', text,
            'Expected override-detection warning not found. '
            'Check that feedback.executed_cmd_topic is being received and '
            'that last_published_speed_ > cmd_epsilon while executed_speed ~ 0.')

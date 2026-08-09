import importlib.util
import os
import tempfile
import unittest
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


LAUNCH_FILE = Path(__file__).resolve().parents[1] / "launch" / "decision.launch.py"


def _node_field(node, field_name):
    # Humble's launch_ros Node has no public getters for these structure fields.
    return getattr(node, f"_Node__{field_name}")


class StandaloneDecisionLaunchTest(unittest.TestCase):
    def assert_launch_configuration(self, substitutions, expected_name):
        self.assertEqual(len(substitutions), 1)
        configuration = substitutions[0]
        self.assertIsInstance(configuration, LaunchConfiguration)
        self.assertEqual(
            "".join(part.text for part in configuration.variable_name), expected_name
        )

    def assert_argument_default(self, argument, context, expected_value):
        self.assertEqual(
            perform_substitutions(context, argument.default_value), expected_value
        )

    def evaluated_parameters(self, parameters, context):
        return {
            perform_substitutions(context, list(name)): perform_substitutions(
                context, list(value)
            )
            for name, value in parameters.items()
        }

    def test_launch_contains_only_decision_execution_chain(self):
        self.assertTrue(LAUNCH_FILE.exists(), f"missing launch file: {LAUNCH_FILE}")

        with tempfile.TemporaryDirectory(prefix="decision_launch_test_") as log_dir:
            os.environ["ROS_LOG_DIR"] = log_dir
            spec = importlib.util.spec_from_file_location("decision_launch", LAUNCH_FILE)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            description = module.generate_launch_description()

        decision_share = Path(get_package_share_directory("decision"))
        bringup_share = Path(get_package_share_directory("bringup"))
        installed_launch = decision_share / "launch" / "decision.launch.py"
        self.assertTrue(
            installed_launch.exists(), f"missing installed launch file: {installed_launch}"
        )

        self.assertEqual(
            [type(entity) for entity in description.entities],
            [DeclareLaunchArgument, DeclareLaunchArgument, DeclareLaunchArgument, Node, Node],
        )

        arguments = description.entities[:3]
        self.assertEqual(
            [argument.name for argument in arguments],
            ["params_file", "log_level", "node_output"],
        )
        context = LaunchContext()
        self.assert_argument_default(
            arguments[0],
            context,
            str(decision_share / "config" / "bt_action_replacement.yaml"),
        )
        self.assert_argument_default(arguments[1], context, "info")
        self.assert_argument_default(arguments[2], context, "screen")

        nodes = description.entities[3:]
        self.assertEqual(
            [(node.node_package, node.node_executable) for node in nodes],
            [
                ("waypoint_editor", "waypoint_follow_executor"),
                ("decision", "bt_action_replacement_node"),
            ],
        )
        self.assertEqual(
            [_node_field(node, "node_name") for node in nodes],
            ["waypoint_follow_executor", "bt_action_replacement"],
        )

        for node in nodes:
            self.assert_launch_configuration(node.output, "node_output")
            node_arguments = _node_field(node, "arguments")
            self.assertEqual(node_arguments[:2], ["--ros-args", "--log-level"])
            self.assert_launch_configuration(node_arguments[2:], "log_level")

        executor_parameters = _node_field(nodes[0], "parameters")
        self.assertEqual(executor_parameters, [])

        decision_parameters = _node_field(nodes[1], "parameters")
        self.assertEqual(len(decision_parameters), 2)
        self.assertIsInstance(decision_parameters[0], ParameterFile)
        self.assert_launch_configuration(decision_parameters[0].param_file, "params_file")
        expected_waypoint_files = {
            f"targets.{target}_waypoint_file": str(
                bringup_share / "config" / "waypoints" / "RMUL" / f"{target}.csv"
            )
            for target in (
                "patrol",
                "center",
                "wait_center",
                "home",
                "wait_home",
                "wait_hp",
            )
        }
        self.assertEqual(
            self.evaluated_parameters(decision_parameters[1], context),
            expected_waypoint_files,
        )


if __name__ == "__main__":
    unittest.main()

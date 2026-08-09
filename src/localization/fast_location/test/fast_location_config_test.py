import math
from pathlib import Path
import unittest

import yaml


class FastLocationConfigTest(unittest.TestCase):
    def test_pcd_alignment_does_not_fallback_to_map_start_pose(self):
        source_path = Path(__file__).parents[1] / "src" / "robot_localization.cpp"
        source = source_path.read_text(encoding="utf-8")

        self.assertNotIn("readMapStartPose", source)
        self.assertNotIn('declare_parameter<std::string>("map_yaml_path"', source)

    def test_pcd_to_map_pose_is_omitted_or_fully_typed(self):
        config_path = Path(__file__).parents[1] / "config" / "fast_location.yaml"
        config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        parameters = config["/**"]["ros__parameters"]

        if "pcd_to_map_pose" not in parameters:
            return

        pose = parameters["pcd_to_map_pose"]
        self.assertEqual(len(pose), 3)
        self.assertTrue(
            all(isinstance(value, float) and math.isfinite(value) for value in pose)
        )


if __name__ == "__main__":
    unittest.main()

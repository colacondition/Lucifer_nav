import math
from pathlib import Path
import unittest

import yaml


def _duplicate_keys(path):
    """Return duplicate YAML keys in a file (PyYAML keeps the last one silently)."""
    duplicates = []

    class StrictLoader(yaml.SafeLoader):
        pass

    def construct_mapping(loader, node, deep=False):
        mapping = {}
        for key_node, value_node in node.value:
            key = loader.construct_object(key_node, deep=deep)
            if key in mapping:
                duplicates.append(key)
            mapping[key] = loader.construct_object(value_node, deep=deep)
        return mapping

    StrictLoader.add_constructor(
        yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, construct_mapping
    )
    yaml.load(path.read_text(encoding="utf-8"), Loader=StrictLoader)
    return duplicates


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

    def test_configs_have_no_duplicate_parameter_keys(self):
        # 回归：bringup/config/common/fast_location.yaml 里曾经出现两个
        # `lost_escape.enable` 键（后一个 false 覆盖前一个 true），跳变门逃生口
        # 被静默关掉 -> LOST 后正确候选被永久拒绝、整车停在 LOST 不动。
        # YAML 重复键不报错，只能在这里守住。
        config_paths = [
            Path(__file__).parents[1] / "config" / "fast_location.yaml",
            Path(__file__).parents[3]
            / "bringup"
            / "config"
            / "common"
            / "fast_location.yaml",
        ]
        for config_path in config_paths:
            if not config_path.exists():
                continue
            self.assertEqual(
                _duplicate_keys(config_path), [], "duplicate keys in %s" % config_path
            )

    def test_lost_escape_timeout_opens_the_jump_gate(self):
        # 跳变门必须留出自动逃生口：enable=true 且 timeout_sec 为有限正值。
        # timeout_sec=0 表示门永久生效 —— 现场无人干预时等于"趴窝到终局"，
        # 需要显式评审，不能靠默认值悄悄退化。
        config_path = (
            Path(__file__).parents[3]
            / "bringup"
            / "config"
            / "common"
            / "fast_location.yaml"
        )
        if not config_path.exists():
            self.skipTest("bringup common config not present")
        parameters = yaml.safe_load(config_path.read_text(encoding="utf-8"))[
            "/**"
        ]["ros__parameters"]

        self.assertTrue(parameters["lost_escape.enable"])
        self.assertGreater(parameters["lost_escape.timeout_sec"], 0.0)
        self.assertIn("global_search_max_map_odom_jump", parameters)

    def test_escape_gate_is_wired_into_the_accept_path(self):
        # 逃生口必须真的接在跳变门上：acceptLocalizationResult 与
        # performGlobalSearch 都要走 jumpGateBlocks / jumpGateIsAdvisory，
        # 否则"配置开了但代码没接"会再次静默趴窝。
        source = (
            Path(__file__).parents[1] / "src" / "robot_localization.cpp"
        ).read_text(encoding="utf-8")

        self.assertIn("jumpGateIsAdvisory()", source)
        self.assertIn("jumpGateBlocks(", source)
        self.assertIn("lost_escape.timeout_sec", source)


if __name__ == "__main__":
    unittest.main()

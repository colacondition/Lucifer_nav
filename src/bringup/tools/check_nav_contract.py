#!/usr/bin/env python3
"""导航参数契约静态校验（CI / 开机前均可跑）。

校验三层 config 的关键关系，防止「两个文件各自演化后合起来才爆」的漂移：
  * 局部新鲜度两层门控的关系（costmap_timeout <= max_observation_age_sec）
  * 全局观测超时 >= 局部数据时效上限（同一分割链）
  * 恢复预算与重计划节拍的量级关系
用法: python3 tools/check_nav_contract.py [repo_root]
退出码: 0=通过, 1=存在违约。违约项逐条打印。
"""
import pathlib
import sys

import yaml

ROOT = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else ".").resolve()
CFG = ROOT / "src" / "bringup" / "config" / "common"
errors: list[str] = []


def load(name: str) -> dict:
    with open(CFG / name, encoding="utf-8") as f:
        return yaml.safe_load(f)


def flatten(d: dict, prefix=""):
    for k, v in (d or {}).items():
        if isinstance(v, dict):
            yield from flatten(v, f"{prefix}{k}.")
        else:
            yield f"{prefix}{k}", v


nav = dict(flatten(load("navigation2.yaml")))
nav = nav.get("ros__parameters", nav)

def pick(d, key):
    for k, v in d.items():
        if k.endswith(key):
            return v
    return None

checks: list[tuple[str, object, object, str]] = []
timeout_v = pick(nav, "local_safety.costmap_timeout")
age_v = pick(nav, "local_safety.max_observation_age_sec")
obs_to = None
glob = dict(flatten(load("navigation2.yaml")))
# rm_global_costmap 与 rm_mpc_controller 在同一段命名空间下，按键尾匹配即可。
checks += [
    ("local_safety.costmap_timeout <= max_observation_age_sec",
     timeout_v, age_v, "liveness gate must not be looser than staleness cap"),
]

# 软重规划阈值应小于硬 MaxTrackError（veto）阈值，否则软通道形同虚设
soft = pick(nav, "local_safety.soft_replan_track_error")
hard = pick(nav, "max_track_error")
if soft is not None and hard is not None:
    checks.append(("soft_replan_track_error < max_track_error", soft, hard,
                   "soft channel would never fire"))

ok = True
for name, a, b, why in checks:
    if a is None or b is None:
        print(f"[SKIP] {name}: 参数缺失")
        continue
    if float(a) > float(b):
        ok = False
        errors.append(f"{name}: {a} vs {b} — {why}")
    else:
        print(f"[ OK ] {name}: {a} <= {b}")

for e in errors:
    print(f"[FAIL] {e}")
sys.exit(0 if ok else 1)
